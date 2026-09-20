#include "YbImu.h"

/*==========================================================================
 * 亚博 YbImu 驱动 —— 位翻转 I2C + 物理量换算 + 失败诊断
 *========================================================================*/

#define IMU_PORT            GPIOB
#define IMU_SCL_PIN         GPIO_Pin_10
#define IMU_SDA_PIN         GPIO_Pin_11

/* 半位延时。72MHz 下大概 4us, 对应 ~110kHz。
   这个模块从几十 kHz 到 400kHz 都能跑, 不用很精确。 */
#define IMU_I2C_DELAY_LOOPS 40

#define SCL_REL()   GPIO_SetBits(IMU_PORT, IMU_SCL_PIN)      /* 开漏高 = 释放, 靠上拉 */
#define SCL_LOW()   GPIO_ResetBits(IMU_PORT, IMU_SCL_PIN)
#define SDA_REL()   GPIO_SetBits(IMU_PORT, IMU_SDA_PIN)
#define SDA_LOW()   GPIO_ResetBits(IMU_PORT, IMU_SDA_PIN)
#define SDA_IS_HIGH()  (GPIO_ReadInputDataBit(IMU_PORT, IMU_SDA_PIN) != Bit_RESET)
#define SCL_IS_HIGH()  (GPIO_ReadInputDataBit(IMU_PORT, IMU_SCL_PIN) != Bit_RESET)

/* IMU 输出是小端, 我们的底盘协议是大端, 这里先按小端合成 */
#define RD_I16_LE(p)  ((int16_t)((uint16_t)(p)[0] | ((uint16_t)(p)[1] << 8)))

/* 读寄存器时 "写寄存器地址 -> 读数据" 这一步用哪种方式。
   多数器件两种都认, 但有些单片机模拟的 I2C 从机不认 repeated start,
   所以第一次失败就自动换成 "先 STOP 再 START" 重试, 之后记住用哪种。 */
static uint8_t use_restart = 1;
static uint8_t pins_ready;

static void I2c_Delay(void)
{
    volatile uint8_t i;
    for (i = 0; i < IMU_I2C_DELAY_LOOPS; i++)
    {
        __NOP();
    }
}

static void I2c_PinsInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = IMU_SCL_PIN | IMU_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;      /* 开漏, 高电平靠外部上拉 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IMU_PORT, &gpio);

    SCL_REL();
    SDA_REL();
    pins_ready = 1;
}

/* 等 SCL 真的到高电平(从机可能拉伸时钟)。返回 0 = 超时 */
static uint8_t SCL_WaitHigh(void)
{
    uint16_t timeout = 2000;

    while ((!SCL_IS_HIGH()) && (--timeout != 0))
    {
    }
    return (uint8_t)(timeout != 0);
}

static uint8_t I2c_Start(void)
{
    SDA_REL();
    SCL_REL();
    I2c_Delay();
    if (!SCL_WaitHigh()) return 1;
    if (!SDA_IS_HIGH())  return 2;      /* SDA 被拉死, 总线有问题 */
    SDA_LOW();
    I2c_Delay();
    SCL_LOW();
    I2c_Delay();
    return 0;
}

static void I2c_Stop(void)
{
    SDA_LOW();
    I2c_Delay();
    SCL_REL();
    I2c_Delay();
    SDA_REL();
    I2c_Delay();
}

/* I2C 总线恢复。
 *
 * 从机如果在一次事务中途被打断(比如主机读了一半就不读了、或者错误路径上
 * 忘了发 STOP), 它可能一直在等下一个时钟, 把 SDA 拉着不放。这时候后续
 * 所有起始条件都会失败, 整个总线卡死, 直到给从机断电。
 *
 * 标准救法: 主机额外补 9 个 SCL 脉冲, 让从机把剩下的位吐完并释放 SDA,
 * 再补一个 STOP。空闲时 SDA 已经是高的就直接返回, 不花时间。
 *
 * ★ 这个是踩出来的: 给 IMU 加寄存器体检、开始读 0x16/0x26 这种长块之后,
 *   陀螺仪的诊断码从"陀螺仪恒 0"变成了"总线死", 而且永不恢复 ——
 *   就是因为 `YbImu_ReadRegs` 的两条错误分支 return 前没有 I2c_Stop()。
 */
static void I2c_BusRecover(void)
{
    uint8_t i;

    SDA_REL();
    SCL_REL();
    I2c_Delay();

    if (SDA_IS_HIGH()) return;          /* 没被拉死, 不用救 */

    for (i = 0; i < 9; i++)
    {
        SCL_LOW();
        I2c_Delay();
        SCL_REL();
        I2c_Delay();
        if (SDA_IS_HIGH()) break;       /* 从机松手了 */
    }
    I2c_Stop();
}

/* 返回 1 = 从机应答 */
static uint8_t I2c_WriteByte(uint8_t value)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0; i < 8; i++)
    {
        if (value & 0x80) SDA_REL();
        else              SDA_LOW();
        I2c_Delay();
        SCL_REL();
        I2c_Delay();
        SCL_LOW();
        I2c_Delay();
        value = (uint8_t)(value << 1);
    }

    SDA_REL();                          /* 释放 SDA, 让从机拉低表示 ACK */
    I2c_Delay();
    SCL_REL();
    I2c_Delay();
    ack = (uint8_t)(!SDA_IS_HIGH());    /* 低电平 = ACK */
    SCL_LOW();
    I2c_Delay();
    return ack;
}

/* ack=1 读完回 ACK(还要继续读), ack=0 回 NACK(最后一个字节) */
static uint8_t I2c_ReadByte(uint8_t ack)
{
    uint8_t i;
    uint8_t value = 0;

    SDA_REL();
    for (i = 0; i < 8; i++)
    {
        value = (uint8_t)(value << 1);
        SCL_REL();
        I2c_Delay();
        if (SDA_IS_HIGH()) value |= 1;
        SCL_LOW();
        I2c_Delay();
    }

    if (ack) SDA_LOW();                 /* ACK */
    else     SDA_REL();                 /* NACK */
    I2c_Delay();
    SCL_REL();
    I2c_Delay();
    SCL_LOW();
    I2c_Delay();
    SDA_REL();
    return value;
}

/*=========================== 读写寄存器 ===========================*/
uint8_t YbImu_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t err;

    if (!pins_ready) I2c_PinsInit();

    /* 上一轮如果被卡住, 先救回来再发起始条件 */
    I2c_BusRecover();

    /* 第一段: 写寄存器地址 */
    err = I2c_Start();
    if (err)
    {
        I2c_Stop();                     /* ★ 出错也必须释放总线 */
        return (err == 2) ? YBIMU_ST_BUS_DEAD : YBIMU_ST_SCL_LOW;
    }
    if (!I2c_WriteByte((uint8_t)(YBIMU_I2C_ADDR << 1))) { I2c_Stop(); return YBIMU_ST_NO_ACK; }
    if (!I2c_WriteByte(reg))                            { I2c_Stop(); return YBIMU_ST_NO_ACK; }

    /* 第二段: 读数据。两种方式, 记住哪种能用 */
    if (use_restart)
    {
        err = I2c_Start();
        if (err) { I2c_Stop(); return YBIMU_ST_BUS_DEAD; }   /* ★ 补 STOP */
    }
    else
    {
        I2c_Stop();
        I2c_Delay();
        err = I2c_Start();
        if (err) { I2c_Stop(); return YBIMU_ST_BUS_DEAD; }   /* ★ 补 STOP */
    }

    if (!I2c_WriteByte((uint8_t)((YBIMU_I2C_ADDR << 1) | 1))) { I2c_Stop(); return YBIMU_ST_READ_ERR; }

    for (i = 0; i < len; i++)
    {
        buf[i] = I2c_ReadByte((uint8_t)(i < (uint8_t)(len - 1)));
    }
    I2c_Stop();
    return YBIMU_ST_OK;
}

/*============================= 诊断 ===============================*/
uint8_t YbImu_BusCheck(void)
{
    if (!pins_ready) I2c_PinsInit();

    SCL_REL();
    SDA_REL();
    I2c_Delay();
    I2c_Delay();

    /* 开漏输出 + 外部上拉: 释放以后两条线都应该是高。
       如果不是高, 说明没有上拉电阻、或者线没接上/对地短路。 */
    if (!SCL_IS_HIGH()) return YBIMU_ST_SCL_LOW;
    if (!SDA_IS_HIGH()) return YBIMU_ST_SDA_LOW;

    {
        uint8_t err = I2c_Start();
        I2c_Stop();
        if (err) return YBIMU_ST_BUS_DEAD;
    }
    return YBIMU_ST_OK;
}

uint8_t YbImu_Scan(void)
{
    uint8_t addr;

    if (!pins_ready) I2c_PinsInit();

    for (addr = 0x08; addr <= 0x77; addr++)
    {
        if (I2c_Start() != 0) return 0;
        if (I2c_WriteByte((uint8_t)(addr << 1)))
        {
            I2c_Stop();
            return addr;                /* 找到第一个应答的器件 */
        }
        I2c_Stop();
    }
    return 0;
}

/*=========================== 寄存器体检 ===========================
 * 每个功能块单独读一遍, 光看"有没有非零字节"就够判断它活没活。
 * 只在诊断周期(1Hz)调用, 多花几毫秒无所谓。
 *=================================================================*/
static uint8_t AnyNonZero(const uint8_t *p, uint8_t n)
{
    uint8_t i;
    for (i = 0; i < n; i++)
    {
        if (p[i] != 0) return 1;
    }
    return 0;
}

void YbImu_Probe(YbImu_Probe_t *out)
{
    uint8_t buf[6];
    uint8_t err;

    out->ver[0] = out->ver[1] = out->ver[2] = 0;
    out->ver_ok = out->gyro_ok = out->mag_ok = 0;
    out->quat_ok = out->euler_ok = 0;
    out->euler_yaw = 0.0f;

    if (YbImu_ReadRegs(YBIMU_REG_VERSION, out->ver, 3) == YBIMU_ST_OK)
    {
        out->ver_ok = 1;
    }

    err = YbImu_ReadRegs(YBIMU_REG_GYRO, buf, 6);
    if (err == YBIMU_ST_OK) out->gyro_ok = AnyNonZero(buf, 6);

    err = YbImu_ReadRegs(YBIMU_REG_MAG, buf, 6);
    if (err == YBIMU_ST_OK) out->mag_ok = AnyNonZero(buf, 6);

    /* ★ 这里**故意不读** 0x16(四元数, 16 字节)和 0x26(欧拉角, 12 字节)。
     *
     * 加了这两个长块读之后, IMU 诊断码从"陀螺仪恒 0"变成了"总线死"
     * (YBIMU_ST_BUS_DEAD, 起始条件拉不起来), 而且一直不恢复 —— 连加速度
     * 都读不到了, 比不做体检还糟。时间点和"开始读长块"完全吻合。
     *
     * 所以先只读 3/6/6 字节这些短块: 一样能回答关键问题
     *   "只有陀螺仪坏"  -> 版本/磁力活, 陀螺死
     *   "模块只剩基本模式" -> 只有版本活
     * 代价是拿不到欧拉角那个备选角速度来源, 但**不能为了诊断把器件搞死**。
     *
     * 真要再试长块, 必须先确认 0x16/0x26 在这颗模块上确实存在 ——
     * 头文件那份寄存器表可能来自另一个批次的 YbImu。
     */
    out->quat_ok = 0;
    out->euler_ok = 0;
}

/*=========================== 对外接口 =============================*/
uint8_t YbImu_ReadVersion(uint8_t out[3])
{
    return YbImu_ReadRegs(YBIMU_REG_VERSION, out, 3);
}

/* 返回 YBIMU_ST_OK 或 YBIMU_ST_GYRO_ZERO(加速度能读但角速度恒 0) */
uint8_t YbImu_ReadMotion(float accel_g[3], float gyro_rad_s[3])
{
    uint8_t raw[6];
    uint8_t err;
    uint8_t gyro_all_zero;

    /* ★ 分两次读, 不要合并成一次 12 字节 —— 见 YbImu.h 里的说明 */
    err = YbImu_ReadRegs(YBIMU_REG_ACCEL, raw, 6);
    if (err != YBIMU_ST_OK)
    {
        /* 这次用的方式不通, 换另一种再试一次, 并记住能用的那种。
           有些单片机模拟的从机不认 repeated start, 只认 "STOP 后再 START"。 */
        use_restart = (uint8_t)(!use_restart);
        err = YbImu_ReadRegs(YBIMU_REG_ACCEL, raw, 6);
        if (err != YBIMU_ST_OK) return err;
    }

    /* 加速度: 满量程 ±16g, 原始值 -> g */
    accel_g[0] = (float)RD_I16_LE(&raw[0]) * (16.0f / 32767.0f);
    accel_g[1] = (float)RD_I16_LE(&raw[2]) * (16.0f / 32767.0f);
    accel_g[2] = (float)RD_I16_LE(&raw[4]) * (16.0f / 32767.0f);

    err = YbImu_ReadRegs(YBIMU_REG_GYRO, raw, 6);
    if (err != YBIMU_ST_OK) return err;

    /* 角速度: 满量程 ±2000dps, 原始值 -> rad/s */
    {
        const float ratio = (2000.0f / 32767.0f) * (3.14159265f / 180.0f);
        gyro_rad_s[0] = (float)RD_I16_LE(&raw[0]) * ratio;
        gyro_rad_s[1] = (float)RD_I16_LE(&raw[2]) * ratio;
        gyro_rad_s[2] = (float)RD_I16_LE(&raw[4]) * ratio;
    }

    /* 三个轴一起恒为 0 不正常 —— 真实陀螺仪静止时也有噪声。
       报上去让上位机看得见, 不要静悄悄地给个 0。 */
    gyro_all_zero = (uint8_t)((raw[0] == 0) && (raw[1] == 0) &&
                              (raw[2] == 0) && (raw[3] == 0) &&
                              (raw[4] == 0) && (raw[5] == 0));
    return gyro_all_zero ? YBIMU_ST_GYRO_ZERO : YBIMU_ST_OK;
}
