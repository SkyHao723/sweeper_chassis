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

    /* 第一段: 写寄存器地址 */
    err = I2c_Start();
    if (err)
    {
        return (err == 2) ? YBIMU_ST_BUS_DEAD : YBIMU_ST_SCL_LOW;
    }
    if (!I2c_WriteByte((uint8_t)(YBIMU_I2C_ADDR << 1))) { I2c_Stop(); return YBIMU_ST_NO_ACK; }
    if (!I2c_WriteByte(reg))                            { I2c_Stop(); return YBIMU_ST_NO_ACK; }

    /* 第二段: 读数据。两种方式, 记住哪种能用 */
    if (use_restart)
    {
        err = I2c_Start();
        if (err) return YBIMU_ST_BUS_DEAD;
    }
    else
    {
        I2c_Stop();
        I2c_Delay();
        err = I2c_Start();
        if (err) return YBIMU_ST_BUS_DEAD;
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

/* 模块的 float 是小端 IEEE754。Cortex-M3 本身就是小端, 所以直接把
   4 个字节按小端拼成 uint32 再按位转成 float (不用指针强转, 免得踩对齐)。 */
static float RdFloatLE(const uint8_t *p)
{
    union { uint32_t u; float f; } cv;

    cv.u = ((uint32_t)p[0])       | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return cv.f;
}

void YbImu_Probe(YbImu_Probe_t *out)
{
    uint8_t buf[16];
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

    err = YbImu_ReadRegs(YBIMU_REG_QUAT, buf, 16);
    if (err == YBIMU_ST_OK) out->quat_ok = AnyNonZero(buf, 16);

    err = YbImu_ReadRegs(YBIMU_REG_EULER, buf, 12);
    if (err == YBIMU_ST_OK)
    {
        out->euler_ok = AnyNonZero(buf, 12);
        out->euler_yaw = RdFloatLE(&buf[8]);    /* float×3: roll, pitch, yaw */
    }
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
