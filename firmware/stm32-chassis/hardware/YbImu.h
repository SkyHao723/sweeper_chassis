#ifndef YBIMU_H
#define YBIMU_H

#include "stm32f10x.h"

/*==========================================================================
 * 亚博 (Yahboom) YbImu 姿态传感器 —— I2C 接口
 *
 *   SCL = PB10      SDA = PB11      从机地址 0x23
 *   VCC 必须接 3.3V (模块的 I2C 上拉通常拉到它自己的 VCC 上, 给 5V 会灌进 IO)
 *
 * 寄存器 (1 字节地址, 数据小端):
 *   0x01 版本号          3 字节
 *   0x04 原始加速度      6 字节  int16×3, 单位 g,   满量程 ±16g
 *   0x0A 原始角速度      6 字节  int16×3, 单位 rad/s, 满量程 ±2000dps
 *   0x10 原始磁力计      6 字节  int16×3, 单位 uT,  满量程 ±800uT
 *   0x16 四元数         16 字节  float×4
 *   0x26 欧拉角         12 字节  float×3, 单位 rad
 *
 * 用软件(位翻转) I2C, 不用 STM32 的硬件 I2C 外设:
 *   STM32F103 的 I2C 有已知 errata 会死锁, 标准库的 I2C_CheckEvent 又是
 *   无超时的忙等 —— 一旦卡住整个固件就挂了。位翻转没有状态机, 好调也好救。
 *========================================================================*/

#define YBIMU_I2C_ADDR      0x23
#define YBIMU_REG_VERSION   0x01
#define YBIMU_REG_ACCEL     0x04    /* 6 字节 int16×3, 单位 g,     满量程 ±16g  */
#define YBIMU_REG_GYRO      0x0A    /* 6 字节 int16×3, 单位 rad/s, 满量程 ±2000dps */

/* ★ 加速度和角速度必须**分两次读**, 不能一次读 12 字节。
 *   虽然 0x04+6 == 0x0A 地址是连续的, 但模块的地址指针不跨功能块自增:
 *   一次读 12 字节的话前 6 字节(加速度)正常, 后 6 字节(角速度)恒为 0。
 *   厂商的 Python 库也是分两次读的。这个坑踩过, 别再"优化"回去。 */

/*------------------------- 诊断状态码 ---------------------------*/
#define YBIMU_ST_OK         0   /* 正常 */
#define YBIMU_ST_SCL_LOW    1   /* 空闲时 SCL 不是高 —— 缺上拉 / 没接 / 对地短路 */
#define YBIMU_ST_SDA_LOW    2   /* 空闲时 SDA 不是高 —— 缺上拉 / 没接 / 对地短路 */
#define YBIMU_ST_BUS_DEAD   3   /* 起始条件失败, 总线被别人拉着 */
#define YBIMU_ST_NO_ACK     4   /* 地址 0x23 不应答 */
#define YBIMU_ST_READ_ERR   5   /* 地址应答了但数据阶段出错 */
#define YBIMU_ST_GYRO_ZERO  6   /* 加速度正常但角速度恒为 0 */

/* 返回 0 = 成功, 非 0 = 失败 */
uint8_t YbImu_ReadMotion(float accel_g[3], float gyro_rad_s[3]);

/*----------------------------- 诊断 -----------------------------*/
uint8_t YbImu_BusCheck(void);   /* 返回 YBIMU_ST_* : 只看总线空闲电平和起始条件 */
uint8_t YbImu_Scan(void);       /* 扫 0x08~0x77, 返回第一个应答的地址, 0 = 没扫到 */
uint8_t YbImu_ReadVersion(uint8_t out[3]);

/* 底层, 一般不用直接调 */
uint8_t YbImu_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);

#endif
