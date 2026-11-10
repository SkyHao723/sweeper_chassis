/********************************** (C) COPYRIGHT *******************************
* File Name          : GamepadHost.C
* Description        : CH554G USB 主机 —— Xbox 360 无线手柄接收器 -> UART 转发给 STM32
*
*   用途：CH554G 作 USB Host，接一个 Xbox 360 / XInput 兼容的无线接收器
*         （VID=045E PID=028E，HID 接口，输入报告 15 字节，约 500 Hz），
*         把每次收到的 15 字节输入报告**原样**从 UART 发给 STM32（115200 8N1）。
*
*   为什么这样设计：
*     1. STM32 侧（firmware/stm32-chassis-v2/user/rc.c）已经按"裸 15 字节"解析，
*        不加帧头/校验，所以这里一个字都不能多、不能少。
*     2. 接收器空闲时也会以 ~500 Hz 重复发送同一帧，但 STM32 侧有 300 ms
*        在线超时，所以这里必须**持续发**（不能像 WCH 官方例程那样"内容变化才发"）。
*        做法：USB 侧收到什么就缓存下来，主循环按 GAMEPAD_SEND_MS 的节奏固定重发。
*     3. 不允许在中断里发 UART：15 字节 @115200 约 1.3 ms，会打乱 USB 轮询。
*
*   硬件连接（与 STM32 的连线，按实际焊接：P3.0 / P3.1）：
*     CH554G P3.1 (TXD0)  ->  STM32 PB11 (USART3_RX)
*     CH554G P3.0 (RXD0)  <-  STM32 PB10 (USART3_TX)   [本固件不发，只收]
*     CH554G GND          ->  STM32 GND
*     CH554G V33/VCC      ->  3.3V
*     注意：数据口用的 P3.0/P3.1 是 UART0；CH554 的串口 ISP 下载口是 **UART1 =
*           P1.6(RXD1)/P1.7(TXD1)**（板上丝印 MISO/SCK），和数据口分开，互不影响。
*           烧录方法见 README §4（USB-TTL 接 P1.6/P1.7/GND，WCHISPStudio 选"串口下载"）。
*
*   编译：Keil C51（uVision4/5），工程文件 CH554Gamepad.uvproj，输出 CH554Gamepad.hex
*   烧录：WCHISPStudio / WCHISPTool，USB 下载方式（见 README.md）
********************************************************************************/
#include "..\Public\CH554.H"
#include "..\Public\Debug.H"
#include "..\USB\USB_LIB\USBHOST.H"
#include <string.h>

#pragma  NOAREGS

/*---------------------------------------------------------------------------
 * 可调参数
 *-------------------------------------------------------------------------*/
#define GAMEPAD_REPORT_SIZE     15      /* 手柄输入报告长度（字节），STM32 侧写死 15 */
#define GAMEPAD_SEND_MS          8      /* 转发节奏：8 ms = 125 Hz（STM32 侧 300 ms 超时）*/
#define GAMEPAD_UART0_ALTER      0      /* 0 = TXD0/RXD0 在 P3.1/P3.0；1 = 改到 P1.3/P1.2 */
#define GAMEPAD_DEBUG_UART1      0      /* 1 = 用 UART1(P1.6/P1.7) 打调试信息（与数据口分开） */

/*---------------------------------------------------------------------------
 * USB 主机库要求由应用提供的数据（保持与 WCH 官方例程一致）
 *-------------------------------------------------------------------------*/
/* 获取设备描述符 */
UINT8C  SetupGetDevDescr[] = { USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof( USB_DEV_DESCR ), 0x00 };

/* 获取配置描述符 */
UINT8C  SetupGetCfgDescr[] = { USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00 };

/* 设置USB地址 */
UINT8C  SetupSetUsbAddr[] = { USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* 设置USB配置 */
UINT8C  SetupSetUsbConfig[] = { USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* 设置USB接口配置 */
UINT8C  SetupSetUsbInterface[] = { USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* 清除端点STALL */
UINT8C  SetupClrEndpStall[] = { USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* 获取HUB描述符 */
UINT8C  SetupGetHubDescr[] = { HUB_GET_HUB_DESCRIPTOR, HUB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_HUB, 0x00, 0x00, sizeof( USB_HUB_DESCR ), 0x00 };

/* 设置 HID 空闲不发包 */
UINT8C  SetupSetHIDIdle[] = { 0x21, HID_SET_IDLE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* 获取HID设备报表描述符 */
UINT8C  SetupGetHIDDevReport[] = { 0x81, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_REPORT, 0x00, 0x00, 0xFF, 0xFF };

/* 打印机类命令（本应用不用，但库里有引用，必须给个定义） */
UINT8C  XPrinterReport[] = { 0xA1, 0, 0x00, 0, 0x00, 0x00, 0xF1, 0x03 };

/* AOA 相关（本应用不用，但库里有引用，必须给定义） */
UINT8C  GetProtocol[] = { 0xc0, 0x33, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 };
UINT8C  TouchAOAMode[] = { 0x40, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
UINT8C  Sendlen[] = { 0, 4, 16, 35, 39, 53, 67 };
UINT8C  StringID[] = { 'W','C','H',0x00,
                       'W','C','H','U','A','R','T','D','e','m','o',0x00,
                       0x57,0x43,0x48,0x20,0x41,0x63,0x63,0x65,0x73,0x73,0x6f,0x72,0x79,0x20,0x54,0x65,0x73,0x74,0x00,
                       '1','.','0',0x00,
                       0x68,0x74,0x74,0x70,0x3a,0x2f,0x2f,0x77,0x63,0x68,0x2e,0x63,0x6e,0x00,
                       0x57,0x43,0x48,0x41,0x63,0x63,0x65,0x73,0x73,0x6f,0x72,0x79,0x31,0x00 };
UINT8C  SetStringID[] = { 0x40,0x34,0x00,0x00,0x00,0x00,0x04,0x00,
                          0x40,0x34,0x00,0x00,0x01,0x00,12,0x00,
                          0x40,0x34,0x00,0x00,0x02,0x00,19,0x00,
                          0x40,0x34,0x00,0x00,0x03,0x00,4,0x00,
                          0x40,0x34,0x00,0x00,0x04,0x00,0x0E,0x00,
                          0x40,0x34,0x00,0x00,0x05,0x00,0x0E,0x00 };

UINT8X  UsbDevEndp0Size;                        /* USB 设备端点0最大包长 */
UINT8X  RxBuffer[ MAX_PACKET_SIZE ] _at_ 0x0000;/* IN, must be even address */
UINT8X  TxBuffer[ MAX_PACKET_SIZE ] _at_ 0x0040;/* OUT, must be even address */

UINT8          Set_Port = 0;
_RootHubDev    xdata ThisUsbDev;
_DevOnHubPort  xdata DevOnHubPort[ HUB_MAX_PORTS ];
bit            FoundNewDev;

/*---------------------------------------------------------------------------
 * 本应用状态
 *-------------------------------------------------------------------------*/
UINT8X  g_last_report[ GAMEPAD_REPORT_SIZE ];   /* 最近一次手柄报告（原始字节） */
UINT8   g_have_report;                          /* 是否已经收到过报告 */
volatile UINT8 g_tick_ms;                       /* Timer0 1 ms 节拍（8 位，回绕安全） */
UINT8   g_last_send_ms;                         /* 上次发送的节拍 */
UINT16  g_sent_frames;                          /* 已转发帧数（调试用） */
UINT16  g_usb_packets;                          /* USB IN 成功包数（调试用） */
UINT8   g_other_len;                            /* 最近一次非 15 字节包的长度（调试用） */

/*---------------------------------------------------------------------------
 * 数据口：UART0，115200 8N1，默认引脚 P3.1(TXD0)/P3.0(RXD0)
 *   P3.1 (TXD0) -> STM32 PB11 (USART3_RX)      [本固件只发]
 *   P3.0 (RXD0) <- STM32 PB10 (USART3_TX)      [本固件不收]
 *   注意：P3.0/P3.1 同时是 CH554 的 ISP 下载口，烧录时要么走 USB-A 口，
 *         要么让 STM32 让开这两根线（见 firmware/ch554-gamepad/README.md）
 *-------------------------------------------------------------------------*/
static void DataUart_Init( void )
{
    mInitSTDIO();                               /* 官方 Debug.C：UART0 模式1，T1 做波特率，115200 */

#if ( GAMEPAD_UART0_ALTER )
    PIN_FUNC |= bUART0_PIN_X;                   /* TXD0/RXD0 改到 P1.3/P1.2 */
    P1_MOD_OC &= ~0x08;  P1_DIR_PU |=  0x08;    /* P1.3 TXD 推挽输出 */
    P1_MOD_OC |=  0x04;  P1_DIR_PU |=  0x04;    /* P1.2 RXD 准双向带上拉 */
#else
    PIN_FUNC &= ~bUART0_PIN_X;                  /* 默认 P3.1/P3.0 */
    P3_MOD_OC &= ~0x02;  P3_DIR_PU |=  0x02;    /* P3.1 TXD 推挽输出 */
    P3_MOD_OC |=  0x01;  P3_DIR_PU |=  0x01;    /* P3.0 RXD 准双向带上拉 */
#endif
    /* 只动低 2 位，绝不碰 P3.6/P3.7（USB D+/D-） */
    TI = 0;                                     /* mInitSTDIO 结束时 TI=1，这里清掉 */
}

static void DataUart_SendByte( UINT8 dat )
{
    TI = 0;                                     /* 必须先清，否则第一字节会立即返回（漏发） */
    SBUF = dat;
    while ( TI == 0 );
    TI = 0;
}

static void DataUart_SendBytes( const UINT8X *buf, UINT8 len )
{
    UINT8 i;
    for ( i = 0; i < len; i++ )
    {
        DataUart_SendByte( buf[ i ] );
    }
}

#if ( GAMEPAD_DEBUG_UART1 )
/* 调试口：UART1，P1.7(TXD1)/P1.6(RXD1)，和数据口分开，不影响手柄转发 */
static void Debug_Uart_Init( void )
{
    U1SM0  = 0;                                 /* 8 位数据位 */
    U1SMOD = 1;                                 /* 快速模式：Fsys/16/(256-SBAUD1) */
    U1REN  = 0;
    SBAUD1 = 0 - ( FREQ_SYS / 16 / UART1_BUAD );
    U1TI   = 0;
    PIN_FUNC &= ~bUART1_PIN_X;                  /* 默认 P1.7/P1.6 */
    P1_MOD_OC &= ~0x80;                         /* P1.7 TXD1 推挽输出 */
    P1_DIR_PU |=  0x80;
}

static void Debug_PutChar( UINT8 c )
{
    U1TI = 0;
    SBUF1 = c;
    while ( U1TI == 0 );
    U1TI = 0;
}

static void Debug_Hex8( UINT8 v )
{
    UINT8 hi = (UINT8)( v >> 4 );
    UINT8 lo = (UINT8)( v & 0x0F );
    Debug_PutChar( hi < 10 ? (UINT8)( '0' + hi ) : (UINT8)( 'A' + hi - 10 ) );
    Debug_PutChar( lo < 10 ? (UINT8)( '0' + lo ) : (UINT8)( 'A' + lo - 10 ) );
}
#endif

/*---------------------------------------------------------------------------
 * Timer0：1 ms 节拍（Fsys=24 MHz -> 计数时钟 2 MHz，1 ms = 2000 个计数）
 *-------------------------------------------------------------------------*/
static void Timer0_Init( void )
{
    TMOD = ( TMOD & 0xF0 ) | 0x01;               /* T0 = 模式1，16 位定时 */
    TH0  = 0xF8;                                 /* 65536 - 2000 = 0xF830 */
    TL0  = 0x30;
    ET0  = 1;
    EA   = 1;
    TR0  = 1;
}

void Timer0_Isr( void ) interrupt 1
{
    TH0 = 0xF8;                                  /* 手动重装 */
    TL0 = 0x30;
    g_tick_ms++;
}

/*---------------------------------------------------------------------------
 * 手柄：找到 HID 设备 -> 取第一个中断 IN 端点 -> 读报告
 *-------------------------------------------------------------------------*/
static UINT16 Gamepad_Loc = 0xFFFF;               /* 设备所在端口（0xFFFF = 没找到） */
static UINT8  Gamepad_Endp;                       /* 中断 IN 端点号 + 同步标志位(bit7) */

static void Gamepad_ScanDevice( void )
{
    UINT16 loc;
    UINT8  endp;

    loc = SearchTypeDevice( USB_DEV_CLASS_HID );  /* 手柄：HID 类，subclass=0/protocol=0 */
    if ( loc == 0xFFFF )
    {
        /* 手柄/接收器不在了：停发，让 STM32 侧 300 ms 超时把遥控判为离线，
           否则会把最后一帧一直重发，STM32 以为手柄还在。 */
        Gamepad_Loc   = 0xFFFF;
        g_have_report = 0;
        return;
    }

    loc = (UINT8)loc;                             /* CH554 只有一个 USB 口，取低 8 位 */
    if ( loc )
    {
        endp = DevOnHubPort[ loc - 1 ].GpVar[ 0 ];
    }
    else
    {
        endp = ThisUsbDev.GpVar[ 0 ];
    }

    Gamepad_Loc  = loc;
    Gamepad_Endp = endp;                          /* bit7=0 表示下一次用 DATA0 */
}

static void Gamepad_Poll( void )
{
    UINT8 s, endp, len;

    if ( Gamepad_Loc == 0xFFFF ) return;

    endp = Gamepad_Endp;
    if ( ( endp & USB_ENDP_ADDR_MASK ) == 0 ) return;         /* 没有中断 IN 端点 */

    SelectHubPort( (UINT8)Gamepad_Loc );                     /* 选中端口、设置速度和地址 */

    s = USBHostTransact( (UINT8)( USB_PID_IN << 4 | ( endp & 0x7F ) ),
                         (UINT8)( ( endp & 0x80 ) ? ( bUH_R_TOG | bUH_T_TOG ) : 0 ),
                         0 );                                /* 0 = NAK 不重试 */
    if ( s == ERR_SUCCESS )
    {
        endp ^= 0x80;                                        /* 翻转同步标志位 */
        if ( Gamepad_Loc )
        {
            DevOnHubPort[ Gamepad_Loc - 1 ].GpVar[ 0 ] = endp;
        }
        else
        {
            ThisUsbDev.GpVar[ 0 ] = endp;
        }

        len = USB_RX_LEN;
        g_usb_packets++;
        if ( len == GAMEPAD_REPORT_SIZE )
        {
            memcpy( g_last_report, RxBuffer, GAMEPAD_REPORT_SIZE );
            g_have_report = 1;
        }
        else if ( len )
        {
            g_other_len = len;                               /* 记下来，方便排查 */
        }
    }
    else if ( s != ( USB_PID_NAK | ERR_USB_TRANSFER ) )
    {
        /* 真正出错（不是 NAK）：设备可能拔掉了，下一轮重新扫描 */
        if ( Gamepad_Loc )
        {
            DevOnHubPort[ Gamepad_Loc - 1 ].GpVar[ 0 ] &= 0x7F;
        }
        else
        {
            ThisUsbDev.GpVar[ 0 ] &= 0x7F;
        }
        Gamepad_Loc = 0xFFFF;
        g_have_report = 0;
    }

    SetUsbSpeed( 1 );                                        /* 默认全速 */
}

/*---------------------------------------------------------------------------
 * 主循环
 *-------------------------------------------------------------------------*/
void main( void )
{
    UINT8  s;

    CfgFsys( );                                 /* 系统时钟 24 MHz（内部） */
    mDelaymS( 50 );

#if ( GAMEPAD_DEBUG_UART1 )
    Debug_Uart_Init( );
#endif

    DataUart_Init( );                           /* 先把手柄数据口准备好（UART0 P3.1/P3.0）*/
    Timer0_Init( );

    InitUSB_Host( );
    FoundNewDev = 0;
    g_have_report = 0;

    while ( 1 )
    {
        /* 1. 插入/拔出检测 */
        if ( UIF_DETECT )
        {
            UIF_DETECT = 0;
            s = AnalyzeRootHub( );
            if ( s == ERR_USB_CONNECT ) FoundNewDev = 1;
        }

        /* 2. 新设备枚举 */
        if ( FoundNewDev )
        {
            FoundNewDev = 0;
            mDelaymS( 20 );                     /* 等设备上电稳定 */
            (void)EnumAllRootDevice( );
        }
        (void)EnumAllHubPort( );                /* 外接 HUB 下的设备（备用） */

        /* 3. 定位手柄并读一包 */
        Gamepad_ScanDevice( );
        Gamepad_Poll( );

        /* 4. 固定节奏把最近一包原样转发给 STM32
              为什么不是"收到就发"：接收器空闲时同一帧会重发，STM32 侧靠
              帧率判断在线；周期发送最简单也最稳。 */
        if ( g_have_report )
        {
            if ( (UINT8)( g_tick_ms - g_last_send_ms ) >= GAMEPAD_SEND_MS )
            {
                g_last_send_ms = g_tick_ms;
                DataUart_SendBytes( g_last_report, GAMEPAD_REPORT_SIZE );
                g_sent_frames++;
            }
        }
    }
}
