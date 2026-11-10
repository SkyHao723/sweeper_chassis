/********************************** (C) COPYRIGHT *******************************
* File Name          : GamepadHost.C
* Description        : CH554G USB 主机 —— Xbox 360 无线手柄接收器 -> UART 转发给 STM32
*
*   用途：CH554G 作 USB Host，接一个 Xbox 360 / XInput 兼容的无线接收器
*         （VID=045E PID=028E，输入报告 15 字节，约 500 Hz），
*         把收到的 15 字节输入报告**原样**从 UART 发给 STM32（115200 8N1）。
*
*   为什么这样设计：
*     1. STM32 侧（firmware/stm32-chassis-v2/user/rc.c）按"裸 15 字节"解析，
*        不加帧头/校验，所以这里一个字都不能多、不能少。
*     2. STM32 侧有 300 ms 在线超时，所以必须**持续发**（不能像 WCH 官方例程
*        那样"内容变化才发"）。做法：USB 侧收到什么就缓存，主循环按
*        GAMEPAD_SEND_MS 的节奏固定重发。
*     3. 不允许在中断里发 UART：15 字节 @115200 约 1.3 ms，会打乱 USB 轮询。
*
*   v2 新增（2026-11-10，为了"接上手柄却收不到数据"这类现场问题）：
*     a) **不依赖 SearchTypeDevice(HID)**：WCH 库只在"设备类 0 + 接口类 HID +
*        SubClass<=1"时才把中断 IN 端点填进 GpVar[]，Xbox360 接收器常常是
*        vendor-specific(0xFF) 类，这样 SearchTypeDevice(HID) 永远返回 0xFFFF，
*        原来的代码就一个字节都不发（现场看起来像"CH554 死了"）。
*        现在改成：自己扫根端口/外部 HUB 端口，只要枚举成功（DeviceStatus>=3）
*        就尝试取端点，GpVar[0] 为空时用 AnalyzeHidIntEndp() 从配置描述符里
*        重新解析出"中断 + IN"端点。
*     b) **空闲心跳**：还没拿到任何报告时，每 GAMEPAD_IDLE_MS 发 15 个 0x55。
*        STM32 侧会把它当坏帧（s_bad 增加）→ 证明"CH554 活着 + 线通 + 波特率对"，
*        把"没插设备"和"线路/波特率坏了"这两种情况区分开。
*     c) **UART1 调试日志**（P1.7=TXD1=板上 SCK，P1.6=RXD1=板上 MISO，和数据口
*        P3.0/P3.1 分开）：打印连接/枚举结果、VID/PID/DeviceType、端点号、
*        USB 收包数、报告长度、已发帧数。GAMEPAD_DEBUG_UART1=0 可关掉。
*
*   硬件连接（与 STM32 的连线，按实际焊接：P3.0 / P3.1）：
*     CH554G P3.1 (TXD0)  ->  STM32 PB11 (USART3_RX)
*     CH554G P3.0 (RXD0)  <-  STM32 PB10 (USART3_TX)   [本固件不发，只收]
*     CH554G GND          ->  STM32 GND
*     CH554G V33/VCC      ->  3.3V
*   调试/烧录口（同一组脚，互不冲突）：
*     CH554G P1.7 (TXD1/SCK)  -> USB-TTL 的 RXD   （调试输出 + ISP 下载）
*     CH554G P1.6 (RXD1/MISO) <- USB-TTL 的 TXD
*     CH554G GND              -> USB-TTL GND
*
*   编译：Keil C51（uVision4/5），工程文件 CH554Gamepad.uvproj，输出 CH554Gamepad.hex
*   烧录：WCHISPStudio / WCHISPTool，串口下载方式（见 README.md）
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
#define GAMEPAD_SEND_MS          8      /* 收到报告后的转发节奏：8 ms = 125 Hz */
#define GAMEPAD_IDLE_MS        200      /* 还没有报告时的心跳周期（5 Hz，仅用于排查） */
#define GAMEPAD_IDLE_BYTE     0x55      /* 心跳字节：整包都是它，STM32 一定判坏帧 */
#define GAMEPAD_UART0_ALTER      0      /* 0 = TXD0/RXD0 在 P3.1/P3.0；1 = 改到 P1.3/P1.2 */
#define GAMEPAD_DEBUG_UART1      1      /* 1 = 用 UART1(P1.6/P1.7) 打调试信息（量产可关） */
#define GAMEPAD_LOG_PERIOD_MS  500      /* 调试状态行的打印周期 */

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
UINT8   g_last_send_ms;                         /* 上次转发报告的节拍 */
UINT8   g_last_idle_ms;                         /* 上次发心跳的节拍 */
UINT8   g_last_log_ms;                          /* 上次打状态行的节拍 */
UINT16  g_sent_frames;                          /* 已转发帧数（调试用） */
UINT16  g_usb_packets;                          /* USB IN 成功包数（调试用） */
UINT8   g_other_len;                            /* 最近一次非 15 字节包的长度（调试用） */
UINT8X  g_other_head[ 4 ];                      /* 那个包的前 4 字节（调试用） */

/*---------------------------------------------------------------------------
 * 数据口：UART0，115200 8N1，默认引脚 P3.1(TXD0)/P3.0(RXD0)
 *   P3.1 (TXD0) -> STM32 PB11 (USART3_RX)      [本固件只发]
 *   P3.0 (RXD0) <- STM32 PB10 (USART3_TX)      [本固件不收]
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

/* 空闲心跳：一整包都是 0x55，STM32 侧必然判坏帧（只为了证明 CH554 活着） */
static void DataUart_SendIdle( void )
{
    UINT8 i;
    for ( i = 0; i < GAMEPAD_REPORT_SIZE; i++ )
    {
        DataUart_SendByte( GAMEPAD_IDLE_BYTE );
    }
}

#if ( GAMEPAD_DEBUG_UART1 )
/*---------------------------------------------------------------------------
 * 调试口：UART1，P1.7(TXD1/SCK)/P1.6(RXD1/MISO)，和数据口 P3.0/P3.1 分开
 *   和 ISP 下载口是同一组脚：烧录时 USB-TTL 接 P1.6/P1.7/GND，
 *   烧完不要拔，就能直接看这里打出来的日志。
 *-------------------------------------------------------------------------*/
static void Debug_Uart_Init( void )
{
    U1SM0  = 0;                                 /* 8 位数据位 */
    U1SMOD = 1;                                 /* 快速模式：Fsys/16/(256-SBAUD1) */
    U1REN  = 0;                                 /* 只发不收 */
    SBAUD1 = (UINT8)( 0 - ( FREQ_SYS / 16 / UART1_BUAD ) );  /* 24MHz/115200 -> 0xF3 */
    U1TI   = 0;
    PIN_FUNC &= ~bUART1_PIN_X;                  /* 默认 P1.7/P1.6 */
    P1_MOD_OC &= ~0x80;                         /* P1.7 TXD1 推挽输出 */
    P1_DIR_PU |=  0x80;
}

static void Log_PutChar( UINT8 c )
{
    U1TI = 0;
    SBUF1 = c;
    while ( U1TI == 0 );
    U1TI = 0;
}

static void Log_Str( char *s )
{
    while ( *s )
    {
        Log_PutChar( (UINT8)*s );
        s++;
    }
}

static void Log_Hex8( UINT8 v )
{
    UINT8 hi = (UINT8)( v >> 4 );
    UINT8 lo = (UINT8)( v & 0x0F );
    Log_PutChar( hi < 10 ? (UINT8)( '0' + hi ) : (UINT8)( 'A' + hi - 10 ) );
    Log_PutChar( lo < 10 ? (UINT8)( '0' + lo ) : (UINT8)( 'A' + lo - 10 ) );
}

static void Log_Hex16( UINT16 v )
{
    Log_Hex8( (UINT8)( v >> 8 ) );
    Log_Hex8( (UINT8)v );
}

static void Log_Dec16( UINT16 v )
{
    UINT8 buf[ 5 ];
    UINT8 i = 0;
    if ( v == 0 )
    {
        Log_PutChar( '0' );
        return;
    }
    while ( v && i < 5 )
    {
        buf[ i++ ] = (UINT8)( '0' + ( v % 10 ) );
        v = (UINT16)( v / 10 );
    }
    while ( i )
    {
        i--;
        Log_PutChar( buf[ i ] );
    }
}

#define LOG(s)          Log_Str( s )
#else
#define LOG(s)
#define Log_Hex8(x)
#define Log_Hex16(x)
#define Log_Dec16(x)
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
 * 手柄：自己找设备（不依赖 SearchTypeDevice 的类匹配）-> 取中断 IN 端点 -> 读报告
 *-------------------------------------------------------------------------*/
static UINT8 g_pad_loc;                           /* 0 = 根端口；1..HUB_MAX_PORTS = HUB 端口 */
static UINT8 g_pad_endp;                          /* 中断 IN 端点号 + 同步标志位(bit7) */
static UINT8 g_pad_ok;                            /* 1 = 当前有可用设备 */
static UINT8 g_last_status;                       /* 上一次看到的 ThisUsbDev.DeviceStatus */

/* 从端口结构里取中断 IN 端点；GpVar[0] 为空时用 AnalyzeHidIntEndp() 兜底解析 */
static UINT8 Pad_GetEndp( UINT8 loc )
{
    UINT8 endp;

    if ( loc )
    {
        endp = DevOnHubPort[ loc - 1 ].GpVar[ 0 ];
    }
    else
    {
        endp = ThisUsbDev.GpVar[ 0 ];
    }

    if ( ( endp & USB_ENDP_ADDR_MASK ) == 0 )
    {
        /* WCH 库没填（设备类不是标准 HID）：自己从配置描述符里找"中断 + IN"端点 */
        if ( AnalyzeHidIntEndp( Com_Buffer, loc ) )
        {
            if ( loc )
            {
                endp = DevOnHubPort[ loc - 1 ].GpVar[ 0 ];
            }
            else
            {
                endp = ThisUsbDev.GpVar[ 0 ];
            }
        }
    }

    return endp;
}

static UINT8 Pad_StatusOf( UINT8 loc )
{
    return loc ? DevOnHubPort[ loc - 1 ].DeviceStatus : ThisUsbDev.DeviceStatus;
}

/* 找一个"枚举成功且带中断 IN 端点"的设备，返回 1 = 找到 */
static UINT8 Gamepad_Find( void )
{
    UINT8 i, endp;

    /* 1) 直接插在 CH554 USB 口上的设备（最常见） */
    if ( ThisUsbDev.DeviceStatus >= ROOT_DEV_SUCCESS )
    {
        endp = Pad_GetEndp( 0 );
        if ( ( endp & USB_ENDP_ADDR_MASK ) != 0 )
        {
            g_pad_loc  = 0;
            g_pad_endp = endp;
            return 1;
        }
    }

    /* 2) 外接 HUB 下面的设备（备用，我们板子上一般没有） */
    for ( i = 1; i <= HUB_MAX_PORTS; i++ )
    {
        if ( DevOnHubPort[ i - 1 ].DeviceStatus >= ROOT_DEV_SUCCESS )
        {
            endp = Pad_GetEndp( i );
            if ( ( endp & USB_ENDP_ADDR_MASK ) != 0 )
            {
                g_pad_loc  = i;
                g_pad_endp = endp;
                return 1;
            }
        }
    }

    return 0;
}

static void Gamepad_ScanDevice( void )
{
    UINT8 st = ThisUsbDev.DeviceStatus;

    /* 打印设备状态变化（0 无设备 / 1 有但没初始化 / 2 枚举失败 / 3 枚举成功） */
    if ( st != g_last_status )
    {
        g_last_status = st;
        LOG( "\r\n[usb] status=" );
        Log_Dec16( st );
        LOG( " type=" );
        Log_Hex8( ThisUsbDev.DeviceType );
        LOG( " vid=" );
        Log_Hex16( ThisUsbDev.DeviceVID );
        LOG( " pid=" );
        Log_Hex16( ThisUsbDev.DevicePID );
        LOG( " endp0=" );
        Log_Hex8( ThisUsbDev.GpVar[ 0 ] );
        LOG( "\r\n" );
    }

    if ( !g_pad_ok )
    {
        if ( Gamepad_Find() )
        {
            g_pad_ok = 1;
            LOG( "[pad] found, loc=" );
            Log_Dec16( g_pad_loc );
            LOG( " endp=" );
            Log_Hex8( g_pad_endp );
            LOG( "\r\n" );
        }
    }
}

static void Gamepad_Poll( void )
{
    UINT8 s, endp, len, i;

    if ( !g_pad_ok ) return;

    endp = g_pad_endp;
    if ( ( endp & USB_ENDP_ADDR_MASK ) == 0 ) return;

    SelectHubPort( g_pad_loc );                              /* 选中端口、设置速度和地址 */

    s = USBHostTransact( (UINT8)( USB_PID_IN << 4 | ( endp & 0x7F ) ),
                         (UINT8)( ( endp & 0x80 ) ? ( bUH_R_TOG | bUH_T_TOG ) : 0 ),
                         0 );                                /* 0 = NAK 不重试 */
    if ( s == ERR_SUCCESS )
    {
        endp ^= 0x80;                                        /* 翻转同步标志位 */
        if ( g_pad_loc )
        {
            DevOnHubPort[ g_pad_loc - 1 ].GpVar[ 0 ] = endp;
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
            if ( !g_have_report )
            {
                LOG( "[pad] first report\r\n" );
            }
            g_have_report = 1;
        }
        else if ( len )
        {
            g_other_len = len;                               /* 记下来，方便排查 */
            for ( i = 0; i < 4; i++ )
            {
                g_other_head[ i ] = RxBuffer[ i ];
            }
            LOG( "[pad] len=" );
            Log_Dec16( len );
            LOG( " head=" );
            for ( i = 0; i < 4; i++ )
            {
                Log_Hex8( g_other_head[ i ] );
            }
            LOG( "\r\n" );
        }
    }
    else if ( s != ( USB_PID_NAK | ERR_USB_TRANSFER ) )
    {
        /* 真正出错（不是 NAK）：设备可能拔掉了，下一轮重新找 */
        if ( g_pad_loc )
        {
            DevOnHubPort[ g_pad_loc - 1 ].GpVar[ 0 ] &= 0x7F;
        }
        else
        {
            ThisUsbDev.GpVar[ 0 ] &= 0x7F;
        }
        LOG( "[pad] lost, err=" );
        Log_Hex8( s );
        LOG( "\r\n" );
        g_pad_ok      = 0;
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

    LOG( "\r\n[ch554] gamepad host v2 (uart0 p3.1/p3.0 115200)\r\n" );

    InitUSB_Host( );
    FoundNewDev    = 0;
    g_have_report  = 0;
    g_pad_ok       = 0;
    g_last_status  = 0xFF;
    g_last_idle_ms = g_tick_ms;
    g_last_log_ms  = g_tick_ms;

    while ( 1 )
    {
        /* 1. 插入/拔出检测 */
        if ( UIF_DETECT )
        {
            UIF_DETECT = 0;
            s = AnalyzeRootHub( );
            if ( s == ERR_USB_CONNECT )
            {
                FoundNewDev = 1;
                LOG( "[usb] connect\r\n" );
            }
            else if ( s == ERR_USB_DISCON )
            {
                LOG( "[usb] disconnect\r\n" );
                g_pad_ok      = 0;
                g_have_report = 0;
            }
        }

        /* 2. 新设备枚举 */
        if ( FoundNewDev )
        {
            FoundNewDev = 0;
            mDelaymS( 20 );                     /* 等设备上电稳定 */
            s = (UINT8)EnumAllRootDevice( );
            LOG( "[usb] enum=" );
            Log_Hex8( s );
            LOG( "\r\n" );
        }
        (void)EnumAllHubPort( );                /* 外接 HUB 下的设备（备用） */

        /* 3. 定位手柄并读一包 */
        Gamepad_ScanDevice( );
        Gamepad_Poll( );

        /* 4. 固定节奏把最近一包原样转发给 STM32
              没有报告时发心跳（15 个 0x55），好让现场能区分"没插设备"和"线路坏" */
        if ( g_have_report )
        {
            if ( (UINT8)( g_tick_ms - g_last_send_ms ) >= GAMEPAD_SEND_MS )
            {
                g_last_send_ms = g_tick_ms;
                DataUart_SendBytes( g_last_report, GAMEPAD_REPORT_SIZE );
                g_sent_frames++;
            }
        }
#if ( GAMEPAD_DEBUG_UART1 )
        else if ( (UINT8)( g_tick_ms - g_last_idle_ms ) >= GAMEPAD_IDLE_MS )
        {
            g_last_idle_ms = g_tick_ms;
            DataUart_SendIdle( );
        }

        /* 5. 周期性状态行 */
        if ( (UINT8)( g_tick_ms - g_last_log_ms ) >= GAMEPAD_LOG_PERIOD_MS )
        {
            g_last_log_ms = g_tick_ms;
            LOG( "[stat] pad=" );
            Log_Dec16( g_pad_ok );
            LOG( " pkt=" );
            Log_Dec16( g_usb_packets );
            LOG( " sent=" );
            Log_Dec16( g_sent_frames );
            LOG( " len=" );
            Log_Dec16( g_other_len );
            LOG( "\r\n" );
            g_usb_packets = 0;
        }
#endif
    }
}
