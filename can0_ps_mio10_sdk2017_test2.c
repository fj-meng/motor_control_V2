/*
 * Zynq-7000 PS CAN0 裸机轮询接收测试程序
 * 适用工具版本：Vivado 2017.4 + SDK 2017.4
 *
 * 一、当前测试所依据的硬件连接
 *
 * 1. 板载 CAN 收发器的逻辑侧已经连接到 Zynq PS 的 MIO：
 *      CAN0_RX = PS_MIO10
 *      CAN0_TX = PS_MIO11
 *    因此本程序使用 Zynq PS 自带的 CAN0 外设，不使用 PL CAN IP，
 *    也不需要在 Block Design 中添加 AXI CAN、AXI FIFO 等 IP。
 *
 * 2. 调试串口使用 UART1：
 *      UART1_TX = PS_MIO48
 *      UART1_RX = PS_MIO49
 *    SDK 工程的标准输出应选择 ps7_uart_1，波特率为 115200。
 *
 * 3. CAN 总线参数（需要与电机的实际配置完全一致）：
 *      CAN 协议       = 经典 CAN 2.0
 *      CAN 波特率     = 1 Mbit/s
 *      电机 CAN ID    = 1
 *      主机 CAN ID    = 100
 *
 * 二、程序功能
 *
 * 1. 初始化 PS CAN0，并设置 CAN 位时序；
 * 2. 清零 PS CAN 控制器内部接收时间戳计数器；
 * 3. 进入正常工作模式；
 * 4. 使用轮询方式检查 CAN 接收 FIFO；
 * 5. 收到数据后打印 CAN ID、DLC、硬件接收时间戳和数据内容。
 *
 * 三、安全策略
 *
 * 本程序默认 SEND_SAFE_PROBE = 0，即只接收、不发送。这样可以先验证
 * CAN 接线、波特率和接收链路，不会因为程序启动而主动改变电机运行状态。
 *
 * 只有在转子已经固定、机械负载安全、电机当前模式已确认之后，才考虑将
 * SEND_SAFE_PROBE 改为 1。发送帧的解释与电机当前运行模式有关，不能把
 * 测试帧简单理解成所有模式下都绝对不会运动。
 */

#include "xcanps.h"
#include "xcanps_hw.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "xstatus.h"

/* xparameters.h 中由 Vivado 导出的 PS CAN0 设备编号。 */
#define CAN_DEVICE_ID       XPAR_XCANPS_0_DEVICE_ID

/*
 * SDK 2017.4 版本的 XCanPs 驱动使用 4 个 32 位字保存一帧：
 *
 *   Frame[0]：CAN ID 寄存器编码值
 *   Frame[1]：DLC 字段和接收时间戳
 *   Frame[2]：数据字节 0~3
 *   Frame[3]：数据字节 4~7
 *
 * 不要使用新版本驱动中可能出现的 XCANPS_MAX_FRAME_SIZE_IN_WORDS；
 * SDK 2017.4 中通常没有这个宏。
 */
#define CAN_FRAME_WORDS     4U

/*
 * 以下位时序数值假定 Zynq PS CAN 外设时钟为 24 MHz。
 *
 * 这是本程序最需要现场确认的参数。请在 Vivado 的 Zynq PS 配置中查看
 * CAN 外设时钟；如果实际时钟不是 24 MHz，必须重新计算 BRPR、TSEG1、
 * TSEG2 和 SJW，否则 CAN 可能完全无法通信，或者出现大量错误帧。
 *
 * XCanPs_SetBitTiming() 接收的是控制器编码值，实际时间段通常比传入值
 * 多 1 个计数，因此不要直接把这些宏当作实际段数使用。
 */
#define CAN_BRPR_1MBPS      1U
#define CAN_BTR_SJW         2U
#define CAN_BTR_TSEG2       2U
#define CAN_BTR_TSEG1       7U

/* 电机资料中的默认示例值；若通过串口重新配置过，必须同步修改。 */
#define MOTOR_CAN_ID        1U
#define HOST_CAN_ID         100U

/*
 * 首次上电测试必须保持为 0：
 *   0 = 只初始化并接收，不主动向电机发送控制帧；
 *   1 = 初始化完成后发送 Can0_SendSafeProbe() 中的测试帧。
 *
 * 发送功能使用条件编译包围，保持为 0 时不会生成无用的发送缓冲区和
 * 发送函数，也不会出现 -Wunused-function / -Wunused-variable 警告。
 */
#define SEND_SAFE_PROBE     0

/* PS CAN0 驱动实例和接收帧缓冲区。 */
static XCanPs Can0;
static u32 RxFrame[CAN_FRAME_WORDS];

#if SEND_SAFE_PROBE
/* 只有开启发送测试时才分配发送缓冲区。 */
static u32 TxFrame[CAN_FRAME_WORDS];
#endif

/*
 * 初始化 PS CAN0。
 *
 * 该函数只负责控制器初始化、位时序配置和进入正常模式，不负责电机
 * 协议初始化。电机的模式、速度、电流和使能命令应在确认基础通信成功后
 * 单独增加，避免把通信问题和电机运动问题混在一起调试。
 */
static int Can0_Init(void)
{
	XCanPs_Config *config;
	int status;

	/*
	 * 根据 xparameters.h 中的设备 ID 查找 CAN0 配置表。
	 * 如果返回 NULL，通常说明 Vivado 没有启用 CAN0，或者导出硬件后
	 * SDK 平台没有更新。
	 */
	config = XCanPs_LookupConfig(CAN_DEVICE_ID);
	if (config == NULL) {
		xil_printf("CAN: lookup config failed\r\n");
		return XST_FAILURE;
	}

	/* 将驱动实例与 PS CAN0 的寄存器基地址绑定。 */
	status = XCanPs_CfgInitialize(&Can0, config, config->BaseAddr);
	if (status != XST_SUCCESS) {
		xil_printf("CAN: CfgInitialize failed: %d\r\n", status);
		return status;
	}

	/*
	 * 复位 CAN 控制器，并进入配置模式。CAN 位时序寄存器只能在配置模式
	 * 下修改，不能在正常通信过程中直接修改。
	 */
	XCanPs_Reset(&Can0);
	XCanPs_EnterMode(&Can0, XCANPS_MODE_CONFIG);

	/* 设置 CAN 波特率预分频器；该值必须和 CAN 时钟一起计算。 */
	status = XCanPs_SetBaudRatePrescaler(&Can0, CAN_BRPR_1MBPS);
	if (status != XST_SUCCESS) {
		xil_printf("CAN: baud prescaler failed: %d\r\n", status);
		return status;
	}

	/* 设置同步跳转宽度、时间段 2 和时间段 1。 */
	status = XCanPs_SetBitTiming(&Can0, CAN_BTR_SJW,
			CAN_BTR_TSEG2, CAN_BTR_TSEG1);
	if (status != XST_SUCCESS) {
		xil_printf("CAN: bit timing failed: %d\r\n", status);
		return status;
	}

	/*
	 * 清零 PS CAN 内部接收时间戳计数器。
	 *
	 * 这是 CAN 控制器自身的硬件计数器，不是 ARM 软件中断时间。接收一帧
	 * 后，XCanPs_Recv() 会把该帧的接收时间戳放在 Frame[1] 的低 16 位，
	 * 而 DLC 位于 Frame[1] 的高 4 位。时间戳只有 16 位，会循环回零；
	 * 后续如果需要长时间运行，应在软件中扩展高位或结合 PPS 进行换算。
	 */
	XCanPs_ClearTimestamp(&Can0);
	XCanPs_EnterMode(&Can0, XCANPS_MODE_NORMAL);

	/* 等待硬件确认已经进入正常模式。 */
	while (XCanPs_GetMode(&Can0) != XCANPS_MODE_NORMAL) {
		/* 如果长期卡在这里，应检查 CAN 复位状态、PS 时钟和硬件配置。 */
	}

	xil_printf("CAN0 ready: 1 Mbps, motor ID=%d, host ID=%d\r\n",
			MOTOR_CAN_ID, HOST_CAN_ID);
	return XST_SUCCESS;
}

#if SEND_SAFE_PROBE
/*
 * 发送一帧受控的测试报文。
 *
 * 这段代码默认不参与编译。只有把 SEND_SAFE_PROBE 改为 1 后，才会执行。
 * 执行前必须固定转子，并确认电机不会因当前运行模式改变而产生危险运动。
 */
static int Can0_SendSafeProbe(void)
{
	u8 *data;
	int status;

	/*
	 * 组装一帧经典 CAN 标准帧：
	 *
	 *   CAN ID：MOTOR_CAN_ID，即 1
	 *   DLC：8 字节
	 *   数据：00 00 80 00 00 00 00 00
	 *
	 * XCanPs_CreateIdValue() 会把普通 CAN ID 转换成 PS CAN ID 寄存器所需
	 * 的编码格式，不能直接把 1 写到 TxFrame[0]。
	 * XCanPs_CreateDlcValue() 会把数据长度 8 放到 DLC 寄存器的正确位域。
	 */
	TxFrame[0] = XCanPs_CreateIdValue(MOTOR_CAN_ID, 0U, 0U, 0U, 0U);
	TxFrame[1] = XCanPs_CreateDlcValue(8U);
	data = (u8 *)&TxFrame[2];
	data[0] = 0x00U;
	data[1] = 0x00U;
	data[2] = 0x80U;
	data[3] = 0x00U;
	data[4] = 0x00U;
	data[5] = 0x00U;
	data[6] = 0x00U;
	data[7] = 0x00U;

	status = XCanPs_Send(&Can0, TxFrame);
	if (status != XST_SUCCESS) {
		xil_printf("CAN: probe send failed: %d\r\n", status);
		return status;
	}

	xil_printf("CAN: probe sent, ID=0x%03x, data=00 00 80 00 00 00 00 00\r\n",
			MOTOR_CAN_ID);
	return XST_SUCCESS;
}
#endif

/*
 * 打印一帧 PS CAN 接收数据。
 *
 * 注意：这里先打印原始 ID 寄存器值，raw_id 不一定就是人眼看到的十进制
 * CAN ID。确认通信正常后，再根据标准帧/扩展帧格式对 ID 做正式解码。
 */
static void PrintReceivedFrame(const u32 *frame)
{
	/* Frame[2] 和 Frame[3] 共保存 8 个 CAN 数据字节。 */
	u8 *data = (u8 *)&frame[2];

	/* DLC 在高 4 位，低 16 位是 PS CAN 硬件接收时间戳。 */
	u32 dlc = (frame[1] & XCANPS_DLCR_DLC_MASK) >> XCANPS_DLCR_DLC_SHIFT;
	u32 timestamp = frame[1] & XCANPS_DLCR_TIMESTAMP_MASK;
	u32 i;

	xil_printf("RX raw_id=0x%08x dlc=%d hw_ts=0x%04x data=",
			frame[0], dlc, timestamp);

	/* 经典 CAN 最大数据长度为 8，防止异常 DLC 导致越界读取。 */
	for (i = 0U; i < dlc && i < 8U; ++i) {
		xil_printf("%02x", data[i]);
		if (i + 1U < dlc && i + 1U < 8U) {
			xil_printf(" ");
		}
	}
	xil_printf("\r\n");
}

int main(void)
{
	int status;

	/*
	 * xil_printf() 输出到 SDK 工程配置的 stdout。请确认 BSP 的 stdout
	 * 选择为 ps7_uart_1，并且串口工具设置为 115200、8 数据位、无校验、
	 * 1 个停止位（115200 8N1）。
	 */
	xil_printf("\r\nH10010 PS CAN0 receive test\r\n");
	xil_printf("UART1 stdout, CAN0 MIO10/MIO11, SDK 2017.4\r\n");

	/* 第一步：初始化 CAN0，失败则不再继续运行。 */
	status = Can0_Init();
	if (status != XST_SUCCESS) {
		xil_printf("CAN initialization failed\r\n");
		return XST_FAILURE;
	}

#if SEND_SAFE_PROBE
	/* 只有显式打开宏时才发送测试帧。 */
	status = Can0_SendSafeProbe();
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
#else
	/* 默认安全模式：等待外部节点或电机主动上传数据。 */
	xil_printf("Receive-only mode: no motor command is sent\r\n");
#endif

	/*
	 * 主循环采用轮询方式，适合第一阶段验证：
	 *
	 *   1. 查询接收 FIFO 是否非空；
	 *   2. 读取一帧；
	 *   3. 打印原始内容和硬件时间戳。
	 *
	 * 当前版本没有加入延时，是为了尽快读取 FIFO。后续正式程序应考虑：
	 *   - 改为 CAN 接收中断；
	 *   - 中断中只搬运原始帧，避免打印阻塞；
	 *   - 使用环形缓冲区保存数据；
	 *   - 结合 PPS 把 16 位 CAN 时间戳扩展成系统绝对时间；
	 *   - 增加接收溢出、总线错误和 bus-off 状态处理。
	 */
	while (1) {
		if (!XCanPs_IsRxEmpty(&Can0)) {
			status = XCanPs_Recv(&Can0, RxFrame);
			if (status == XST_SUCCESS) {
				PrintReceivedFrame(RxFrame);
			} else {
				xil_printf("CAN: receive failed: %d\r\n", status);
			}
		}
	}
}
