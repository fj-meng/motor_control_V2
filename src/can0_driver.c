/*
 * PS CAN0 驱动函数实现
 *
 * 适用工具版本：Vivado 2017.4 + SDK 2017.4
 * 硬件连接：CAN0_RX=PS_MIO10，CAN0_TX=PS_MIO11。
 *
 * 接收走中断。中断服务程序只做三件事：从 RX FIFO 取帧、把 IDR/DLCR
 * 解码成线性 ID 和时间戳、推给 logger 的环形缓冲。不打印、不写卡。
 * 落盘由主循环调用 Logger_Process() 完成。
 */

#include "can0_config.h"
#include "can0_driver.h"
#include "logger.h"
#include "xcanps.h"
#include "xcanps_hw.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "sleep.h"

/* 要打开的 CAN 中断：收帧、FIFO 水位、FIFO 溢出、总线错误、总线关闭。 */
#define CAN_RX_INTR_MASK \
	((u32)XCANPS_IXR_RXNEMP_MASK | (u32)XCANPS_IXR_RXFWMFLL_MASK | \
	 (u32)XCANPS_IXR_RXOFLW_MASK | (u32)XCANPS_IXR_ERROR_MASK | \
	 (u32)XCANPS_IXR_BSOFF_MASK)

/* PS CAN0 驱动实例、中断控制器实例和发送缓冲只在本文件内部使用。 */
static XCanPs Can0;
static XScuGic Intc;

/*
 * 两套发送缓冲：
 *   TxFrame     主循环用（命令帧，带重试）
 *   TxFrameIrq  定时器中断用（周期速度帧，不重试）
 * 分开是为了避免主循环正在组帧时被定时器中断打断、把缓冲写乱。
 * 虽然调用方会先停轮询，但多一块 16 字节的缓冲换一个绝对安全，值得。
 */
static u32 TxFrame[CAN_FRAME_WORDS];
static u32 TxFrameIrq[CAN_FRAME_WORDS];

/* 中断里累加的统计量，主循环只读。 */
static volatile Can0RxStats RxStats;

/* 总线关闭标志，由中断置位、主循环清除并恢复。 */
static volatile u32 BusOffFlag;

/* 16 位硬件时间戳扩展成 32 位用的状态。 */
static u64 TsHigh;
static u16 TsLast16;
static u32 TsLastExt;
static u32 TsSeen;

/*
 * 把 16 位硬件时间戳扩展成 32 位，顺带做丢帧自检。
 *
 * 硬件时间戳是 1us 计数、16 位，65.5ms 回绕一次。回绕时补一个 0x10000。
 * 相邻两帧的差值落在 [CAN_RX_GAP_MIN_US, CAN_RX_GAP_MAX_US] 之间，
 * 说明中间至少少了一帧，计入 Gaps。
 *
 * 局限：总线静默超过 65.5ms 时无法判断回绕了几次，会低估一次；
 * 对 1ms 连续上传的场景没有影响。
 */
static u32 Can0_TrackTimestamp(u32 Ts16)
{
	u16 T = (u16)Ts16;
	u64 Ext;
	u32 Us;
	u32 Delta;

	if (T < TsLast16) {
		TsHigh += 0x10000ULL;
	}
	TsLast16 = T;
	Ext = TsHigh | (u64)T;

	/*
	 * 把计数换算成真正的微秒。
	 *
	 * 计数器时钟是 PS_CLK 33.333MHz，不是手册说的「位时间」（1Mbps 下
	 * 1 计数 = 1us）。不做这一步的话，文件里的时间戳会被放大约 33 倍，
	 * 1ms 的帧间隔显示成 33ms，丢帧判断全部失效。
	 * 推导过程见 can0_config.h 的 CAN_TS_CLK_HZ。
	 *
	 * Ext 是 64 位，×1000000 后会到 1e15 量级，必须用 u64 中间量。
	 */
	Us = (u32)((Ext * 1000000ULL) / (u64)CAN_TS_CLK_HZ);

	if (TsSeen != 0U) {
		/*
		 * Us 是 32 位微秒，71.6 分钟回绕一次。回绕那一帧不做判断，
		 * 否则 Us - TsLastExt 会下溢成一个巨大的数。
		 */
		if (Us >= TsLastExt) {
			Delta = Us - TsLastExt;
			if ((Delta > CAN_RX_GAP_MIN_US)
					&& (Delta < CAN_RX_GAP_MAX_US)) {
				++RxStats.Gaps;
			}
		}
	}

	TsSeen = 1U;
	TsLastExt = Us;

	return Us;
}

/*
 * 初始化 PS CAN0。
 *
 * 该函数负责查找配置、绑定寄存器、设置位时序、清零硬件时间戳并进入
 * 正常模式；不负责中断，也不负责电机协议初始化。
 */
int Can0_Init(void)
{
	XCanPs_Config *Config;
	int Status;

	Config = XCanPs_LookupConfig(CAN_DEVICE_ID);
	if (Config == NULL) {
		xil_printf("CAN: lookup config failed\r\n");
		return XST_FAILURE;
	}

	Status = XCanPs_CfgInitialize(&Can0, Config, Config->BaseAddr);
	if (Status != XST_SUCCESS) {
		xil_printf("CAN: CfgInitialize failed: %d\r\n", Status);
		return Status;
	}

	/* 位时序寄存器只能在配置模式下修改。 */
	XCanPs_Reset(&Can0);
	XCanPs_EnterMode(&Can0, XCANPS_MODE_CONFIG);

	Status = XCanPs_SetBaudRatePrescaler(&Can0, CAN_BRPR_1MBPS);
	if (Status != XST_SUCCESS) {
		xil_printf("CAN: baud prescaler failed: %d\r\n", Status);
		return Status;
	}

	Status = XCanPs_SetBitTiming(&Can0, CAN_BTR_SJW,
			CAN_BTR_TSEG2, CAN_BTR_TSEG1);
	if (Status != XST_SUCCESS) {
		xil_printf("CAN: bit timing failed: %d\r\n", Status);
		return Status;
	}

	/* 清零 PS CAN 内部 16 位接收时间戳计数器。 */
	XCanPs_ClearTimestamp(&Can0);

#if USE_RX_FILTER
	/*
	 * 验收滤波器必须在使能前写入，且只能在滤波器空闲时写。
	 * 复位后四个滤波器都是关闭状态，此时写入一定成功。
	 * 不配置的话硬件全收，总线上任何节点的帧都会进 RX FIFO。
	 */
	Status = XCanPs_AcceptFilterSet(&Can0, XCANPS_AFR_UAF1_MASK,
			RX_FILTER_MASK, RX_FILTER_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("CAN: accept filter set failed: %d\r\n", Status);
		return Status;
	}
	XCanPs_AcceptFilterEnable(&Can0, XCANPS_AFR_UAF1_MASK);
	xil_printf("CAN: RX filter on, id=0x%08x mask=0x%08x\r\n",
			RX_FILTER_ID, RX_FILTER_MASK);
#endif

	XCanPs_EnterMode(&Can0, XCANPS_MODE_NORMAL);

	while (XCanPs_GetMode(&Can0) != XCANPS_MODE_NORMAL) {
		/* 等待硬件确认进入正常模式。 */
	}

	xil_printf("CAN0 ready: 1 Mbps, motor ID=%d, host ID=%d\r\n",
			MOTOR_CAN_ID, HOST_CAN_ID);
	return XST_SUCCESS;
}

/*
 * 发送一帧 CAN 报文。
 *
 * ID 走 Can0_MakeExtIdValue / Can0_MakeStdIdValue 转成 IDR 分片格式，
 * 数据按字节顺序写入 DW1R / DW2R（与 XCanPs_Recv 的字节序处理对称）。
 */
int Can0_SendFrame(u32 id, u8 ide, u8 rtr, const u8 *data, u8 dlc)
{
	u8 *Dst;
	u8 i;
	u32 Retry;

	if ((data == NULL) || (dlc > 8U)) {
		return XST_INVALID_PARAM;
	}

	TxFrame[0] = (ide != 0U) ? Can0_MakeExtIdValue(id, (u32)rtr)
			: Can0_MakeStdIdValue(id, (u32)rtr);
	TxFrame[1] = XCanPs_CreateDlcValue((u32)dlc);

	Dst = (u8 *)&TxFrame[2];
	for (i = 0U; i < dlc; ++i) {
		Dst[i] = data[i];
	}

	/* TX FIFO 满时 XCanPs_Send 返回 XST_FIFO_NO_ROOM，短暂退避后重试。 */
	for (Retry = CAN_TX_RETRY_LIMIT; Retry > 0U; --Retry) {
		if (XCanPs_Send(&Can0, TxFrame) == XST_SUCCESS) {
			return XST_SUCCESS;
		}
		usleep(CAN_TX_RETRY_DELAY_US);
	}

	return XST_FAILURE;
}

/*
 * 接收中断回调。
 *
 * XCanPs_IntrHandler 检测到 RXNEMP/RXFWMFLL 后调用本函数。一次中断
 * 可能对应 FIFO 里好几帧（尤其被更高优先级打断之后），所以必须循环
 * 排空，不能只取一帧——只取一帧是这类驱动最常见的丢帧原因。
 *
 * 本函数只做「取帧 + 解码 + 入队」，任何耗时操作都不能放进来。
 */
static void Can0_RecvHandler(void *CallBackRef)
{
	u32 Raw[CAN_FRAME_WORDS];
	u32 Idr;
	u32 Dlcr;
	u32 Id1;
	u32 Id2;
	u32 Id;
	u32 Ts;
	u8 Ide;
	u8 Rtr;
	u8 Dlc;

	(void)CallBackRef;

	while (XCanPs_IsRxEmpty(&Can0) == FALSE) {
		if (XCanPs_Recv(&Can0, Raw) != XST_SUCCESS) {
			break;
		}

		Idr = Raw[0];
		Dlcr = Raw[1];

		Id1 = (Idr & XCANPS_IDR_ID1_MASK) >> XCANPS_IDR_ID1_SHIFT;
		Id2 = (Idr & XCANPS_IDR_ID2_MASK) >> XCANPS_IDR_ID2_SHIFT;
		Ide = ((Idr & XCANPS_IDR_IDE_MASK) != 0U) ? 1U : 0U;
		Rtr = ((Idr & XCANPS_IDR_RTR_MASK) != 0U) ? 1U : 0U;
		Id = (Ide != 0U) ? ((Id1 << 18) | Id2) : Id1;

		Dlc = (u8)((Dlcr & XCANPS_DLCR_DLC_MASK) >>
				XCANPS_DLCR_DLC_SHIFT);
		if (Dlc > 8U) {
			Dlc = 8U;
		}

		Ts = Can0_TrackTimestamp(Dlcr & XCANPS_DLCR_TIMESTAMP_MASK);
		++RxStats.Frames;

		/* XCanPs_Recv 已对 DW1R/DW2R 做过字节序转换，
		 * 小端 ARM 上 &Raw[2] 的内存首字节就是数据字节 0。 */
		Logger_Push(Id, Ide, Rtr, Dlc, (const u8 *)&Raw[2], Ts);
	}
}

/* 错误中断里记录下来的错误类型，主循环读走即清零。 */
static volatile u32 LastErrorMask;

/* 总线错误回调：只计数，不在这里做任何恢复动作。 */
static void Can0_ErrorHandler(void *CallBackRef, u32 ErrorMask)
{
	(void)CallBackRef;

	++RxStats.Errors;

	/*
	 * ErrorMask 和 ESR 的位定义一致：
	 *   bit0 CRC 错  bit1 格式错  bit2 填充错  bit3 位错  bit4 ACK 错
	 * 必须在这里抓，因为驱动随后会清 ESR，主循环再去读就是 0 了。
	 */
	LastErrorMask |= ErrorMask;
}

/* 事件回调：FIFO 溢出、总线关闭等。 */
static void Can0_EventHandler(void *CallBackRef, u32 Mask)
{
	(void)CallBackRef;

	if ((Mask & XCANPS_IXR_RXOFLW_MASK) != 0U) {
		++RxStats.FifoOverflow;
	}

	if ((Mask & XCANPS_IXR_BSOFF_MASK) != 0U) {
		++RxStats.BusOff;
		BusOffFlag = 1U;
	}
}

/*
 * 挂 GIC、注册回调、打开 CAN 接收中断。
 *
 * 顺序不能反：必须先 XScuGic_Connect 再 XScuGic_Enable，否则中断线
 * 打开的那一瞬间没人处理，硬件会一直挂着。
 */
int Can0_SetupInterrupt(void)
{
	XScuGic_Config *IntcConfig;
	int Status;

	IntcConfig = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID);
	if (IntcConfig == NULL) {
		xil_printf("CAN: GIC lookup failed\r\n");
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(&Intc, IntcConfig,
			IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("CAN: GIC init failed: %d\r\n", Status);
		return Status;
	}

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			(Xil_ExceptionHandler)XScuGic_InterruptHandler, &Intc);

	Status = XScuGic_Connect(&Intc, XPAR_XCANPS_0_INTR,
			(Xil_ExceptionHandler)XCanPs_IntrHandler, (void *)&Can0);
	if (Status != XST_SUCCESS) {
		xil_printf("CAN: GIC connect failed: %d\r\n", Status);
		return Status;
	}

	XCanPs_SetHandler(&Can0, XCANPS_HANDLER_RECV,
			(void *)Can0_RecvHandler, (void *)&Can0);
	XCanPs_SetHandler(&Can0, XCANPS_HANDLER_ERROR,
			(void *)Can0_ErrorHandler, (void *)&Can0);
	XCanPs_SetHandler(&Can0, XCANPS_HANDLER_EVENT,
			(void *)Can0_EventHandler, (void *)&Can0);

	/* 先清干净历史状态，再开中断，避免一上来就进一次假的溢出中断。 */
	XCanPs_IntrClear(&Can0, 0xFFFFFFFFU);
	XCanPs_IntrEnable(&Can0, CAN_RX_INTR_MASK);

	XScuGic_Enable(&Intc, XPAR_XCANPS_0_INTR);
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

/*
 * 发送一帧，TX FIFO 满就立刻返回失败，不重试、不等待。
 *
 * 专给中断里用：Can0_SendFrame() 里有 usleep 重试循环，在中断上下文里
 * 会把整个系统卡死。中断里只能用这个版本，发不出去就跳过这一拍。
 */
int Can0_SendFrameNoWait(u32 id, u8 ide, u8 rtr, const u8 *data, u8 dlc)
{
	u8 *Dst;
	u8 i;

	if ((data == NULL) || (dlc > 8U)) {
		return XST_INVALID_PARAM;
	}

	TxFrameIrq[0] = (ide != 0U) ? Can0_MakeExtIdValue(id, (u32)rtr)
			: Can0_MakeStdIdValue(id, (u32)rtr);
	TxFrameIrq[1] = XCanPs_CreateDlcValue((u32)dlc);

	Dst = (u8 *)&TxFrameIrq[2];
	for (i = 0U; i < dlc; ++i) {
		Dst[i] = data[i];
	}

	return XCanPs_Send(&Can0, TxFrameIrq);
}

/*
 * 把另一个中断源挂到同一个 GIC 上。
 *
 * GIC 实例在 Can0_SetupInterrupt() 里初始化，这里复用。不要重新
 * XScuGic_CfgInitialize，那会把已经连好的 CAN 中断配置清掉。
 */
int Can0_ConnectIrq(u32 IrqId, Xil_ExceptionHandler Handler, void *Ref)
{
	int Status;

	Status = XScuGic_Connect(&Intc, IrqId, Handler, Ref);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	XScuGic_Enable(&Intc, IrqId);
	return XST_SUCCESS;
}

void Can0_GetRxStats(Can0RxStats *Stats)
{
	if (Stats == NULL) {
		return;
	}

	/* 逐个字段取，避免读到半更新的结构体。 */
	Stats->Frames = RxStats.Frames;
	Stats->Gaps = RxStats.Gaps;
	Stats->FifoOverflow = RxStats.FifoOverflow;
	Stats->BusOff = RxStats.BusOff;
	Stats->Errors = RxStats.Errors;
}

u32 Can0_GetErrorCounters(void)
{
	return XCanPs_ReadReg(Can0.CanConfig.BaseAddr, XCANPS_ECR_OFFSET);
}

/*
 * 取走中断里记录的错误类型并清零。
 *
 *   bit0 CRC 错   —— 数据被干扰，线长/终端电阻/干扰
 *   bit1 格式错   —— 帧格式不对，多半波特率不匹配
 *   bit2 填充错   —— 位填充违规，同上
 *   bit3 位错     —— 采样位与发出的不一致，总线冲突或物理层问题
 *   bit4 ACK 错   —— 发出去没人应答
 *
 * 返回 0 表示上次读取以来没发生过错误。
 */
u32 Can0_GetLastErrorMask(void)
{
	u32 Mask = LastErrorMask;

	LastErrorMask = 0U;
	return Mask;
}

/*
 * 复位 CAN 控制器并重新配置。
 *
 * 两个用途：
 *   1. bus-off 之后恢复——控制器会自己断开总线，不恢复就永远收不到帧
 *   2. 手动清错误计数——REC/TEC 涨到错误被动（>127）时，复位一次就归零
 *
 * 复位会清掉 IER 和验收滤波器，所以位时序和中断使能都要重新配一遍。
 * 整个过程屏蔽 CAN 中断，避免复位期间 ISR 进来读到半配置的寄存器。
 *
 * 硬件时间戳计数器也被清零了，软件里扩展成 32 位的回绕状态必须一起重置，
 * 否则 ts 会突然跳一大截，丢帧检测（gap）会误报。
 */
void Can0_Restart(void)
{
	u32 Guard;

	XScuGic_Disable(&Intc, XPAR_XCANPS_0_INTR);
	XCanPs_IntrDisable(&Can0, 0xFFFFFFFFU);

	XCanPs_Reset(&Can0);
	XCanPs_EnterMode(&Can0, XCANPS_MODE_CONFIG);
	(void)XCanPs_SetBaudRatePrescaler(&Can0, CAN_BRPR_1MBPS);
	(void)XCanPs_SetBitTiming(&Can0, CAN_BTR_SJW,
			CAN_BTR_TSEG2, CAN_BTR_TSEG1);
	XCanPs_ClearTimestamp(&Can0);
	XCanPs_EnterMode(&Can0, XCANPS_MODE_NORMAL);

	/* 硬件时间戳清零，软件回绕状态跟着重置 */
	TsHigh = 0ULL;
	TsLast16 = 0U;
	TsSeen = 0U;
	TsLastExt = 0U;

	/* 等硬件确认，但不能死等——超时就交给下一次心跳重试。 */
	Guard = 100000U;
	while ((XCanPs_GetMode(&Can0) != XCANPS_MODE_NORMAL) && (Guard > 0U)) {
		--Guard;
	}

	XCanPs_IntrClear(&Can0, 0xFFFFFFFFU);
	XCanPs_IntrEnable(&Can0, CAN_RX_INTR_MASK);
	XScuGic_Enable(&Intc, XPAR_XCANPS_0_INTR);
}

int Can0_RecoverIfNeeded(void)
{
	if (BusOffFlag == 0U) {
		return 0;
	}
	BusOffFlag = 0U;

	Can0_Restart();
	return 1;
}

/*
 * 把 RX FIFO 的 4 个 32 位字解码成可读的帧结构。
 *
 * IDR 用的是 Bosch PeliCAN 的分片格式，不是线性 ID：
 *
 *   bit 31..21  ID1 = ID.28..ID.18   （高 11 位）
 *   bit 20      SRR
 *   bit 19      IDE = 1 表示扩展帧（电机手册里的 FF 位）
 *   bit 18..1   ID2 = ID.17..ID.0    （低 18 位）
 *   bit 0       RTR
 *
 * 所以扩展帧完整 ID = (ID1 << 18) | ID2，共 29 位。
 * 标准帧只占 ID1 这 11 位，ID2 为 0。
 *
 * DLCR 里 DLC 在最高 4 位 [31:28]，低 16 位是接收时间戳，不是 DLC。
 */
void Can0_DecodeFrame(const u32 *raw, Can0Frame *frame)
{
	u32 Idr = raw[0];
	u32 Dlcr = raw[1];
	u32 Id1;
	u32 Id2;
	u8 i;

	Id1 = (Idr & XCANPS_IDR_ID1_MASK) >> XCANPS_IDR_ID1_SHIFT;
	Id2 = (Idr & XCANPS_IDR_ID2_MASK) >> XCANPS_IDR_ID2_SHIFT;

	frame->Ide = ((Idr & XCANPS_IDR_IDE_MASK) != 0U) ? 1U : 0U;
	frame->Rtr = ((Idr & XCANPS_IDR_RTR_MASK) != 0U) ? 1U : 0U;

	if (frame->Ide != 0U) {
		frame->Id = (Id1 << 18) | Id2;
	} else {
		frame->Id = Id1;
	}

	frame->Dlc = (u8)((Dlcr & XCANPS_DLCR_DLC_MASK) >>
			XCANPS_DLCR_DLC_SHIFT);
	frame->Timestamp = (u16)(Dlcr & XCANPS_DLCR_TIMESTAMP_MASK);

	/* XCanPs_Recv() 已经对 DW1R/DW2R 做过字节序转换，
	 * 所以在小端 ARM 上 raw[2] 的内存首字节就是数据字节 0。
	 */
	for (i = 0U; i < 8U; ++i) {
		frame->Data[i] = ((const u8 *)&raw[2])[i];
	}
}

#if SEND_SAFE_PROBE
/*
 * 发送一帧受控测试报文。启用前必须固定转子并确认电机模式。
 * 数据内容沿用原测试程序：00 00 80 00 00 00 00 00。
 */
int Can0_SendSafeProbe(void)
{
	u8 *Data;
	int Status;

	TxFrame[0] = Can0_MakeStdIdValue(MOTOR_CAN_ID, 0U);
	TxFrame[1] = XCanPs_CreateDlcValue(8U);
	Data = (u8 *)&TxFrame[2];
	Data[0] = 0x00U;
	Data[1] = 0x00U;
	Data[2] = 0x80U;
	Data[3] = 0x00U;
	Data[4] = 0x00U;
	Data[5] = 0x00U;
	Data[6] = 0x00U;
	Data[7] = 0x00U;

	Status = XCanPs_Send(&Can0, TxFrame);
	if (Status != XST_SUCCESS) {
		xil_printf("CAN: probe send failed: %d\r\n", Status);
		return Status;
	}

	xil_printf("CAN: probe sent, ID=0x%03x, data=00 00 80 00 00 00 00 00\r\n",
			MOTOR_CAN_ID);
	return XST_SUCCESS;
}
#endif
