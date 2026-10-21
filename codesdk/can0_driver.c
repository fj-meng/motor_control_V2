/*
 * PS CAN0 驱动函数实现
 *
 * 适用工具版本：Vivado 2017.4 + SDK 2017.4
 * 硬件连接：CAN0_RX=PS_MIO10，CAN0_TX=PS_MIO11。
 */

#include "can0_config.h"
#include "can0_driver.h"
#include "xcanps.h"
#include "xcanps_hw.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "sleep.h"

/* PS CAN0 驱动实例和接收帧缓冲区只在本文件内部使用。 */
static XCanPs Can0;
static u32 RxFrame[CAN_FRAME_WORDS];
static u32 TxFrame[CAN_FRAME_WORDS];

/*
 * 初始化 PS CAN0。
 *
 * 该函数负责查找配置、绑定寄存器、设置位时序、清零硬件时间戳并进入
 * 正常模式；不负责电机协议初始化和运动控制。
 */
int Can0_Init(void)
{
	XCanPs_Config *config;
	int status;

	config = XCanPs_LookupConfig(CAN_DEVICE_ID);
	if (config == NULL) {
		xil_printf("CAN: lookup config failed\r\n");
		return XST_FAILURE;
	}

	status = XCanPs_CfgInitialize(&Can0, config, config->BaseAddr);
	if (status != XST_SUCCESS) {
		xil_printf("CAN: CfgInitialize failed: %d\r\n", status);
		return status;
	}

	/* 位时序寄存器只能在配置模式下修改。 */
	XCanPs_Reset(&Can0);
	XCanPs_EnterMode(&Can0, XCANPS_MODE_CONFIG);

	status = XCanPs_SetBaudRatePrescaler(&Can0, CAN_BRPR_1MBPS);
	if (status != XST_SUCCESS) {
		xil_printf("CAN: baud prescaler failed: %d\r\n", status);
		return status;
	}

	status = XCanPs_SetBitTiming(&Can0, CAN_BTR_SJW,
			CAN_BTR_TSEG2, CAN_BTR_TSEG1);
	if (status != XST_SUCCESS) {
		xil_printf("CAN: bit timing failed: %d\r\n", status);
		return status;
	}

	/* 清零 PS CAN 内部 16 位接收时间戳计数器。 */
	XCanPs_ClearTimestamp(&Can0);

#if USE_RX_FILTER
	/*
	 * 验收滤波器必须在使能前写入，且只能在滤波器空闲时写。
	 * 复位后四个滤波器都是关闭状态，此时写入一定成功。
	 * 不配置的话硬件全收，总线上任何节点的帧都会进 RX FIFO。
	 */
	status = XCanPs_AcceptFilterSet(&Can0, XCANPS_AFR_UAF1_MASK,
			RX_FILTER_MASK, RX_FILTER_ID);
	if (status != XST_SUCCESS) {
		xil_printf("CAN: accept filter set failed: %d\r\n", status);
		return status;
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
	u8 *dst;
	u8 i;
	u32 retry;

	if ((data == NULL) || (dlc > 8U)) {
		return XST_INVALID_PARAM;
	}

	TxFrame[0] = (ide != 0U) ? Can0_MakeExtIdValue(id, (u32)rtr)
			: Can0_MakeStdIdValue(id, (u32)rtr);
	TxFrame[1] = XCanPs_CreateDlcValue((u32)dlc);

	dst = (u8 *)&TxFrame[2];
	for (i = 0U; i < dlc; ++i) {
		dst[i] = data[i];
	}

	/* TX FIFO 满时 XCanPs_Send 返回 XST_FIFO_NO_ROOM，短暂退避后重试。 */
	for (retry = CAN_TX_RETRY_LIMIT; retry > 0U; --retry) {
		if (XCanPs_Send(&Can0, TxFrame) == XST_SUCCESS) {
			return XST_SUCCESS;
		}
		usleep(CAN_TX_RETRY_DELAY_US);
	}

	return XST_FAILURE;
}

#if SEND_SAFE_PROBE
/*
 * 发送一帧受控测试报文。启用前必须固定转子并确认电机模式。
 * 数据内容沿用原测试程序：00 00 80 00 00 00 00 00。
 */
int Can0_SendSafeProbe(void)
{
	u8 *data;
	int status;

	TxFrame[0] = Can0_MakeStdIdValue(MOTOR_CAN_ID, 0U);
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
	u32 idr = raw[0];
	u32 dlcr = raw[1];
	u32 id1;
	u32 id2;
	u8 i;

	id1 = (idr & XCANPS_IDR_ID1_MASK) >> XCANPS_IDR_ID1_SHIFT;
	id2 = (idr & XCANPS_IDR_ID2_MASK) >> XCANPS_IDR_ID2_SHIFT;

	frame->Ide = ((idr & XCANPS_IDR_IDE_MASK) != 0U) ? 1U : 0U;
	frame->Rtr = ((idr & XCANPS_IDR_RTR_MASK) != 0U) ? 1U : 0U;

	if (frame->Ide != 0U) {
		frame->Id = (id1 << 18) | id2;
	} else {
		frame->Id = id1;
	}

	frame->Dlc = (u8)((dlcr & XCANPS_DLCR_DLC_MASK) >>
			XCANPS_DLCR_DLC_SHIFT);
	frame->Timestamp = (u16)(dlcr & XCANPS_DLCR_TIMESTAMP_MASK);

	/* XCanPs_Recv() 已经对 DW1R/DW2R 做过字节序转换，
	 * 所以在小端 ARM 上 raw[2] 的内存首字节就是数据字节 0。
	 */
	for (i = 0U; i < 8U; ++i) {
		frame->Data[i] = ((const u8 *)&raw[2])[i];
	}
}

/*
 * 判断这一帧在协议里的角色。
 *
 * 按「ID = 目的地址」约定：
 *   ID=HOST_CAN_ID  → 目的地是主机，即电机发来的反馈帧
 *   ID=MOTOR_CAN_ID → 目的地是电机，即别的主机发出的命令帧
 * 两者都不匹配说明总线上有第三方报文。
 */
static const char *FrameRole(const Can0Frame *frame)
{
	if (frame->Ide != 0U) {
		return "EXT";
	}
	if (frame->Id == HOST_CAN_ID) {
		return "FB ";
	}
	if (frame->Id == MOTOR_CAN_ID) {
		return "CMD";
	}
	return "UNK";
}

/* 打印一帧解码后的 CAN 报文。 */
static void PrintReceivedFrame(const u32 *raw)
{
	Can0Frame frame;
	u32 i;

	Can0_DecodeFrame(raw, &frame);

	/* ff 对应电机手册里的帧格式位，rtr 对应帧类型位。 */
	xil_printf("RX [%s] id=0x%08x ff=%d rtr=%d dlc=%d ts=%u data=",
			FrameRole(&frame), frame.Id, frame.Ide, frame.Rtr,
			frame.Dlc, frame.Timestamp);

	/* 经典 CAN 最多 8 字节，防止异常 DLC 导致越界读取。 */
	for (i = 0U; i < frame.Dlc && i < 8U; ++i) {
		xil_printf("%02x", frame.Data[i]);
		if (i + 1U < frame.Dlc && i + 1U < 8U) {
			xil_printf(" ");
		}
	}
	xil_printf(" | raw_idr=0x%08x raw_dlcr=0x%08x\r\n", raw[0], raw[1]);

#if PRINT_MANUAL_FORMAT
	{
		u8 manual[CAN_MANUAL_FRAME_BYTES];

		Can0_FrameToManual(&frame, manual);
		xil_printf("   manual:");
		for (i = 0U; i < CAN_MANUAL_FRAME_BYTES; ++i) {
			xil_printf(" %02x", manual[i]);
		}
		xil_printf("\r\n");
	}
#endif
}

/*
 * 轮询接收 FIFO。按 CAN_RX_PRINT_EVERY_N 限流打印解码结果。
 * 后续正式版本可在这里替换为中断+环形缓冲区实现。
 */
void Can0_RunReceiveLoop(void)
{
#if (CAN_RX_PRINT_EVERY_N > 0U)
	static u32 frame_count;
#endif
	int status;

	while (1) {
		if (!XCanPs_IsRxEmpty(&Can0)) {
			status = XCanPs_Recv(&Can0, RxFrame);
			if (status == XST_SUCCESS) {
#if (CAN_RX_PRINT_EVERY_N > 0U)
				if ((frame_count % CAN_RX_PRINT_EVERY_N) == 0U) {
					PrintReceivedFrame(RxFrame);
				}
				++frame_count;
#endif
			} else {
				xil_printf("CAN: receive failed: %d\r\n", status);
			}
		}
	}
}
