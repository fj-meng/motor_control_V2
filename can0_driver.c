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

/* PS CAN0 驱动实例和接收帧缓冲区只在本文件内部使用。 */
static XCanPs Can0;
static u32 RxFrame[CAN_FRAME_WORDS];

#if SEND_SAFE_PROBE
static u32 TxFrame[CAN_FRAME_WORDS];
#endif

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
	XCanPs_EnterMode(&Can0, XCANPS_MODE_NORMAL);

	while (XCanPs_GetMode(&Can0) != XCANPS_MODE_NORMAL) {
		/* 等待硬件确认进入正常模式。 */
	}

	xil_printf("CAN0 ready: 1 Mbps, motor ID=%d, host ID=%d\r\n",
			MOTOR_CAN_ID, HOST_CAN_ID);
	return XST_SUCCESS;
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

/* 打印一帧 PS CAN 接收数据。 */
static void PrintReceivedFrame(const u32 *frame)
{
	u8 *data = (u8 *)&frame[2];
	u32 dlc = (frame[1] & XCANPS_DLCR_DLC_MASK) >> XCANPS_DLCR_DLC_SHIFT;
	u32 timestamp = frame[1] & XCANPS_DLCR_TIMESTAMP_MASK;
	u32 i;

	xil_printf("RX raw_id=0x%08x dlc=%d hw_ts=0x%04x data=",
			frame[0], dlc, timestamp);

	/* 经典 CAN 最多 8 字节，防止异常 DLC 导致越界读取。 */
	for (i = 0U; i < dlc && i < 8U; ++i) {
		xil_printf("%02x", data[i]);
		if (i + 1U < dlc && i + 1U < 8U) {
			xil_printf(" ");
		}
	}
	xil_printf("\r\n");
}

/*
 * 轮询接收 FIFO。接收一帧后打印原始 ID、DLC、硬件时间戳和数据。
 * 后续正式版本可在这里替换为中断+环形缓冲区实现。
 */
void Can0_RunReceiveLoop(void)
{
	int status;

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
