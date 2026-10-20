/*
 * 应用程序入口
 *
 * main.c 只负责启动顺序：初始化 UART 输出、初始化 CAN0、按配置决定
 * 是否发送测试帧，然后进入 CAN 接收循环。所有 CAN 函数定义都在
 * can0_driver.c 中。
 */

#include "can0_config.h"
#include "can0_driver.h"
#include "xil_printf.h"
#include "xstatus.h"

int main(void)
{
	int status;

	/* SDK BSP 的 stdout 应设置为 ps7_uart_1，串口参数为 115200 8N1。 */
	xil_printf("\r\nH10010 PS CAN0 receive test\r\n");
	xil_printf("UART1 stdout, CAN0 MIO10/MIO11, SDK 2017.4\r\n");

	status = Can0_Init();
	if (status != XST_SUCCESS) {
		xil_printf("CAN initialization failed\r\n");
		return XST_FAILURE;
	}

#if SEND_SAFE_PROBE
	status = Can0_SendSafeProbe();
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
#else
	xil_printf("Receive-only mode: no motor command is sent\r\n");
#endif

	/* 该函数持续轮询接收 FIFO，正常情况下不会返回。 */
	Can0_RunReceiveLoop();
	return XST_SUCCESS;
}
