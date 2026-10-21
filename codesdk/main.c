/*
 * 应用程序入口
 *
 * main.c 只负责启动顺序：初始化 UART 输出、初始化 CAN0、把电机切到
 * 多圈绝对值速度模式（不使能），然后进入 CAN 接收循环。
 *
 * 所有 CAN 外设函数在 can0_driver.c，电机协议函数在 motor_driver.c。
 */

#include "can0_config.h"
#include "can0_driver.h"
#include "motor_driver.h"
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

#if MOTOR_AUTO_INIT
	/*
	 * 只建立模式并写入目标速度和限流，不使能。
	 * 电机此刻保持静止；要让它转需要显式调用 Motor_StartSpeed()。
	 */
	status = Motor_InitSpeedMode(MOTOR_TARGET_SPEED_RPM,
			MOTOR_TARGET_CURRENT_A);
	Motor_SetAutoUpload(1);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: speed mode init failed\r\n");
	} else {
		xil_printf("Motor: ready, NOT enabled (motor stays still)\r\n");
	}
#endif

#if SEND_SAFE_PROBE
	status = Can0_SendSafeProbe();
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
#endif

	/* 该函数持续轮询接收 FIFO，正常情况下不会返回。 */
	Can0_RunReceiveLoop();
	return XST_SUCCESS;
}
