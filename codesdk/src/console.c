/*
 * 串口命令行
 *
 * 直接读写 ps7_uart_1 的寄存器，不建 XUartPs 实例——stdout 用的也是这个
 * 串口，再初始化一遍驱动容易和 xil_printf 抢状态。ps7_init 已经把串口
 * 配好波特率和数据格式了，这里只确认收发都使能，然后轮询 FIFO。
 *
 * 主循环里反复调用 Console_Poll()，每次最多读 CONSOLE_POLL_BUDGET 个
 * 字符就返回，避免长时间占着 CPU 把 CAN 中断的活耽误了。
 */

#include "console.h"
#include "can0_config.h"
#include "can0_driver.h"
#include "logger.h"
#include "motor_driver.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "xuartps_hw.h"
#include <string.h>

/* stdin 就是 ps7_uart_1，和 stdout 同一个口。 */
#define CONSOLE_UART_BASE   STDIN_BASEADDRESS

/* 一行最多 31 个字符，够放最长的命令。 */
#define CONSOLE_LINE_MAX    32U

/*
 * 每次轮询最多读多少个字符。
 *
 * 115200 下 1 个字符约 87us，16 个约 1.4ms，不至于把主循环卡住；
 * 一次只读 1 个的话，粘贴长命令会明显发涩，而且 RX FIFO 只有 64 字节。
 */
#define CONSOLE_POLL_BUDGET 16U

static char LineBuf[CONSOLE_LINE_MAX];
static u32  LineLen;

/* 执行一条已经收完的命令。 */
static void Console_Execute(const char *Cmd)
{
	if (strcmp(Cmd, "writemotordata") == 0) {
		if (Logger_StartRecording() == XST_SUCCESS) {
			xil_printf("CMD: recording started -> %s\r\n", LOG_FILE_NAME);
		} else {
			xil_printf("CMD: recording start FAILED, see LOG: above\r\n");
		}
		return;
	}

	/*
	 * 停止记录：把环形缓冲和落盘缓冲里剩下的数据全部写完，再 f_sync +
	 * f_close。**断电前必须先执行这个**，否则最多丢 2 秒的数据。
	 */
	if (strcmp(Cmd, "stoprecord") == 0) {
		if (Logger_StopRecording() == XST_SUCCESS) {
			xil_printf("CMD: recording stopped, safe to power off\r\n");
		} else {
			xil_printf("CMD: stoprecord FAILED, see LOG: above\r\n");
		}
		return;
	}

	/*
	 * 切「速度力矩 + 增量位置」模式（手册 7.4 / 7.6 / 7.8.1）。
	 *
	 * 依次发：停止(FF...FD) → 切速度力矩模式(FF...FA) →
	 *         切增量位置模式(扩展帧全 0) → 清零(FF...FE) → 普通指令(速度+限流)。
	 * 只写参数**不启动**，电机保持静止；后面 motoron 才真正转。
	 *
	 * 用增量模式是为了拿到连续的位置反馈：单圈模式的位置量程只有 ±360°，
	 * 连续同向旋转会顶在 0xFFFF 不动。
	 */
	if (strcmp(Cmd, "motormod2init") == 0) {
		if (Motor_InitSpeedMode(MOTOR_TARGET_SPEED_RPM,
				MOTOR_TARGET_CURRENT_A) == XST_SUCCESS) {
			xil_printf("CMD: SPEED + INCREMENTAL mode ready, %d rpm, "
					"NOT started\r\n", (int)MOTOR_TARGET_SPEED_RPM);
		} else {
			xil_printf("CMD: motormod2init FAILED\r\n");
		}
		return;
	}

	/* 启动电机（FF...FC），按 motormod2init 设定的速度转起来。 */
	if (strcmp(Cmd, "motoron") == 0) {
		if (Motor_IsReady() == 0) {
			xil_printf("CMD: motoron refused, run motormod2init first\r\n");
			return;
		}

		if (Motor_StartSpeed() == XST_SUCCESS) {
			xil_printf("CMD: motor ON, %d rpm\r\n",
					(int)MOTOR_TARGET_SPEED_RPM);
		} else {
			xil_printf("CMD: motoron FAILED\r\n");
		}
		return;
	}

	/*
	 * 停止电机（FF...FD）：电机无输出力，处于自由状态。
	 *
	 * 速度和限流参数还留在电机里，下次 motoron 直接发启动指令就能转，
	 * 不用重新 motormod2init。
	 */
	if (strcmp(Cmd, "motoroff") == 0) {
		if (Motor_StopSpeed() == XST_SUCCESS) {
			xil_printf("CMD: motor OFF (free state)\r\n");

			/*
			 * 特别提醒：motoroff 只管电机，录制还在继续。
			 * 这时候断电会丢掉 RAM 里没落盘的数据。
			 */
			if (Logger_IsRecording() != 0) {
				xil_printf("CMD: recording still ON, send "
						"'stoprecord' before power off\r\n");
			}
		} else {
			xil_printf("CMD: motoroff FAILED\r\n");
		}
		return;
	}

	/*
	 * 打开 / 关闭自动上传（手册 7.2.2 ③ 序号 9、7.5.5）。
	 *
	 * 帧内容：00 05 01 00 00 00 00 00 打开，00 05 00 ... 关闭。
	 * 这个设置电机掉电不保持，所以每次给电机重新上电后都要补发一次。
	 */
	if (strcmp(Cmd, "motorupload") == 0) {
		if (Motor_SetAutoUpload(1) == XST_SUCCESS) {
			xil_printf("CMD: auto upload requested ON\r\n");
		} else {
			xil_printf("CMD: motorupload FAILED\r\n");
		}
		return;
	}

	if (strcmp(Cmd, "motoruploadoff") == 0) {
		if (Motor_SetAutoUpload(0) == XST_SUCCESS) {
			xil_printf("CMD: auto upload requested OFF\r\n");
		} else {
			xil_printf("CMD: motoruploadoff FAILED\r\n");
		}
		return;
	}

	/*
	 * 切换轮询尾帧模式，用来现场试哪种组合能让电机「既转又有应答」。
	 *
	 *   0 = 只发速度指令（电机能转，但实测不回应答）
	 *   1 = 先 FD 停止再发速度（复刻上位机，实测电机不转）
	 *   2 = 先发速度再发 FC 启动（默认，FC 每拍重新使能）
	 *
	 * 只改一个变量，定时器中断下一拍就生效，不用重新烧录。
	 */
	if (strncmp(Cmd, "motorpoll", 9) == 0) {
		const char *Arg = Cmd + 9;

		while (*Arg == ' ') {
			++Arg;
		}

		if (*Arg == '\0') {
			xil_printf("CMD: poll tail = %u (0=speed, 1=stop+speed, "
					"2=speed+start, 3=speed dither)\r\n",
					(unsigned int)Motor_GetPollTail());
			return;
		}

		if ((*Arg >= '0') && (*Arg <= '3') && (Arg[1] == '\0')) {
			Motor_SetPollTail((u8)(*Arg - '0'));
		} else {
			xil_printf("CMD: usage: motorpoll 0|1|2|3\r\n");
		}
		return;
	}

	/*
	 * 改轮询周期（微秒）。不带参数时显示当前值。
	 *
	 * 用来验证「是不是发太快把电机的应答队列灌满了」——实测每拍两帧时
	 * 约 2 秒（≈2128 拍）后应答断流，接近 2048，很像队列积压。
	 * 把周期放大到 2000（500Hz）或 5000 再观察即可验证。
	 */
	if (strncmp(Cmd, "motorperiod", 11) == 0) {
		const char *Arg = Cmd + 11;
		u32 Us = 0U;
		u8 Digits = 0U;

		while (*Arg == ' ') {
			++Arg;
		}

		if (*Arg == '\0') {
			xil_printf("CMD: poll period = %u us\r\n",
					(unsigned int)Motor_GetPollPeriod());
			return;
		}

		while ((*Arg >= '0') && (*Arg <= '9')) {
			Us = (Us * 10U) + (u32)(*Arg - '0');
			++Arg;
			++Digits;
		}

		if ((Digits == 0U) || (*Arg != '\0')) {
			xil_printf("CMD: usage: motorperiod <500..200000>\r\n");
			return;
		}

		(void)Motor_SetPollPeriod(Us);
		return;
	}

	/*
	 * 复位 CAN 控制器。
	 *
	 * 心跳里的 tec/rec 涨到 128 以上就是「错误被动」了，收不到帧。
	 * 这个命令不用重启板子就能把控制器和错误计数清干净，重新配位时序、
	 * 重开中断，然后重新开始收。
	 */
	if (strcmp(Cmd, "canreset") == 0) {
		Can0_Restart();
		xil_printf("CMD: CAN restarted, error counters cleared\r\n");
		return;
	}

	if (strcmp(Cmd, "help") == 0) {
		xil_printf("CMD: writemotordata  - start writing frames to %s\r\n",
				LOG_FILE_NAME);
		xil_printf("CMD: stoprecord     - flush all buffered frames, "
				"close file (do this before power off)\r\n");
		xil_printf("CMD: motormod2init  - speed+incremental, %d rpm, "
				"not started\r\n", (int)MOTOR_TARGET_SPEED_RPM);
		xil_printf("CMD: motoron        - start motor\r\n");
		xil_printf("CMD: motoroff       - stop motor\r\n");
		xil_printf("CMD: motorupload    - turn auto upload ON\r\n");
		xil_printf("CMD: motoruploadoff - turn auto upload OFF\r\n");
		xil_printf("CMD: motorpoll N    - tail: 0=speed 1=stop+speed "
				"2=speed+start 3=dither\r\n");
		xil_printf("CMD: motorperiod N  - poll period in us (500..200000)\r\n");
		xil_printf("CMD: canreset       - reset CAN controller, clear errors\r\n");
		return;
	}

	xil_printf("CMD: unknown '%s', try: help\r\n", Cmd);
}

void Console_Init(void)
{
	u32 Cr;

	/*
	 * ps7_init 通常已经把 TX/RX 都打开了，这里只是兜底：
	 * 万一只开了 TX，读 FIFO 会永远为空，命令就收不到。
	 * 先清掉 TX_DIS/RX_DIS 再置 TX_EN/RX_EN，只 OR 使能位是不够的。
	 */
	Cr = XUartPs_ReadReg(CONSOLE_UART_BASE, XUARTPS_CR_OFFSET);
	Cr &= ~(u32)XUARTPS_CR_EN_DIS_MASK;
	Cr |= ((u32)XUARTPS_CR_RX_EN | (u32)XUARTPS_CR_TX_EN);
	XUartPs_WriteReg(CONSOLE_UART_BASE, XUARTPS_CR_OFFSET, Cr);

	LineLen = 0U;
}

void Console_Poll(void)
{
	u32 Budget = CONSOLE_POLL_BUDGET;
	u32 Sr;
	char Ch;

	while (Budget > 0U) {
		Sr = XUartPs_ReadReg(CONSOLE_UART_BASE, XUARTPS_SR_OFFSET);
		if ((Sr & XUARTPS_SR_RXEMPTY) != 0U) {
			break;
		}

		Ch = (char)(XUartPs_ReadReg(CONSOLE_UART_BASE,
				XUARTPS_FIFO_OFFSET) & 0xFFU);
		--Budget;

		/* 回车或换行都当作一行结束，两种终端习惯都照顾到。 */
		if ((Ch == '\r') || (Ch == '\n')) {
			xil_printf("\r\n");
			LineBuf[LineLen] = '\0';
			if (LineLen > 0U) {
				Console_Execute(LineBuf);
			}
			LineLen = 0U;
			continue;
		}

		/* 退格：删掉一个字符，顺便把屏幕上的也擦掉。 */
		if ((Ch == 0x08) || (Ch == 0x7F)) {
			if (LineLen > 0U) {
				--LineLen;
				xil_printf("\b \b");
			}
			continue;
		}

		/* 只收可见 ASCII，控制字符一律忽略。 */
		if ((Ch >= 0x20) && (Ch <= 0x7E)) {
			if (LineLen < (CONSOLE_LINE_MAX - 1U)) {
				LineBuf[LineLen] = Ch;
				++LineLen;
				xil_printf("%c", Ch);
			}
			continue;
		}
	}
}
