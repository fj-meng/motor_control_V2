/*
 * 应用程序入口
 *
 * 启动顺序：
 *   1. 初始化 CAN0 硬件（此时还没开中断）
 *   2. 挂载 SD 卡（只挂载，不建文件；必须在开中断之前）
 *   3. 挂 GIC、打开 CAN 接收中断
 *   4. 把电机切到单圈绝对位置模式的速度力矩控制并打开 1ms 自动上传
 *   5. 进入主循环：搬环形缓冲 → 处理串口命令 → 每秒打印状态
 *
 * 默认只收不存：帧进环形缓冲、每秒打印，然后直接丢掉，卡上一个字节不写。
 * 串口输入 writemotordata 之后才开始按正常流程落盘。
 *
 * 中断只负责收帧入队，落盘全部在主循环里做，两者靠 2 秒的环形缓冲解耦。
 */

#include "can0_config.h"
#include "can0_driver.h"
#include "console.h"
#include "logger.h"
#include "motor_can_commands.h"
#include "motor_driver.h"
#include "timer_pl.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "xtime_l.h"

/*
 * 标准帧应答解码（手册 7.4.4，单圈绝对位置模式）。
 *
 * 6 字节应答的布局：
 *   data[0]      电机 ID（用于多机鉴别）
 *   data[1..2]   当前位置，16 位，0x8000 = 0°，32768 个码值 = 360°
 *   data[3]      当前速度 [11:4]
 *   data[4]      当前速度 [3:0] | 当前力矩 [11:8]
 *   data[5]      当前力矩 [7:0]
 *
 * 速度和力矩都是 12 位，0x800 为零点，半量程对应各自的满量程常数。
 */
static void PrintSignedHundred(const char *Name, s32 Val100)
{
	s32 Abs = Val100;
	char Sign = '+';

	if (Abs < 0) {
		Sign = '-';
		Abs = -Abs;
	}

	xil_printf(" %s=%c%d.%02d", Name, Sign,
			(int)(Abs / 100), (int)(Abs % 100));
}

static void PrintStandardDetail(const LogRecord *Rec)
{
	s32 Pos16;
	s32 Pos32;
	s32 Vel;
	s32 Cur;
	s32 Deg100;
	s32 Rads100;
	s32 Amp100;
	s32 Turns;
	s64 Rem;

	if ((Rec->Ide != 0U) || (Rec->Dlc < 6U)) {
		return;
	}

	Vel = (s32)(((u16)Rec->Data[3] << 4) | ((u16)Rec->Data[4] >> 4));
	Cur = (s32)((((u16)Rec->Data[4] & 0x0FU) << 8) | (u16)Rec->Data[5]);

	/* 速度：0x800 = 0，半量程 0x800 对应 58.639 rad/s */
	Rads100 = ((Vel - (s32)MOTOR_VEL_ZERO) * 5864) / (s32)MOTOR_VEL_ZERO;

	/* 力矩：0x800 = 0，半量程 0x800 对应 32 A */
	Amp100 = ((Cur - (s32)MOTOR_CUR_ZERO) * 3200) / (s32)MOTOR_CUR_ZERO;

	xil_printf("  -> id=%u", (unsigned int)Rec->Data[0]);

	if (Rec->Dlc >= 8U) {
		/*
		 * 增量位置模式：位置 32 位有符号，1048576 = 1 圈（手册 7.6.3）。
		 *   data[6] = 位置[31:24]  data[7] = 位置[23:16]
		 *   data[1] = 位置[15:8]   data[2] = 位置[7:0]
		 * 这个是连续累加的，转多少圈都不回绕（±2048 圈内）。
		 */
		Pos32 = (s32)(((u32)Rec->Data[6] << 24) |
				((u32)Rec->Data[7] << 16) |
				((u32)Rec->Data[1] << 8) |
				(u32)Rec->Data[2]);

		Turns = Pos32 / (s32)MOTOR_FB_COUNT_PER_TURN;
		Rem = (s64)Pos32 % (s64)MOTOR_FB_COUNT_PER_TURN;
		if (Rem < 0) {
			/* 负角度：借一圈，保证 deg 是 0~360 的正数 */
			Rem += (s64)MOTOR_FB_COUNT_PER_TURN;
			--Turns;
		}
		Deg100 = (s32)((Rem * 36000) / (s64)MOTOR_FB_COUNT_PER_TURN);

		xil_printf(" turn=%d", (int)Turns);
		PrintSignedHundred("deg", Deg100);
	} else {
		/*
		 * 还没切到增量模式（应答还是 6 字节）：按单圈模式解，
		 * 0~360° 锯齿波。注意这个模式下位置顶到 360 就不会再涨了。
		 */
		Pos16 = (s32)(((u16)Rec->Data[1] << 8) | (u16)Rec->Data[2]);
		Deg100 = (((Pos16 - (s32)MOTOR_POS_ZERO) & 0x7FFF) * 36000)
				/ (s32)MOTOR_POS_ZERO;
		PrintSignedHundred("deg", Deg100);
	}

	PrintSignedHundred("rad/s", Rads100);
	PrintSignedHundred("A", Amp100);
	xil_printf("\r\n");
}

/*
 * 每秒打印一次运行状态。
 *
 * 第 1 行：本秒收帧数、累计、丢帧自检，以及当前的记录状态
 *   rx   本秒收到的帧数，1ms 上传时应该稳定在 1000 左右
 *   tot  累计收到帧数
 *   gap  相邻帧时间戳跳变的次数，应该一直是 0
 *   ovf  硬件 RX FIFO 溢出次数，应该一直是 0
 *   drop 环形缓冲满丢弃的条数，应该一直是 0
 *   REC  off 表示只收不存；开记录后显示文件名和文件里已有的帧数
 *
 * 第 2 行：最新一帧的内容（时间戳、ID、帧格式、长度、8 个数据字节）
 */
static void PrintLogStatus(void)
{
	static u64 LastTick;
	static u32 LastFrames;
	static u32 LastPolls;
	static u32 Started;

	XTime Now;
	u64 Period;
	u64 Elapsed;
	u64 AbsNs;
	Can0RxStats St;
	LogRecord Rec;
	u32 Rate;
	u32 PollRate;
	u32 Polls;
	u32 Frames;
	u32 Ecr;
	u32 Esr;
	u32 i;

	XTime_GetTime(&Now);
	Period = (u64)LOG_STATS_PERIOD_MS * (u64)(COUNTS_PER_SECOND / 1000U);
	Elapsed = (u64)Now - LastTick;

	if ((Started != 0U) && (Elapsed < Period)) {
		return;
	}

	/*
	 * 刷新晶振误差。放在这个时间门后面，天然就是每秒一次。
	 *
	 * 漏了这一行，TimerPl_ReadNs() 用的还是上一次的 ppm（甚至上电时的 0），
	 * AbsNs 就退化成没校正的原始值，秒末误差能到几十微秒。
	 * 函数内部只做两次 AXI 读 + 一个整数除法，约 1us，不影响打印节奏。
	 */
	(void)TimerPl_UpdatePpm();

	Can0_GetRxStats(&St);
	Ecr = Can0_GetErrorCounters();
	Esr = Can0_GetLastErrorMask();

	/*
	 * 按实际经过的时间折算，不能直接拿帧数当每秒帧数。
	 * 打印这一行本身要约 12ms，两次打印的真实间隔是 1.012s 而不是 1s，
	 * 直接相减会显示成 1011/s，看着像电机发快了。
	 */
	if (Elapsed == 0U) {
		Elapsed = 1U;
	}
	Rate = (u32)(((u64)(St.Frames - LastFrames) *
			(u64)COUNTS_PER_SECOND) / Elapsed);

	/*
	 * 轮询实际发出的拍数，用来确认定时器真的按 1kHz 在跑。
	 * 注意 Motor_StopPolling/StartPolling 会把计数清零，所以这里要防回绕。
	 */
	Polls = Motor_GetPollTicks();
	if (Polls >= LastPolls) {
		PollRate = (u32)(((u64)(Polls - LastPolls) *
				(u64)COUNTS_PER_SECOND) / Elapsed);
	} else {
		PollRate = 0U;
	}

	LastFrames = St.Frames;
	LastPolls = Polls;
	LastTick = (u64)Now;
	Started = 1U;

	if (Logger_IsRecording() != 0) {
		Frames = Logger_GetFileFrames();
		xil_printf("rx=%u/s tot=%u gap=%u ovf=%u drop=%u boff=%u"
				" tec=%u rec=%u emask=%x"
				" | poll=%u/s pmiss=%u tmode=%u tmiss=%u"
				" | REC: %u tppm=%d\r\n",
				Rate, St.Frames, St.Gaps, St.FifoOverflow,
				Logger_GetDropped(), St.BusOff,
				Ecr & 0xFFU, (Ecr >> 8) & 0xFFU, Esr,
				PollRate, Motor_GetPollMiss(),
				(unsigned int)Motor_GetPollTail(),
				Motor_GetPollTailMiss(), Frames,
				(int)TimerPl_GetPpmX100());
	} else {
		xil_printf("rx=%u/s tot=%u gap=%u ovf=%u drop=%u boff=%u"
				" tec=%u rec=%u emask=%x"
				" | poll=%u/s pmiss=%u tmode=%u tmiss=%u"
				" | REC: off tppm=%d\r\n",
				Rate, St.Frames, St.Gaps, St.FifoOverflow,
				Logger_GetDropped(), St.BusOff,
				Ecr & 0xFFU, (Ecr >> 8) & 0xFFU, Esr,
				PollRate, Motor_GetPollMiss(),
				(unsigned int)Motor_GetPollTail(),
				Motor_GetPollTailMiss(),
				(int)TimerPl_GetPpmX100());
	}

	if (Logger_GetLatest(&Rec) == 0) {
		xil_printf("latest: no frame yet\r\n");
		return;
	}

	/* 两个时间基准一起打出来：
	 *   utc 是绝对时刻（AbsNs + 文件头里的偏移量），对过表才有；
	 *   abs 是 PL 的上电后单调时间，没对表时它就是全部信息；
	 *   ts  是 CAN 硬件的 16 位微秒计数（1.966ms 回绕），帧间精度 30ns。 */
	AbsNs = ((u64)Rec.AbsNsHi << 32) | (u64)Rec.AbsNsLo;

	if (Logger_IsUtcValid() != 0) {
		u32 Y;
		u32 Mo;
		u32 D;
		u32 H;
		u32 Mi;
		u32 S;
		u64 UtcSec = AbsNs / 1000000000ULL;

		TimerPl_UnixToUtc((u32)((s32)UtcSec + Logger_GetUtcOffset()),
				&Y, &Mo, &D, &H, &Mi, &S);
		xil_printf("latest: utc=%04u-%02u-%02uT%02u:%02u:%02u",
				Y, Mo, D, H, Mi, S);
	} else {
		xil_printf("latest: utc=not-set");
	}

	xil_printf(" abs=%d.%09d ts=%u id=0x%x %s dlc=%u data=",
			(int)(AbsNs / 1000000000ULL),
			(int)(AbsNs % 1000000000ULL),
			Rec.Ts, Rec.Id, (Rec.Ide != 0U) ? "EXT" : "STD", Rec.Dlc);

	for (i = 0U; (i < Rec.Dlc) && (i < 8U); ++i) {
		xil_printf("%02x", Rec.Data[i]);
		if ((i + 1U) < Rec.Dlc) {
			xil_printf(" ");
		}
	}
	xil_printf("\r\n");

	/* 标准帧应答再补一行解码，省得对着十六进制换算。 */
	PrintStandardDetail(&Rec);
}

int main(void)
{
	int Status;

	/* SDK BSP 的 stdout 应设置为 ps7_uart_1，串口参数为 115200 8N1。 */
	xil_printf("\r\n=== H10010 PS CAN0 log test ===\r\n");
	xil_printf("UART1 stdout, CAN0 MIO10/MIO11, SDK 2017.4\r\n");

	Status = Can0_Init();
	if (Status != XST_SUCCESS) {
		xil_printf("CAN initialization failed\r\n");
		return XST_FAILURE;
	}

	/*
	 * 只挂载 SD 卡，不建文件。默认状态是「只收不存」，帧收到就丢。
	 * 挂载失败也继续跑：此时每秒打印照常，能看出确实在收帧。
	 */
	if (Logger_Init() == XST_SUCCESS) {
		xil_printf("LOG: SD mounted, NOT recording\r\n");
		xil_printf("LOG: type 'writemotordata' to start writing %s\r\n",
				LOG_FILE_NAME);
	} else {
		xil_printf("LOG: SD not available, frames will be discarded\r\n");
	}

	Console_Init();

	/*
	 * PL 时间基准。必须在开 CAN 中断之前初始化，否则第一批帧进中断时
	 * TimerPl_IsReady() 还是 0，那批帧的 AbsNs 会是 0。
	 *
	 * 失败也继续跑：AbsNs 记 0，CAN 接收和落盘完全不受影响，
	 * 事后 parse_log.py 会把这些记录单独标出来。
	 */
	if (TimerPl_Init() == XST_SUCCESS) {
		if (TimerPl_SelfTest() != 0) {
			xil_printf("TIME: PL clock ready, id=0x%08x\r\n",
					(unsigned int)TIMER_PL_ID);
			/*
			 * PL 只知道「上电后过了多久」，不知道是几点。
			 * 这里提醒一句，免得录完发现没有绝对时间。
			 */
			xil_printf("TIME: UTC not set, send 'setutc <unix_seconds>'"
					" before recording\r\n");
		} else {
			xil_printf("TIME: AXI write path broken (scratch readback)\r\n");
		}
	} else {
		xil_printf("TIME: PL clock NOT found at 0x%08x\r\n",
				(unsigned int)TIMER_PL_BASE);
		xil_printf("TIME: AbsNs will be 0, CAN logging unaffected\r\n");
	}

	Status = Can0_SetupInterrupt();
	if (Status != XST_SUCCESS) {
		xil_printf("CAN interrupt setup failed: %d\r\n", Status);
		return XST_FAILURE;
	}
	xil_printf("CAN: RX interrupt on, IRQ=%d\r\n", XPAR_XCANPS_0_INTR);

#if MOTOR_AUTO_INIT
	/*
	 * 只建立模式并写入目标速度和限流，不启动。
	 * 自动上传在 Motor_InitSpeedMode() 内部打开（切模式会把它清掉）。
	 * 电机此刻保持静止；要让它转需要显式调用 Motor_StartSpeed()。
	 */
	Status = Motor_InitSpeedMode(MOTOR_TARGET_SPEED_RPM,
			MOTOR_TARGET_CURRENT_A);
	if (Status == XST_SUCCESS) {
		xil_printf("Motor: mode ready, NOT started (motor stays still)\r\n");
	} else {
		xil_printf("Motor: speed mode init failed\r\n");
	}
#endif

#if SEND_SAFE_PROBE
	Status = Can0_SendSafeProbe();
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
#endif

	xil_printf("CMD: 'motormod2init' speed+incremental, %d rpm (not started)\r\n",
			(int)MOTOR_TARGET_SPEED_RPM);
	xil_printf("CMD: 'motoron' / 'motoroff' start / stop, 'help' lists all\r\n");

	/* 主循环：落盘、串口命令、打印都不允许长时间占用 CPU。 */
	while (1) {
		Logger_Process();
		Console_Poll();

		if (Can0_RecoverIfNeeded() != 0) {
			xil_printf("CAN: bus-off recovered\r\n");
		}

		PrintLogStatus();
	}

	return XST_SUCCESS;
}
