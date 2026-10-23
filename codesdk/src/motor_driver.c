/*
 * 机致电机控制模块实现
 *
 * 使用【单圈绝对位置模式 + 速度力矩控制模式】，全部走标准帧。
 * 标准帧的 8 个字节按位打包了位置/速度/Kp/Kd/力矩 5 个物理量（手册 7.4.1），
 * 一条帧就能把目标速度和限流一起给过去，比多圈模式的扩展帧简单得多。
 *
 * 只有「自动上传开关」必须用扩展帧（Cmd=0x00 / Index=0x05），其余都是标准帧。
 *
 * 仲裁 ID 一律填电机 ID（1）；标准帧和扩展帧用的是同一个 ID，区别只在 IDE 位。
 */

#include "motor_driver.h"
#include "motor_can_commands.h"
#include "can0_driver.h"
#include "can0_config.h"
#include "xil_printf.h"
#include "xscutimer.h"
#include "xstatus.h"
#include "sleep.h"

/*
 * 两条指令之间的间隔。
 * 手册 7.1：电机从收到指令到应答需要 500us，最小通讯间隔也是 500us。
 * 这里留一倍余量，避免下一条指令和上一条的应答撞在一起。
 */
#define MOTOR_CMD_GAP_US    1000U

/*
 * 切模式之后的额外等待。
 *
 * 手册只规定普通指令间隔 500us，但切模式（FA / 增量 / 清零）要重构整个
 * 控制环，1ms 可能还没重整完，后面的指令就白发了。这里给 10ms 保守余量，
 * 只在初始化时多花几十毫秒，不影响正常运行。
 */
#define MOTOR_MODE_SETTLE_US    10000U

/* 确认自动上传是否真的开始的轮询参数。 */
#define MOTOR_UPLOAD_CHECK_MS   100U   /* 每次最多等多久 */
#define MOTOR_UPLOAD_TRY        2U     /* 最多试几次 */

/*
 * 初始化时每一步最多等多久的应答（毫秒）。
 *
 * 只用于定位问题：串口会打出每条指令收到了几条应答。正常情况下每条都该
 * 有 1 条，某一步是 0 就说明是那条指令让电机不说话了。
 * 收到应答会立刻返回，所以正常时每步只花 1ms 左右。
 */
#define MOTOR_REPLY_WAIT_MS     50U

/*
 * 周期轮询间隔（微秒）。
 *
 * 手册规定最小通讯间隔 500us（2kHz），这里取 1ms（1kHz）留一倍余量。
 * 改这个值只需要保证 >= 500。
 */
#define MOTOR_POLL_PERIOD_US    1000U

/* 上一次设定的速度码和限流码，Motor_StartSpeed() 和改速时复用。 */
static u16 MotorSpeedCode;
static u16 MotorCurrentCode;

/* 速度和限流是否已经写入过电机。 */
static u8 MotorModeReady;

/* 周期轮询用的私有定时器。 */
static XScuTimer MotorTimer;
static u8  PollRunning;
static volatile u32 PollTicks;      /* 实际发出的速度指令拍数 */
static volatile u32 PollMiss;       /* 速度指令没发出去的拍数（TX FIFO 满） */
static volatile u32 PollTailTicks;  /* 实际发出的尾部帧数 */
static volatile u32 PollTailMiss;   /* 尾部帧没发出去的次数 */

/* 轮询尾帧模式，运行时可改（motorpoll 命令）。 */
static volatile u8 PollTailMode = MOTOR_POLL_TAIL_DEFAULT;

/* 模式 3 用的抖动相位，每拍翻转一次。 */
static volatile u8 PollToggle;

/* 轮询周期，运行时可改（motorperiod 命令）。 */
static volatile u32 PollPeriodUs = MOTOR_POLL_PERIOD_US;

/*
 * 轮询里可能用到的两条特殊帧，内容固定，做成静态常量，中断里直接引用，
 * 不用每次重新填充。
 */
static const u8 MotorStopFrame[MOTOR_STD_CMD_DLC] = {
	(u8)MOTOR_STD_CMD_FILL, (u8)MOTOR_STD_CMD_FILL,
	(u8)MOTOR_STD_CMD_FILL, (u8)MOTOR_STD_CMD_FILL,
	(u8)MOTOR_STD_CMD_FILL, (u8)MOTOR_STD_CMD_FILL,
	(u8)MOTOR_STD_CMD_FILL, (u8)MOTOR_STD_CMD_STOP
};

static const u8 MotorStartFrame[MOTOR_STD_CMD_DLC] = {
	(u8)MOTOR_STD_CMD_FILL, (u8)MOTOR_STD_CMD_FILL,
	(u8)MOTOR_STD_CMD_FILL, (u8)MOTOR_STD_CMD_FILL,
	(u8)MOTOR_STD_CMD_FILL, (u8)MOTOR_STD_CMD_FILL,
	(u8)MOTOR_STD_CMD_FILL, (u8)MOTOR_STD_CMD_START
};

/* 发送 8 字节标准帧给电机。 */
static int Motor_SendStandard(const u8 *data)
{
	return Can0_SendFrame(MOTOR_CAN_ID, 0U, 0U, data, MOTOR_STD_CMD_DLC);
}

/* 发送 8 字节扩展帧给电机（只有自动上传这种参数类命令用）。 */
static int Motor_SendExtended(const u8 *data)
{
	return Can0_SendFrame(MOTOR_CAN_ID, 1U, 0U, data, MOTOR_EXT_CMD_DLC);
}

/*
 * 发送标准帧特殊指令：前 7 字节 0xFF，最后 1 字节是命令码（手册 7.2.1）。
 *   0xFC 启动  0xFD 停止  0xF9 力矩  0xFA 速度力矩  0xFB 位置速度力矩
 */
static int Motor_SendSpecial(u8 cmd)
{
	u8 data[MOTOR_STD_CMD_DLC];
	u8 i;

	for (i = 0U; i < (MOTOR_STD_CMD_DLC - 1U); ++i) {
		data[i] = MOTOR_STD_CMD_FILL;
	}
	data[MOTOR_STD_CMD_DLC - 1U] = cmd;

	return Motor_SendStandard(data);
}

/*
 * 等电机应答，最多等 timeout_ms 毫秒，返回这段时间收到的帧数。
 *
 * 判据是 CAN 接收中断里的帧计数，不是发送返回值——发送成功只代表帧进了
 * TX FIFO，电机到底回没回，只有接收计数能说明。
 *
 * 只在初始化阶段（主循环上下文、轮询关闭时）调用。此时总线上只有我们对
 * 电机的指令和它的应答，收到的每一帧都是应答，不用区分 ID。
 */
static u32 Motor_WaitReply(u32 timeout_ms)
{
	Can0RxStats Before;
	Can0RxStats Now;
	u32 Elapsed = 0U;

	Can0_GetRxStats(&Before);

	while (Elapsed < timeout_ms) {
		usleep(1000U);
		++Elapsed;

		Can0_GetRxStats(&Now);
		if (Now.Frames != Before.Frames) {
			return Now.Frames - Before.Frames;
		}
	}

	return 0U;
}

/* 发一条特殊指令并返回它收到的应答数，初始化时用来逐步定位。 */
static u32 Motor_SendSpecialChecked(u8 cmd)
{
	(void)Motor_SendSpecial(cmd);
	return Motor_WaitReply(MOTOR_REPLY_WAIT_MS);
}

/*
 * RPM 换算成标准帧的 12 位速度码（手册 7.4.2 ②）。
 *
 *   rad/s = RPM * 2π / 60
 *   code  = (rad/s / 满量程) * 0x800 + 0x800
 *
 * 例：HO10010 满量程 58.639rad/s，10rad/s -> (10/58.639)*2048+2048 = 2397 = 0x95D
 */
static u16 Motor_RpmToStdSpeedCode(float rpm)
{
	float rad_s;
	float code;

	rad_s = rpm * 6.28318531f / 60.0f;
	code = (rad_s / MOTOR_SPEED_FULL_SCALE_RAD_S) * (float)MOTOR_VEL_ZERO
			+ (float)MOTOR_VEL_ZERO;

	if (code < 0.0f) {
		code = 0.0f;
	} else if (code > (float)MOTOR_VEL_CODE_MAX) {
		code = (float)MOTOR_VEL_CODE_MAX;
	}

	return (u16)code;
}

/*
 * 电流（A）换算成标准帧的 12 位电流码（手册 7.4.2 ④）。
 *
 *   code = (A / 满量程) * 0x800 + 0x800
 *
 * 例：满量程 4A 的型号，2A -> (2/4)*2048+2048 = 3072 = 0xC00
 */
static u16 Motor_AmpToStdCurrentCode(float amp)
{
	float code;

	code = (amp / MOTOR_CURRENT_FULL_SCALE_A) * (float)MOTOR_CUR_ZERO
			+ (float)MOTOR_CUR_ZERO;

	if (code < 0.0f) {
		code = 0.0f;
	} else if (code > (float)MOTOR_CUR_CODE_MAX) {
		code = (float)MOTOR_CUR_CODE_MAX;
	}

	return (u16)code;
}

/*
 * 把 5 个物理量按手册 7.4.1 的位布局打包成 8 字节标准帧。
 *
 *   Byte1    位置[15:8]
 *   Byte2    位置[7:0]
 *   Byte3    速度[11:4]
 *   Byte4    速度[3:0] | Kp[11:8]
 *   Byte5    Kp[7:0]
 *   Byte6    Kd[11:8]
 *   Byte7    Kd[7:4] | 力矩[11:8]
 *   Byte8    力矩[7:0]
 *
 * 速度模式下位置/Kp/Kd 无效，位置填 0x8000（0 度）、Kp/Kd 填 0。
 */
static void Motor_FillStandardFrame(u8 *d, u16 pos, u16 vel, u16 kp, u16 kd,
		u16 cur)
{
	d[0] = (u8)MOTOR_STD_PACK_B1_POS_HI(pos);
	d[1] = (u8)MOTOR_STD_PACK_B2_POS_LO(pos);
	d[2] = (u8)MOTOR_STD_PACK_B3_VEL_HI(vel);
	d[3] = (u8)MOTOR_STD_PACK_B4_VEL_KP(vel, kp);
	d[4] = (u8)MOTOR_STD_PACK_B5_KP_LO(kp);
	d[5] = (u8)MOTOR_STD_PACK_B6_KD_HI(kd);
	d[6] = (u8)MOTOR_STD_PACK_B7_KD_CUR(kd, cur);
	d[7] = (u8)MOTOR_STD_PACK_B8_CUR_LO(cur);
}

/*
 * 发扩展帧切换电机模式（手册 7.2.2 ②）：
 *   02 00 = 单圈绝对位置模式（上电默认）
 *   02 01 = 多圈绝对位置模式
 *   02 02 = 增量位置模式
 *
 * 注意：这条命令本身电机是用「普通标准帧」应答的（6 字节，见 7.4），
 * 不是扩展帧。所以发完之后看到一帧 6 字节数据不代表切换失败。
 */
static int Motor_SendModeSwitch(u8 index)
{
	u8 frame[MOTOR_EXT_CMD_DLC];
	u8 i;

	for (i = 0U; i < MOTOR_EXT_CMD_DLC; ++i) {
		frame[i] = 0U;
	}
	frame[0] = (u8)MOTOR_EXT_CMD_SET_MODE;
	frame[1] = index;

	return Motor_SendExtended(frame);
}

/*
 * 切增量模式用哪一条扩展帧。手册给了两种写法，实测可以 A/B 切换：
 *   0 = 00 00 00 00 00 00 00 00（7.2.2 ② 序号 4「与原协议兼容」，7.8.1 雷达实例用的）
 *   1 = 02 02 00 00 00 00 00 00（7.2.2 ② 序号 3，Cmd/Index 语义最明确）
 */
#ifndef MOTOR_INC_MODE_USE_0202
#define MOTOR_INC_MODE_USE_0202     0
#endif

/*
 * 切到增量位置模式（手册 7.5.2 / 7.2.2 ②）。
 *
 * 手册 7.6.1 ②：进入增量模式后电机**延续之前的控制模式**，所以必须先发
 * FF...FA 切速度力矩模式，再发这条，否则会进「位置增量模式」。
 * 7.6.1 ③：切换时电机必须处于静置状态。
 * 7.6.1 ①：位置从 2 字节扩到 4 字节，20bit 分辨率，最大 ±2048 圈。
 * 7.6.1 ④：应答从 6 字节变 8 字节，可以用这个特征确认切换成功。
 */
static int Motor_SendIncrementalMode(void)
{
#if MOTOR_INC_MODE_USE_0202
	return Motor_SendModeSwitch((u8)MOTOR_EXT_MODE_INCREMENTAL_POS);
#else
	u8 frame[MOTOR_EXT_CMD_DLC];
	u8 i;

	for (i = 0U; i < MOTOR_EXT_CMD_DLC; ++i) {
		frame[i] = 0U;
	}

	return Motor_SendExtended(frame);
#endif
}

#if MOTOR_AUTO_UPLOAD_ON_INIT
/*
 * 打开自动上传，并**确认电机真的开始传了**。
 *
 * 注意：现在默认走「定时器轮询」而不是自动上传（见 Motor_StartPolling），
 * 所以这个开关默认关。留着是因为万一以后要回到上传模式还能用。
 *
 * 切模式会把上传设置清掉，而且发完 00 05 01 之后电机要过一会儿才开始发。
 * 光看发送成功没意义（帧只是进了 TX FIFO），得看接收帧计数有没有涨。
 */
static void Motor_StartUploadChecked(void)
{
	Can0RxStats Before;
	Can0RxStats Now;
	u32 Try;
	u32 Wait;

	for (Try = 0U; Try < MOTOR_UPLOAD_TRY; ++Try) {
		Can0_GetRxStats(&Before);
		(void)Motor_SetAutoUpload(1U);

		for (Wait = 0U; Wait < MOTOR_UPLOAD_CHECK_MS; ++Wait) {
			usleep(1000U);
			Can0_GetRxStats(&Now);
			if (Now.Frames != Before.Frames) {
				xil_printf("Motor: upload confirmed, %u frames in %u ms\r\n",
						(unsigned int)(Now.Frames - Before.Frames),
						(unsigned int)(Wait + 1U));
				return;
			}
		}

		xil_printf("Motor: upload NOT confirmed (try %u/%u)\r\n",
				(unsigned int)(Try + 1U),
				(unsigned int)MOTOR_UPLOAD_TRY);
	}

	xil_printf("Motor: no upload yet, will retry after motoron\r\n");
}
#endif /* MOTOR_AUTO_UPLOAD_ON_INIT */

/* ---------------------------------------------------------------------------
 * 周期轮询（私有定时器中断驱动）
 * ------------------------------------------------------------------------- */

/*
 * 定时器中断：每 MOTOR_POLL_PERIOD_US 发一「轮」帧。
 *
 * 为什么用中断发而不是主循环发：主循环里有 SD 落盘（几毫秒）和串口打印
 * （十几毫秒），靠主循环轮询会让采样间隔抖得很厉害。私有定时器中断的
 * 间隔由硬件保证，稳定在 1ms。
 *
 * 一轮发什么由 PollTailMode 决定（运行时可用 motorpoll 命令切换）：
 *   0  只发速度指令
 *   1  先 FD 停止，再速度指令（复刻上位机；**实测电机不转**）
 *   2  先速度指令，再 FC 启动（默认；FC 每拍重新使能）
 *
 * 两帧之间不加延时：CAN 硬件会等前一帧发完（含帧间隔）再发下一帧，
 * 中间自然隔开约 130us。中断里绝不 usleep、绝不打印。
 */
static void Motor_PollIsr(void *CallBackRef)
{
	u8 frame[MOTOR_STD_CMD_DLC];
	u8 tail = PollTailMode;
	u16 speed = MotorSpeedCode;

	(void)CallBackRef;

	XScuTimer_ClearInterruptStatus(&MotorTimer);

	/*
	 * 模式 3：速度码在 N 和 N+1 之间交替。
	 *
	 * ±1 码 = 58.639/2048 = 0.029 rad/s ≈ 0.27 RPM，转速上完全看不出来，
	 * 但每拍发出去的内容都不一样。用来验证「电机只在状态变化时才回应答」
	 * 这个假设——如果这样能拿到应答，就说明之前 rx=0 确实是「内容重复」导致。
	 */
	if (tail == 3U) {
		if (PollToggle != 0U) {
			++speed;
		}
		PollToggle ^= 1U;
	}

	Motor_FillStandardFrame(frame, (u16)MOTOR_POS_ZERO, speed,
			0U, 0U, MotorCurrentCode);

	/* 模式 1：先发停止帧。发不出去（TX FIFO 满）记一笔，继续发速度。 */
	if (tail == 1U) {
		if (Can0_SendFrameNoWait(MOTOR_CAN_ID, 0U, 0U, MotorStopFrame,
				MOTOR_STD_CMD_DLC) != XST_SUCCESS) {
			++PollTailMiss;
		} else {
			++PollTailTicks;
		}
	}

	if (Can0_SendFrameNoWait(MOTOR_CAN_ID, 0U, 0U, frame,
			MOTOR_STD_CMD_DLC) != XST_SUCCESS) {
		++PollMiss;
	} else {
		++PollTicks;
	}

	/* 模式 2：速度指令之后补一条启动，让电机每拍都重新使能。 */
	if (tail == 2U) {
		if (Can0_SendFrameNoWait(MOTOR_CAN_ID, 0U, 0U, MotorStartFrame,
				MOTOR_STD_CMD_DLC) != XST_SUCCESS) {
			++PollTailMiss;
		} else {
			++PollTailTicks;
		}
	}
}

static int Motor_PollInit(void)
{
	XScuTimer_Config *Cfg;
	u32 Load;

	Cfg = XScuTimer_LookupConfig(XPAR_XSCUTIMER_0_DEVICE_ID);
	if (Cfg == NULL) {
		return XST_FAILURE;
	}

	if (XScuTimer_CfgInitialize(&MotorTimer, Cfg, Cfg->BaseAddr)
			!= XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* 私有定时器时钟 = CPU 时钟 / 2 */
	Load = (XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ / 2U)
			/ (1000000U / PollPeriodUs);

	XScuTimer_Stop(&MotorTimer);
	XScuTimer_DisableAutoReload(&MotorTimer);
	XScuTimer_LoadTimer(&MotorTimer, Load - 1U);
	XScuTimer_EnableAutoReload(&MotorTimer);
	XScuTimer_ClearInterruptStatus(&MotorTimer);
	XScuTimer_EnableInterrupt(&MotorTimer);

	return Can0_ConnectIrq(XPAR_PS7_SCUTIMER_0_INTR,
			(Xil_ExceptionHandler)Motor_PollIsr, (void *)&MotorTimer);
}

int Motor_StartPolling(void)
{
	if (PollRunning != 0U) {
		return XST_SUCCESS;
	}

	if (Motor_PollInit() != XST_SUCCESS) {
		xil_printf("Motor: poll timer init failed\r\n");
		return XST_FAILURE;
	}

	PollTicks = 0U;
	PollMiss = 0U;
	PollTailTicks = 0U;
	PollTailMiss = 0U;
	XScuTimer_Start(&MotorTimer);
	PollRunning = 1U;

	xil_printf("Motor: polling ON, %u us period, tail mode %u\r\n",
			(unsigned int)MOTOR_POLL_PERIOD_US,
			(unsigned int)PollTailMode);
	return XST_SUCCESS;
}

/*
 * 运行时切换轮询尾帧模式（0/1/2，含义见 motor_driver.h 的
 * MOTOR_POLL_TAIL_DEFAULT）。切换只改一个变量，中断下一拍就生效，
 * 不会打断正在跑的轮询。
 */
void Motor_SetPollTail(u8 mode)
{
	const char *Name;

	if (mode > 3U) {
		xil_printf("Motor: poll tail must be 0/1/2/3, got %u\r\n",
				(unsigned int)mode);
		return;
	}

	switch (mode) {
	case 0U: Name = "speed only"; break;
	case 1U: Name = "stop+speed"; break;
	case 2U: Name = "speed+start"; break;
	default: Name = "speed dither"; break;
	}

	PollTailMode = mode;
	PollToggle = 0U;
	xil_printf("Motor: poll tail mode = %u (%s)\r\n", (unsigned int)mode,
			Name);
}

u8 Motor_GetPollTail(void)
{
	return PollTailMode;
}

/*
 * 运行时改轮询周期（微秒）。
 *
 * 用来验证「是不是发太快把电机的应答队列灌满了」——实测模式 2 下约 2 秒
 * （≈2128 拍）后应答断流，这个数字接近 2048，很像发送队列积压。
 * 把周期放大到 2000us（500Hz）可以验证这个猜测。
 *
 * 定时器正在跑时只重设 Load，不重新 CfgInitialize（那会把中断连接冲掉）。
 */
int Motor_SetPollPeriod(u32 us)
{
	u32 Load;

	if ((us < 500U) || (us > 200000U)) {
		xil_printf("Motor: period must be 500..200000 us\r\n");
		return XST_INVALID_PARAM;
	}

	PollPeriodUs = us;

	if (PollRunning != 0U) {
		Load = (XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ / 2U)
				/ (1000000U / us);
		XScuTimer_Stop(&MotorTimer);
		XScuTimer_LoadTimer(&MotorTimer, Load - 1U);
		XScuTimer_ClearInterruptStatus(&MotorTimer);
		XScuTimer_Start(&MotorTimer);
	}

	xil_printf("Motor: poll period = %u us\r\n", (unsigned int)us);
	return XST_SUCCESS;
}

u32 Motor_GetPollPeriod(void)
{
	return PollPeriodUs;
}

void Motor_StopPolling(void)
{
	if (PollRunning == 0U) {
		return;
	}

	XScuTimer_Stop(&MotorTimer);
	XScuTimer_DisableInterrupt(&MotorTimer);
	PollRunning = 0U;
}

int Motor_IsPolling(void)
{
	return (PollRunning != 0U) ? 1 : 0;
}

u32 Motor_GetPollMiss(void)
{
	return PollMiss;
}

u32 Motor_GetPollTailTicks(void)
{
	return PollTailTicks;
}

u32 Motor_GetPollTailMiss(void)
{
	return PollTailMiss;
}

u32 Motor_GetPollTicks(void)
{
	return PollTicks;
}

int Motor_InitSpeedMode(float speed_rpm, float current_a)
{
	u8 frame[MOTOR_STD_CMD_DLC];
	int status;
	u32 Reply;

	/* 先停轮询，避免定时器中断插进来和我们抢 TX FIFO。 */
	Motor_StopPolling();

	MotorSpeedCode = Motor_RpmToStdSpeedCode(speed_rpm);
	MotorCurrentCode = Motor_AmpToStdCurrentCode(current_a);

	/*
	 * 下面每一步都统计应答数并打印。
	 *
	 * 为什么要这么啰嗦：2026-10-23 反复遇到「电机 ACK 但不回应答」，
	 * 靠猜根本分不清是哪条指令让它闭嘴的。有了这一行行的 reply=N，
	 * 一眼就能看出断点在哪一步——reply=0 的那条就是元凶。
	 */
	xil_printf("Motor: init start, vel_code=%u cur_code=%u\r\n",
			(unsigned int)MotorSpeedCode,
			(unsigned int)MotorCurrentCode);

	/* [1] 先停一次，确保电机处于自由状态再切模式（手册 7.3.1 建议）。 */
	Reply = Motor_SendSpecialChecked((u8)MOTOR_STD_CMD_STOP);
	xil_printf("Motor: [1] stop          reply=%u\r\n", (unsigned int)Reply);
	usleep(MOTOR_CMD_GAP_US);

	/*
	 * [2] 强制切回单圈绝对位置模式，建立**已知基线**。
	 *
	 * 电机不会因为 ARM 重烧程序而复位，它保留着上一次留下的模式。
	 * 如果上次停在增量模式，再发一次「切增量」就是重复操作，
	 * 实测会让电机彻底静默（CAN 照样 ACK 但固件不再应答）。
	 * 所以每次初始化都先把它拉回上电默认的单圈模式。
	 */
	(void)Motor_SendModeSwitch((u8)MOTOR_EXT_MODE_SINGLE_TURN_ABS);
	Reply = Motor_WaitReply(MOTOR_REPLY_WAIT_MS);
	xil_printf("Motor: [2] back to single reply=%u\r\n", (unsigned int)Reply);
	usleep(MOTOR_MODE_SETTLE_US);

	/* [3] 切速度力矩控制模式（FF...FA），速度字段才会被电机采用。 */
	Reply = Motor_SendSpecialChecked((u8)MOTOR_STD_CMD_SPEED_TORQUE_MODE);
	xil_printf("Motor: [3] speed-torque  reply=%u\r\n", (unsigned int)Reply);
	usleep(MOTOR_MODE_SETTLE_US);

	/*
	 * [4] 切增量位置模式，位置从 2 字节扩到 4 字节、±2048 圈。
	 * 这一步的应答是关键判据：有应答说明电机认了这条命令，
	 * 之后应答应该变成 8 字节。
	 */
	status = Motor_SendIncrementalMode();
	if (status != XST_SUCCESS) {
		xil_printf("Motor: [4] incremental send FAILED: %d\r\n", status);
	} else {
		Reply = Motor_WaitReply(MOTOR_REPLY_WAIT_MS);
		xil_printf("Motor: [4] incremental   reply=%u\r\n",
				(unsigned int)Reply);
	}
	usleep(MOTOR_MODE_SETTLE_US);

#if MOTOR_ZERO_ON_INIT
	/* [5] 把当前位置设为机械零位并清多圈计数（FF...FE）。 */
	Reply = Motor_SendSpecialChecked((u8)MOTOR_STD_CMD_SET_MECH_ZERO);
	xil_printf("Motor: [5] set zero      reply=%u\r\n", (unsigned int)Reply);
	usleep(MOTOR_MODE_SETTLE_US);
#endif

#if MOTOR_AUTO_UPLOAD_ON_INIT
	/*
	 * 重新打开自动上传，并确认真的开始传了。
	 *
	 * **切模式会把这个设置清掉**（实测：切完增量模式后 rx 直接掉到 0），
	 * 所以每次初始化都要补一次。手册 7.8.1 的配方也是这个顺序：
	 * 切速度力矩 → 切增量 → 打开自动上传 → 设速度 → 启动。
	 *
	 * 这里用带自检的版本：发完看帧计数有没有涨，没涨会重试。
	 */
	Motor_StartUploadChecked();
	usleep(MOTOR_CMD_GAP_US);
#endif

	/* [6] 写入目标速度和限流，但**不启动**：位置填 0x8000，Kp/Kd 填 0。 */
	Motor_FillStandardFrame(frame, (u16)MOTOR_POS_ZERO, MotorSpeedCode,
			0U, 0U, MotorCurrentCode);

	status = Motor_SendStandard(frame);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: [6] speed set send FAILED: %d\r\n", status);
		return status;
	}
	Reply = Motor_WaitReply(MOTOR_REPLY_WAIT_MS);
	xil_printf("Motor: [6] speed set     reply=%u\r\n", (unsigned int)Reply);
	usleep(MOTOR_CMD_GAP_US);

	MotorModeReady = 1U;

	xil_printf("Motor: SPEED-TORQUE + INCREMENTAL mode set, "
			"vel_code=%u(0x%03X) cur_code=%u, NOT started\r\n",
			(unsigned int)MotorSpeedCode, (unsigned int)MotorSpeedCode,
			(unsigned int)MotorCurrentCode);
	xil_printf("Motor: frame was %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
			frame[0], frame[1], frame[2], frame[3],
			frame[4], frame[5], frame[6], frame[7]);
	xil_printf("Motor: reply should now be 8 bytes (incremental)\r\n");

	return XST_SUCCESS;
}

int Motor_IsReady(void)
{
	return (MotorModeReady != 0U) ? 1 : 0;
}

/*
 * 启动电机。
 *
 * 先把速度和限流再写一遍，再发启动指令（FF...FC），最后补一次自动上传。
 *
 * 手册 7.2.1 序号 1 特别提醒：启动前没收到过设定命令，电机运动状态未知。
 * 所以每次启动都重新下一次参数，即使中途电机掉过电也能保证有有效值。
 *
 * 为什么这里还要开上传：实测增量模式下「未使能时发 00 05 01」电机不回帧、
 * 也不周期上报，要等使能之后才认。所以 motormod2init 里那次多半会失败，
 * 真正起作用的是启动之后这一次。
 */
int Motor_StartSpeed(void)
{
	u8 frame[MOTOR_STD_CMD_DLC];
	int status;
	u32 Reply;

	/* 先停轮询，避免定时器中断插进来和我们抢 TX FIFO。 */
	Motor_StopPolling();

	Motor_FillStandardFrame(frame, (u16)MOTOR_POS_ZERO, MotorSpeedCode,
			0U, 0U, MotorCurrentCode);

	status = Motor_SendStandard(frame);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: start prep failed: %d\r\n", status);
		return status;
	}
	Reply = Motor_WaitReply(MOTOR_REPLY_WAIT_MS);
	xil_printf("Motor: start param frame reply=%u\r\n", (unsigned int)Reply);
	usleep(MOTOR_CMD_GAP_US);

	status = Motor_SendSpecial((u8)MOTOR_STD_CMD_START);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: start failed: %d\r\n", status);
		return status;
	}
	Reply = Motor_WaitReply(MOTOR_REPLY_WAIT_MS);
	xil_printf("Motor: start FC frame     reply=%u\r\n", (unsigned int)Reply);
	usleep(MOTOR_CMD_GAP_US);

	xil_printf("Motor: STARTED, vel_code=%u\r\n",
			(unsigned int)MotorSpeedCode);

	/*
	 * 启动之后开始周期轮询。
	 *
	 * 从这里开始所有发送都由定时器中断完成，主循环只负责改 MotorSpeedCode
	 * 这种参数，不再直接发帧。这样采样间隔是硬件保证的，不受 SD 落盘和
	 * 串口打印影响。
	 */
	status = Motor_StartPolling();
	if (status != XST_SUCCESS) {
		return status;
	}

	/*
	 * 轮询开起来后立刻验证一次：200ms 内应该收到上百帧。
	 *
	 * 这一步能直接把「轮询没跑」「电机不回应答」「总线有问题」三种情况
	 * 区分开——发送统计只看得到发出去多少，收不到就是收不到。
	 */
	Reply = Motor_WaitReply(200U);
	if (Reply == 0U) {
		xil_printf("Motor: WARNING no reply in 200ms "
				"(sent %u frames, check mode/power)\r\n",
				(unsigned int)Motor_GetPollTicks());
	} else {
		xil_printf("Motor: polling OK, %u frames in 200ms\r\n",
				(unsigned int)Reply);
	}

	return XST_SUCCESS;
}

/* 停止电机：FF...FD，电机无输出力，处于自由状态。 */
int Motor_StopSpeed(void)
{
	int status;

	/*
	 * 先停轮询。
	 *
	 * 不只是为了避免抢 TX FIFO：轮询发的是速度指令，如果停完电机还继续
	 * 发速度指令，某些固件可能会重新使能，那就不安全了。
	 */
	Motor_StopPolling();

	status = Motor_SendSpecial((u8)MOTOR_STD_CMD_STOP);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: stop failed: %d\r\n", status);
		return status;
	}

	xil_printf("Motor: STOPPED (free state)\r\n");
	return XST_SUCCESS;
}

/* 改目标速度，只重写参数帧，不动启动/停止状态。 */
int Motor_SetSpeedRpm(float speed_rpm)
{
	u8 frame[MOTOR_STD_CMD_DLC];
	int status;

	MotorSpeedCode = Motor_RpmToStdSpeedCode(speed_rpm);

	/*
	 * 轮询开着的时候，改速只要更新 MotorSpeedCode 就够了——定时器中断
	 * 下一拍会把新速度发出去。这里绝不能直接发帧，会和中断抢 TX FIFO。
	 */
	if (PollRunning != 0U) {
		xil_printf("Motor: speed updated, vel_code=%u (next tick)\r\n",
				(unsigned int)MotorSpeedCode);
		return XST_SUCCESS;
	}

	Motor_FillStandardFrame(frame, (u16)MOTOR_POS_ZERO, MotorSpeedCode,
			0U, 0U, MotorCurrentCode);

	status = Motor_SendStandard(frame);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: set speed failed: %d\r\n", status);
		return status;
	}

	xil_printf("Motor: speed updated, vel_code=%u\r\n",
			(unsigned int)MotorSpeedCode);
	return XST_SUCCESS;
}

/*
 * 打开 / 关闭主动上传。
 *
 * 帧格式（手册 7.2.2 ③ 序号 9）：
 *   00 05 01 00 00 00 00 00   打开，1ms 周期
 *   00 05 00 00 00 00 00 00   关闭
 *
 * 这是唯一必须用扩展帧的命令：Cmd=0x00（设置参数）、Index=0x05（自动上传）。
 * 该参数掉电不保持，电机重新上电后要补发一次。
 */
int Motor_SetAutoUpload(u8 enable)
{
	u8 frame[MOTOR_EXT_CMD_DLC];
	u8 i;
	int status;

	frame[0] = (u8)MOTOR_EXT_CMD_SET_PARAMETER;
	frame[1] = (u8)MOTOR_EXT_PARAM_AUTO_UPLOAD;
	frame[2] = (enable != 0U) ? (u8)MOTOR_AUTO_UPLOAD_ENABLE
			: (u8)MOTOR_AUTO_UPLOAD_DISABLE;
	for (i = 3U; i < MOTOR_EXT_CMD_DLC; ++i) {
		frame[i] = 0U;
	}

	status = Motor_SendExtended(frame);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: auto upload %s failed: %d\r\n",
				(enable != 0U) ? "on" : "off", status);
		return status;
	}

	if (enable != 0U) {
		xil_printf("Motor: auto upload ON, %d ms period\r\n",
				MOTOR_AUTO_UPLOAD_PERIOD_MS);
	} else {
		xil_printf("Motor: auto upload OFF\r\n");
	}

	return XST_SUCCESS;
}
