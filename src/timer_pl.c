/*
 * timer_pl.c  —— PL 绝对时间基准的 PS 侧驱动实现
 *
 * 只依赖 xil_io.h，不依赖任何其他模块，可以直接加进现有工程。
 * 不修改任何原有文件，只在 CAN 接收中断里多调一次 TimerPl_ReadNs()。
 */

#include "timer_pl.h"
#include "xil_io.h"
#include "xstatus.h"

/* PL 时间基准是否可用。TimerPl_Init() 里置位，中断里先判它。 */
static volatile u32 TimerPlReady;

/*
 * 缓存的晶振误差，单位 ppm×100。
 * 由主循环的 TimerPl_UpdatePpm() 写，中断里的 TimerPl_ReadNs() 只读。
 * 32 位对齐读写，Cortex-A9 上原子，不需要关中断。
 */
static volatile s32 TimerPlPpm100;

int TimerPl_Init(void)
{
	u32 Id;

	TimerPlReady = 0U;
	TimerPlPpm100 = 0;

	Id = Xil_In32(TIMER_PL_BASE + TIMER_PL_REG_ID);
	if (Id != TIMER_PL_ID) {
		/*
		 * 常见原因：PL bitstream 没下载、Address Editor 里分配的地址
		 * 和 TIMER_PL_BASE 不一致、M_AXI_GP0 没使能。
		 * 这里不报错退出——原来的 CAN 记录功能不受影响，只是时间戳为 0。
		 */
		return XST_FAILURE;
	}

	TimerPlReady = 1U;
	return XST_SUCCESS;
}

int TimerPl_IsReady(void)
{
	return (TimerPlReady != 0U) ? 1 : 0;
}

/*
 * 读 64 位绝对时间（纳秒），已做 ppm 校正。
 *
 * 校正公式推导：
 *   设本地时钟偏快 ε（ε = ppm×1e-6），则 1 个 tick 实际是 100ns/(1+ε)。
 *   走了 tick 个 tick 的真实时间是  tick × 100ns / (1+ε) ≈ tick × 100ns × (1-ε)
 *   所以校正量 = tick × 100ns × ε = tick × ppm100 × 1e-6  (ns)
 *
 *   例：tick = 10,000,000（一秒）、ppm100 = 5000（+50ppm）
 *       校正量 = 1e7 × 5000 × 1e-6 = 50,000 ns = 50us  ✓
 *
 * 除以 1000000 是除以编译期常数，GCC 会优化成乘法移位，没有硬件除法开销。
 */
u64 TimerPl_ReadNs(void)
{
	u32 Tick;
	u32 Sec;
	s64 Ppm;
	s64 Base;
	s64 Corr;
	s64 Sub;

	Tick = Xil_In32(TIMER_PL_BASE + TIMER_PL_REG_TS_TICK);
	Sec = Xil_In32(TIMER_PL_BASE + TIMER_PL_REG_TS_SEC);

	/* 高 8 位是保留位，恒 0，这里掩一下防止以后改位域时踩坑。 */
	Tick &= 0x00FFFFFFU;

	Ppm = (s64)TimerPlPpm100;

	/* 读数异常时不做校正，宁可用原始值也不要用一个离谱的校正量。 */
	if ((Ppm > (s64)TIMER_PL_PPM_LIMIT) || (Ppm < -(s64)TIMER_PL_PPM_LIMIT)) {
		Ppm = 0;
	}

	Base = (s64)Tick * (s64)TIMER_PL_TICK_NS;      /* 原始 ns */
	Corr = ((s64)Tick * Ppm) / 1000000;            /* 校正量 ns，可正可负 */
	Sub = Base - Corr;

	/* 极端情况下校正量可能把亚秒部分压成负数，兜一下底。 */
	if (Sub < 0) {
		Sub = 0;
	}

	return ((u64)Sec * 1000000000ULL) + (u64)Sub;
}

u32 TimerPl_ReadTick(void)
{
	return Xil_In32(TIMER_PL_BASE + TIMER_PL_REG_TS_TICK) & 0x00FFFFFFU;
}

u32 TimerPl_Status(void)
{
	return Xil_In32(TIMER_PL_BASE + TIMER_PL_REG_STATUS);
}

int TimerPl_IsHoldover(void)
{
	return ((TimerPl_Status() & TIMER_PL_STATUS_HOLDOVER) != 0U) ? 1 : 0;
}

u32 TimerPl_ReadDebug(void)
{
	return Xil_In32(TIMER_PL_BASE + TIMER_PL_REG_DEBUG);
}

/*
 * ppm×100 = (Cyc / 200e6 - 1) × 1e8 = (Cyc - 200e6) / 2
 *
 * 推导：设实测周期数 Cyc，标称 N = 200,000,000。
 *   频率误差 = Cyc/N - 1
 *   换算成 ppm×100 就是乘以 1e8，即 (Cyc - N) × 1e8 / N = (Cyc - N) / 2
 * 所以硬件每多计 2 个周期，就是 +0.01 ppm，整数运算没有精度损失。
 */
s32 TimerPl_UpdatePpm(void)
{
	u32 Status;
	u32 Cyc;
	s32 Ppm;

	if (TimerPlReady == 0U) {
		return TimerPlPpm100;
	}

	Status = Xil_In32(TIMER_PL_BASE + TIMER_PL_REG_STATUS);

	/* 还没测到完整一秒，或者正在守时，这个数都不可信，保留上一次的值。 */
	if ((Status & TIMER_PL_STATUS_PPS_CYC_VLD) == 0U) {
		return TimerPlPpm100;
	}
	if ((Status & TIMER_PL_STATUS_HOLDOVER) != 0U) {
		return TimerPlPpm100;
	}

	Cyc = Xil_In32(TIMER_PL_BASE + TIMER_PL_REG_PPS_CYC);
	if (Cyc == 0U) {
		return TimerPlPpm100;
	}

	Ppm = (s32)(((s64)Cyc - (s64)TIMER_PL_NOMINAL_HZ) / 2);

	/*
	 * 上电后第一次测量是「复位到第一个 PPS」的残秒，不是完整一秒，
	 * 算出来的 ppm 会大得离谱。用范围判断挡掉，等下一秒自然就正常了。
	 */
	if ((Ppm > TIMER_PL_PPM_LIMIT) || (Ppm < -TIMER_PL_PPM_LIMIT)) {
		return TimerPlPpm100;
	}

	TimerPlPpm100 = Ppm;
	return Ppm;
}

s32 TimerPl_GetPpmX100(void)
{
	return TimerPlPpm100;
}

int TimerPl_SelfTest(void)
{
	u32 Back;

	if (TimerPlReady == 0U) {
		return 0;
	}

	Xil_Out32(TIMER_PL_BASE + TIMER_PL_REG_SCRATCH, 0xA5A55A5AU);
	Back = Xil_In32(TIMER_PL_BASE + TIMER_PL_REG_SCRATCH);

	return (Back == 0xA5A55A5AU) ? 1 : 0;
}

/* ===========================================================================
 * UTC 秒与秒内时间的组合
 * ========================================================================= */

/*
 * UTC_sec = sec_cnt + UtcOffset
 *
 * 只在 TimerPl_SetUtcSeconds() 里改，之后一直不变。
 * PPS 每秒让 sec_cnt +1，UTC 秒自然跟着 +1，不需要软件再去更新。
 */
static volatile s32 TimerPlUtcOffset;
static volatile u32 TimerPlUtcValid;

/* 太靠近秒边界的 tick 阈值：前 10 ms 不采纳同步命令。
 * 依据：电脑发命令到 PS 收到的延迟通常在几 ms 以内（UART 115200 下
 * 一条 20 字节命令约 1.7 ms）。如果收到时 tick 还不到 10 ms，
 * 说明命令很可能是在上一秒发出的，命令里的秒数属于上一秒，会算错 1 秒。 */
#define TIMER_PL_UTC_GUARD_TICK     100000U     /* 10 ms = 100000 × 100ns */

int TimerPl_SetUtcSeconds(u32 UtcSec)
{
	u32 Tick;
	u32 Sec1;
	u32 Sec2;
	u32 Retry;

	if (TimerPlReady == 0U) {
		return XST_FAILURE;
	}

	for (Retry = 0U; Retry < 8U; ++Retry) {
		/* 读 TS_TICK 会同时锁存快照，随后读到的 TS_SEC 属于同一时刻 */
		Tick = Xil_In32(TIMER_PL_BASE + TIMER_PL_REG_TS_TICK) & 0x00FFFFFFU;
		Sec1 = Xil_In32(TIMER_PL_BASE + TIMER_PL_REG_TS_SEC);

		/* 太靠近秒边界：命令可能跨越了秒，拒绝，让调用方稍后重试 */
		if (Tick < TIMER_PL_UTC_GUARD_TICK) {
			return XST_FAILURE;
		}

		/* 再读一次秒，确认这两次读之间没跨过秒边界 */
		Sec2 = Xil_In32(TIMER_PL_BASE + TIMER_PL_REG_TS_SEC);
		if (Sec1 != Sec2) {
			continue;
		}

		TimerPlUtcOffset = (s32)UtcSec - (s32)Sec1;
		TimerPlUtcValid = 1U;
		return XST_SUCCESS;
	}

	return XST_FAILURE;
}

int TimerPl_UtcIsValid(void)
{
	return (TimerPlUtcValid != 0U) ? 1 : 0;
}

s32 TimerPl_GetUtcOffset(void)
{
	return TimerPlUtcOffset;
}

u64 TimerPl_ReadUtcNs(void)
{
	u64 Raw;
	s64 Utc;

	Raw = TimerPl_ReadNs();

	if (TimerPlUtcValid == 0U) {
		return Raw;
	}

	Utc = (s64)Raw + ((s64)TimerPlUtcOffset * 1000000000LL);

	/* 时间基准还没对上时 offset 可能让结果为负，兜个底 */
	if (Utc < 0) {
		Utc = 0;
	}

	return (u64)Utc;
}

/*
 * 把 Unix 秒拆成日历时间（UTC）。
 *
 * 用的是 Howard Hinnant 的 days_from_civil 逆算法：把 1970-01-01 之后的
 * 天数按 400 年一个"纪元"分组，再用整数运算反解出年月日。
 * 相比逐月逐年减法，这里是常数时间，也不怕闰年边界。
 */
void TimerPl_UnixToUtc(u32 Sec, u32 *Year, u32 *Month, u32 *Day,
		u32 *Hour, u32 *Minute, u32 *Second)
{
	s32 Days = (s32)(Sec / 86400U);
	u32 Rem = Sec % 86400U;
	s32 Era;
	u32 Doe;
	u32 Yoe;
	s32 Y;
	u32 Doy;
	u32 Mp;
	u32 D;
	u32 M;

	if (Hour != 0) {
		*Hour = Rem / 3600U;
	}
	if (Minute != 0) {
		*Minute = (Rem % 3600U) / 60U;
	}
	if (Second != 0) {
		*Second = Rem % 60U;
	}

	Days += 719468;                     /* 平移到 0000-03-01 为纪元起点 */
	Era = (Days >= 0 ? Days : Days - 146096) / 146097;
	Doe = (u32)(Days - Era * 146097);   /* [0, 146096] */
	Yoe = (Doe - Doe / 1460U + Doe / 36524U - Doe / 146096U) / 365U;
	Y   = (s32)Yoe + Era * 400;
	Doy = Doe - (365U * Yoe + Yoe / 4U - Yoe / 100U);
	Mp  = (5U * Doy + 2U) / 153U;       /* [0, 11]，0 = 三月 */
	D   = Doy - (153U * Mp + 2U) / 5U + 1U;
	M   = (Mp < 10U) ? (Mp + 3U) : (Mp - 9U);

	if (Year != 0) {
		*Year = (u32)(Y + ((M <= 2U) ? 1 : 0));
	}
	if (Month != 0) {
		*Month = M;
	}
	if (Day != 0) {
		*Day = D;
	}
}
