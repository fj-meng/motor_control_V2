/*
 * timer_pl.h  —— PL 绝对时间基准的 PS 侧驱动
 *
 * 对应 RTL：timer_outer_top.v（GNSS 1PPS + 200 MHz 计数 + AXI4-Lite 从机）
 *
 * 用途：在 CAN 接收中断里读一次绝对时间，塞进 LogRecord 的 AbsNs 字段，
 *       和 CAN 数据一起落盘。PL 侧没有任何数据通路，只有寄存器。
 *
 * ---------------------------------------------------------------------------
 * 精度（重要，别搞混「分辨率」和「精度」）
 * ---------------------------------------------------------------------------
 *   分辨率       100 ns      —— 计数器的 1 个 tick
 *   量化误差     ±50 ns      —— 半个 tick
 *   晶振误差     0 ~ 50 µs   —— ±50ppm 普通晶振，秒内线性累积，秒末最大
 *   校正后       约 ±60 ns   —— 见下
 *
 * 晶振误差为什么能校正：
 *   PL 每秒把「这一秒实际有多少个 200MHz 周期」锁存进 PPS_CYC 寄存器，
 *   标称 200,000,000。多出来的部分就是晶振偏快多少 ppm。
 *   本驱动用这个值把 tick 换算成真实的纳秒，把锯齿误差抹掉。
 *
 *   TimerPl_UpdatePpm() 由主循环每秒调一次（不要在中断里调，它有 AXI 读），
 *   TimerPl_ReadNs() 在中断里用缓存值做校正，零额外开销。
 *
 *   不做这一步的后果：AbsNs 每秒从 0 爬到 50µs 再跳回去，形成锯齿，
 *   事后拟合会被这个锯齿污染，残差从 1µs 变成 14µs，真实抖动被淹没。
 * ---------------------------------------------------------------------------
 *
 * 另一个时间戳 Ts（CAN 控制器硬件给的，30ns 分辨率、无抖动）仍在 LogRecord
 * 的 Ts 字段里，两者配合使用的方法见 README 第 8 章。
 */

#ifndef TIMER_PL_H
#define TIMER_PL_H

#include "xil_types.h"

/* ---------------------------------------------------------------------------
 * AXI4-Lite 基地址
 *
 * 必须和 Vivado Block Design 里 Address Editor 分配的地址一致。
 * ------------------------------------------------------------------------- */
#define TIMER_PL_BASE           0x43C00000U

/* 寄存器偏移（32 位访问）。 */
#define TIMER_PL_REG_TS_TICK    0x00U   /* RO，读此地址会锁存快照 */
#define TIMER_PL_REG_TS_SEC     0x04U   /* RO，返回快照里的秒 */
#define TIMER_PL_REG_PPS_CYC    0x08U   /* RO，上一秒实测 200MHz 周期数 */
#define TIMER_PL_REG_STATUS     0x0CU   /* RO */
#define TIMER_PL_REG_ID         0x10U   /* RO，固定 0x54494D45 */
#define TIMER_PL_REG_SCRATCH    0x14U   /* RW，测试用 */
#define TIMER_PL_REG_DEBUG      0x18U   /* RO，PPS 调试字 */

#define TIMER_PL_STATUS_PPS_SEEN    (1U << 0)
#define TIMER_PL_STATUS_HOLDOVER    (1U << 1)
#define TIMER_PL_STATUS_PPS_CYC_VLD (1U << 2)

/* 调试字位定义（0x18）。连读两次比较计数有没有变化。 */
#define TIMER_PL_DBG_PPS_LEVEL      (1U << 0)    /* 滤波后的 1PPS 电平，1 = 空闲高 */
#define TIMER_PL_DBG_PPS_SEEN       (1U << 1)
#define TIMER_PL_DBG_FALL_CNT(w)    (((w) >> 8)  & 0xFFU)   /* 下降沿计数 */
#define TIMER_PL_DBG_RISE_CNT(w)    (((w) >> 16) & 0xFFU)   /* 上升沿计数 */

#define TIMER_PL_ID             0x54494D45U     /* ASCII "TIME" */

/* 1 tick = 100 ns；标称时钟 200 MHz。 */
#define TIMER_PL_TICK_NS        100ULL
#define TIMER_PL_NOMINAL_HZ     200000000U

/* 超过这个值就认为 ppm 读数异常（>1000ppm），不做校正。 */
#define TIMER_PL_PPM_LIMIT      100000

/* ---------------------------------------------------------------------------
 * 接口
 * ------------------------------------------------------------------------- */

/*
 * 探测并初始化。
 *
 * 只读一次 ID 寄存器做校验，不写任何硬件配置。
 * PL 没下载、地址配错、AXI 没连上，都会返回 XST_FAILURE，
 * 此时 TimerPl_IsReady() 返回 0，中断里的时间戳会自动退化成 0，
 * 不影响原来的 CAN 接收和落盘通路。
 */
int  TimerPl_Init(void);

/* 1 = PL 时间基准可用。中断里先判这个再读，避免 PL 没加载时白等总线。 */
int  TimerPl_IsReady(void);

/*
 * 读 64 位绝对时间，单位纳秒。**已经做过晶振 ppm 校正。**
 *
 * 内部是两次 AXI 读：先读 TS_TICK（同时锁存快照），再读 TS_SEC。
 * 这样保证拿到的秒和 tick 属于同一时刻——tick 每 100ns 变一次，
 * 而一次 AXI 读要 200ns 左右，两次独立读会拼出「新 tick + 旧秒」。
 *
 * 开销：约 0.6us @100MHz AXI + 校正运算（乘法和常数除法，几十个周期）。
 * 1kHz 调用占 CPU 约 0.1%。
 *
 * 注意：校正用的是 TimerPl_UpdatePpm() 缓存的值。如果从来没调过
 * UpdatePpm，等于没校正，精度会退化成秒末 50us。
 */
u64  TimerPl_ReadNs(void);

/*
 * 只读亚秒部分（100ns 计数，0 ~ 9999999），一次 AXI 读，约 0.3us。
 * 不带秒信息、不做校正，只适合「同一秒内」的粗略比较，一般不直接用。
 */
u32  TimerPl_ReadTick(void);

/* 状态位，见 TIMER_PL_STATUS_*。 */
u32  TimerPl_Status(void);

/* 是否处于丢星守时（PPS 断了 1.5 秒以上）。1 = 时间不可信。 */
int  TimerPl_IsHoldover(void);

/*
 * 读 0x18 调试字，用来诊断 PPS 为什么收不到。
 *
 * 用法：隔一秒连读两次，比较两个边沿计数：
 *   fall 在涨、rise 不动   -> 正常（低有效）
 *   rise 在涨、fall 不动   -> 极性反了，模块输出是高有效
 *   两个都不涨             -> 管脚上没信号（没接 / GNSS 没定位 / 管脚错）
 *   两个都在涨             -> 有信号，正常
 *
 * 用 TIMER_PL_DBG_FALL_CNT(w) / TIMER_PL_DBG_RISE_CNT(w) 取计数。
 */
u32  TimerPl_ReadDebug(void);

/*
 * 刷新并返回晶振频率误差，单位 ppm×100（例：5037 = +50.37 ppm）。
 *
 * 由主循环每秒调一次。处于 holdover、或还没测到完整一秒时返回上一次的值。
 * 内部有一次 AXI 读，不要放在中断里。
 */
s32  TimerPl_UpdatePpm(void);

/* 取上次 UpdatePpm 缓存的值，不做 AXI 读，可在任意上下文调用。 */
s32  TimerPl_GetPpmX100(void);

/* 测试用：写一个值再读回，验证 AXI 读写通路都正常。1 = 通过。 */
int  TimerPl_SelfTest(void);

/* ---------------------------------------------------------------------------
 * UTC 秒与秒内时间的组合
 *
 * 【为什么不能直接拼】
 *   tick 是硬件计数器，PPS 沿一到立刻归零（纳秒级同步）；
 *   UTC 秒是软件变量，什么时候更新取决于代码。
 *   两者之间必然有一个时间窗口，窗口内拼出来「新 tick + 旧秒」，差整整 1 秒。
 *
 * 【正确做法：算偏移量，不是每秒更新】
 *   UTC_sec = sec_cnt + offset
 *   offset 只在收到时间同步时更新一次，之后由 PPS 硬件保持 sec_cnt 递增，
 *   不会漂。所以不存在「每秒都要对齐」的竞争。
 *
 *   注意：offset 更新本身仍有边界风险（同步命令跨越秒边界到达时，
 *   命令里的秒可能属于上一秒）。TimerPl_SetUtcSeconds() 内部会检查 tick，
 *   太靠近秒边界时拒绝这次同步，调用方稍后重试即可。
 * ------------------------------------------------------------------------- */

/*
 * 用电脑给的时间建立 sec_cnt → UTC 的映射。
 *
 * 参数 UtcSec：当前 UTC 秒（Unix 时间戳或任意单调秒基准都行，
 *              只要后面读出来的值用同一个基准解释）。
 *
 * 返回 XST_SUCCESS 表示已采纳；XST_FAILURE 表示这次太靠近秒边界、
 * 或 PL 不可用，调用方应该过一会儿重试（比如 100 ms 后）。
 *
 * 只有一次 AXI 读，可以在串口命令处理里调用，不要在中断里调。
 */
int  TimerPl_SetUtcSeconds(u32 UtcSec);

/* UTC 映射是否已经建立。0 = 还没同步过，TimerPl_ReadUtcNs() 返回原始值。 */
int  TimerPl_UtcIsValid(void);

/* 取当前的 offset（UTC_sec − sec_cnt），调试用。 */
s32  TimerPl_GetUtcOffset(void);

/*
 * 读 64 位 UTC 时间（纳秒）= (sec_cnt + offset) × 1e9 + 校正后的亚秒。
 *
 * 如果还没同步过（UtcIsValid = 0），等价于 TimerPl_ReadNs()。
 * 开销和 TimerPl_ReadNs() 一样，可以在中断里调用。
 */
u64  TimerPl_ReadUtcNs(void);

/*
 * 把 Unix 秒（UTC）拆成日历时间。
 *
 * 自己算，不依赖 newlib 的 gmtime/localtime——那两个会拖进一堆 locale
 * 和时区表，而且裸机上还要配 reent。这里就是纯整数除法，约 30 条指令。
 *
 * 只做 UTC，不做时区转换。要看北京时间自己加 8 小时。
 */
void TimerPl_UnixToUtc(u32 Sec, u32 *Year, u32 *Month, u32 *Day,
		u32 *Hour, u32 *Minute, u32 *Second);

#endif /* TIMER_PL_H */
