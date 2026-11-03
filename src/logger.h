/*
 * CAN 记录器：中断里入队，主循环里落盘
 *
 * 数据流：
 *
 *   CAN0 RX FIFO --(中断)--+--> Ring[2048] --(主循环)--> FlushBuf[4KB]
 *                          |                                   |
 *                     Logger_Push()                     Logger_Flush()
 *                                                             |
 *                                                     FatFs -> XSdPs(ADMA2) -> SD
 *
 * 分工原则：
 *   - 中断里只做「取帧 + 入队」，绝不打串口、绝不碰 SD。
 *     SD 写一扇区要几毫秒，放在中断里必然把后续帧全丢。
 *   - 落盘只在主循环里做。落盘期间中断照常入队，靠 2 秒的环形缓冲兜住。
 *
 * 记录格式固定 32 字节，16 条正好占满一个 512 字节扇区，所以文件里
 * 记录不会跨扇区，用 Python 或 C 直接按 32 字节切就行。
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "xil_types.h"
#include "can0_config.h"

/* 单条记录字节数，必须能被 512 整除（16 条/扇区）。 */
#define LOG_RECORD_SIZE         32U
#define LOG_RECORDS_PER_SECTOR  16U

/*
 * 文件头占满一整个扇区。
 *
 * 这样记录区从 512 字节边界开始，之后每次写整扇区都是对齐的，
 * FatFs 就不会为了补半个扇区去做「读-改-写」，SD 卡写入快一倍以上。
 * 头里只有前 28 个字节有用，其余补零。
 *
 * 版本 2 相比版本 1 多存了 UTC 偏移量：
 *   记录里的 AbsNs 是「上电后过了多少纳秒」，单调、永不跳变，但不知道几点；
 *   文件头里的 UtcOffsetSec 是「UTC 秒 − sec_cnt」，一个标量。
 *   两者相加就是绝对时间：utc_ns = AbsNs + UtcOffsetSec × 1e9
 *
 * 为什么把偏移量放文件头而不是直接往记录里写 UTC：
 *   ① 记录永远单调，不会因为中途重新对表而跳变；
 *   ② 偏移量只是一个数，事后发现对错了，改这一个数就能修正整个文件；
 *   ③ AbsNs 数值小（上电几十秒就是几亿），浮点拟合不会丢精度。
 */
#define LOG_HEADER_SIZE         512U

/* 文件头里这两个字段的字节偏移，用于录制中原地改写。 */
#define LOG_HDR_OFF_FLAGS       16U
#define LOG_HDR_OFF_UTCOFF      20U

/* Flags 的 bit0：UtcOffsetSec 有效。 */
#define LOG_HDR_FLAG_UTC_VALID  0x00000001U

/* 一次落盘写入的记录数 = 扇区数 × 每扇区记录数。 */
#define LOG_RECORDS_PER_FLUSH   (LOG_SECTORS_PER_FLUSH * LOG_RECORDS_PER_SECTOR)

/*
 * 一条 CAN 记录。
 *
 * Ts      ：微秒时间戳。CAN 硬件在帧起始处锁存，16 位（1.966ms 回绕），
 *           驱动里已按回绕累加成 32 位。30ns 分辨率、零抖动，
 *           用来算帧间隔、查丢帧最准。
 * Id      ：线性仲裁 ID（标准帧 11 位 / 扩展帧 29 位），不是寄存器分片格式。
 * Dlc     ：0~8。
 * Ide     ：1 = 扩展帧，0 = 标准帧。
 * Rtr     ：1 = 远程帧。
 * Flag    ：保留，当前恒为 0，留给以后标记异常帧。
 * AbsNsLo/Hi：PL 给的时间基准，纳秒，64 位拆成两个 u32。
 *           分辨率 100ns，抖动约 ±60ns。
 *
 *           **注意：这是「上电后过了多久」，不是 UTC。**
 *           它是单调递增的，不会因为重新对表而跳变。
 *           要换成 UTC 用文件头里的 UtcOffsetSec：
 *               utc_ns = AbsNs + UtcOffsetSec × 1e9
 *           这个偏移量由串口命令 setutc 设置，只在开机后设一次。
 *
 *           为什么不直接存 UTC：记录永远单调；偏移量是一个标量，
 *           事后发现对错了改一个数就能修正整个文件；而且 AbsNs 数值小，
 *           浮点拟合不丢精度（Unix 纳秒是 1.8e18，float64 只剩 256ns 分辨率）。
 *
 *           Ts 回答「帧间隔多长」，AbsNs 回答「这一帧在上电后第几纳秒」，
 *           两者加上文件头偏移量才构成完整的绝对时间。
 *
 * 为什么拆成两个 u32 而不是一个 u64：
 *   u64 要求 8 字节对齐。放在偏移 20 会被插入 4 字节 padding，
 *   结构体变成 40 字节，扇区对齐的假设就崩了（logger.c 里的
 *   LogRecordSizeCheck 会在编译期直接报错）。两个 u32 正好 32 字节。
 */
typedef struct {
	u32 Ts;
	u32 Id;
	u8  Dlc;
	u8  Ide;
	u8  Rtr;
	u8  Flag;
	u8  Data[8];
	u32 AbsNsLo;        /* 偏移 20，绝对时间低 32 位 */
	u32 AbsNsHi;        /* 偏移 24，绝对时间高 32 位 */
	u8  Pad[4];         /* 偏移 28，补齐到 32 字节 */
} LogRecord;

/*
 * 挂载 SD 卡。只挂载，不建文件、不写卡。
 *
 * 默认状态就是「只收不存」：帧照常进环形缓冲、照常每秒打印，
 * 但 Logger_Process() 会直接丢掉，卡上一个字节都不写。
 */
int Logger_Init(void);

/*
 * 开始记录：建文件、写 512 字节文件头，之后才真正落盘。
 *
 * 由串口命令 writemotordata 触发。命令之前的帧已被丢弃，不会进文件。
 * 重复调用安全；返回 XST_SUCCESS 表示文件已经准备好。
 */
int Logger_StartRecording(void);

/* 是否正在记录。0 = 只收不存。 */
int Logger_IsRecording(void);

/*
 * 设置 UTC 偏移量，并写进文件头。
 *
 * Offset = UTC 秒 − PL 的 sec_cnt，由 TimerPl_GetUtcOffset() 给出。
 * Valid = 0 表示还没对过表，文件头里的 Flags 不置位。
 *
 * 三种时机都能用：
 *   ① 录制之前调用：下次 Logger_StartRecording() 会把它写进新文件头；
 *   ② 录制中调用：立刻原地改写文件头的这 8 个字节（不影响已有记录）；
 *   ③ 录制结束后调用：文件已关，改不了，只能下次生效。
 *
 * 因为记录里存的是「上电后的单调时间」，这个偏移量对整份文件是同一个数，
 * 所以哪怕录完才发现对错了，也能在 PC 上用 parse_log.py 重新算一遍。
 */
int Logger_SetUtcOffset(s32 Offset, int Valid);

/* 取当前生效的偏移量，调试用。 */
s32 Logger_GetUtcOffset(void);

/* 偏移量是否有效（对过表）。 */
int Logger_IsUtcValid(void);

/*
 * 停止记录：把环形缓冲和落盘缓冲里剩下的数据**全部写完**，
 * 然后 f_sync + f_close。
 *
 * 断电前必须先调这个。直接断电最多会丢 2 秒的数据——
 * 环形缓冲 2048 条 + 落盘缓冲 127 条都还在 RAM 里没落盘。
 *
 * 停止后可以再 Logger_StartRecording() 重新开录，但注意文件名是固定的，
 * 重新开始会 FA_CREATE_ALWAYS 覆盖上一次的文件。
 */
int Logger_StopRecording(void);

/*
 * 入队一条记录。**只允许在中断上下文调用**。
 *
 * 环形缓冲满时直接丢弃并计数，绝不阻塞——中断里不能等。
 *
 * 这是老接口，AbsNs 固定填 0。新代码请用 Logger_PushEx()。
 */
void Logger_Push(u32 Id, u8 Ide, u8 Rtr, u8 Dlc, const u8 *Data, u32 Ts);

/*
 * 入队一条带绝对时间的记录。**只允许在中断上下文调用**。
 *
 * AbsNs 由调用方（CAN 接收中断）在入口处读一次 TimerPl_ReadNs() 得到。
 * 一次中断排空多帧时，这几帧共用同一个 AbsNs——它们的先后顺序仍然
 * 由 Ts 精确给出（30ns），只是整体基准是"这次中断进来的时刻"。
 */
void Logger_PushEx(u32 Id, u8 Ide, u8 Rtr, u8 Dlc, const u8 *Data,
		u32 Ts, u64 AbsNs);

/* 主循环里反复调用：搬空环形缓冲，攒够或超时就落盘。不会长时间阻塞。 */
void Logger_Process(void);

/* 立刻把 FlushBuf 里已有的记录写下去。 */
void Logger_Flush(void);

/*
 * 记录文件里已经有多少帧。
 *
 * 按 FatFs 记的文件长度算：(长度 - 512 字节头) / 32。
 * 每次落盘后都做过 f_sync，所以这个数就是卡上文件的真实帧数。
 */
u32 Logger_GetFileFrames(void);

/*
 * 取出最近收到的一帧，成功返回 1，还没有任何帧时返回 0。
 *
 * 由主循环在 Logger_Process() 里顺手记录，不在中断里做，所以不会读到
 * 半新半旧的字段。
 */
int Logger_GetLatest(LogRecord *Out);

/* 统计量，用于心跳打印和「有没有丢帧」的判定。 */
u32 Logger_GetDropped(void);   /* 环形缓冲满而丢掉的条数 */
u32 Logger_GetWritten(void);   /* 已写入 SD 的字节数 */
u32 Logger_GetWrites(void);    /* 落盘次数 */
u32 Logger_GetErrors(void);    /* 写失败次数 */

#endif /* LOGGER_H */
