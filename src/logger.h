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
 * 头里只有前 24 个字节有用，其余补零。
 */
#define LOG_HEADER_SIZE         512U

/* 一次落盘写入的记录数 = 扇区数 × 每扇区记录数。 */
#define LOG_RECORDS_PER_FLUSH   (LOG_SECTORS_PER_FLUSH * LOG_RECORDS_PER_SECTOR)

/*
 * 一条 CAN 记录。
 *
 * Ts   ：微秒时间戳。CAN 硬件只给 16 位（65.5ms 回绕），驱动里已经
 *        按回绕累加成 32 位，所以可以直接用来算帧间隔。
 * Id   ：线性仲裁 ID（标准帧 11 位 / 扩展帧 29 位），不是寄存器分片格式。
 * Dlc  ：0~8。
 * Ide  ：1 = 扩展帧，0 = 标准帧。
 * Rtr  ：1 = 远程帧。
 * Flag ：保留，当前恒为 0，留给以后标记异常帧。
 */
typedef struct {
	u32 Ts;
	u32 Id;
	u8  Dlc;
	u8  Ide;
	u8  Rtr;
	u8  Flag;
	u8  Data[8];
	u8  Pad[12];
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
 */
void Logger_Push(u32 Id, u8 Ide, u8 Rtr, u8 Dlc, const u8 *Data, u32 Ts);

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
