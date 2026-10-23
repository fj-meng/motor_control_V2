/*
 * CAN 记录器实现
 *
 * 生产者是 CAN 接收中断（Logger_Push），消费者是主循环（Logger_Process），
 * 两者只通过 RingHead / RingTail 两个游标交接，不需要关中断，也不会互等。
 *
 * 单核 Cortex-A9 上唯一的风险是编译器把记录体的写操作排到游标更新之后，
 * 所以在发布 RingHead 之前插了一条 dmb 屏障。
 */

#include "logger.h"
#include "ff.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "xtime_l.h"
#include <string.h>

/* 深度必须是 2 的幂，取模才能退化成按位与。 */
#define LOG_RING_MASK   (LOG_RING_RECORDS - 1U)

#if ((LOG_RING_RECORDS & LOG_RING_MASK) != 0U)
#error "LOG_RING_RECORDS 必须是 2 的幂"
#endif

/* 发布游标前的内存屏障：保证记录体先落地，再让消费者看到新数据。 */
#define LOG_BARRIER()   __asm__ __volatile__ ("dmb sy" : : : "memory")

/* 文件头占满一个扇区，前 24 字节有用，其余补零。 */
typedef struct {
	char Magic[8];      /* "CANLOG01" */
	u32  RecordSize;    /* 32 */
	u32  Version;       /* 1 */
	u32  Reserved;
	u32  StartTick;
	u8   Pad[LOG_HEADER_SIZE - 24U];
} LogHeader;

/* 结构体大小必须严格等于 32，否则扇区对齐的假设就不成立了。 */
typedef char LogRecordSizeCheck[(sizeof(LogRecord) == LOG_RECORD_SIZE) ? 1 : -1];
typedef char LogHeaderSizeCheck[(sizeof(LogHeader) == LOG_HEADER_SIZE) ? 1 : -1];

/* 64 字节对齐：SD 走 ADMA2 搬运，缓冲区对齐到 cache line 最省事。 */
static LogRecord Ring[LOG_RING_RECORDS] __attribute__((aligned(64)));
static LogRecord FlushBuf[LOG_RECORDS_PER_FLUSH] __attribute__((aligned(64)));

/* RingHead 只由中断写，RingTail 只由主循环写，无需临界区。 */
static volatile u32 RingHead;
static volatile u32 RingTail;

static FATFS FatFs;
static FIL   File;

static u8  Mounted;          /* SD 卡挂载成功 */
static u8  Recording;        /* 文件已打开，正在落盘 */
static u32 Pending;          /* FlushBuf 里已攒的记录数 */
static u32 Dropped;
static u32 Written;
static u32 WriteCount;
static u32 ErrorCount;
static u64 LastFlushTick;

/* 最近收到的一帧，只由主循环在 Logger_Process() 里更新。 */
static LogRecord Latest;
static u32 LatestValid;

/* 超时阈值换算成全局定时器的计数。 */
#define LOG_TIMEOUT_TICKS \
	((u64)LOG_FLUSH_TIMEOUT_MS * (u64)(COUNTS_PER_SECOND / 1000U))

static u64 Logger_Now(void)
{
	XTime T;

	XTime_GetTime(&T);
	return (u64)T;
}

/* 把 FRESULT 翻成名字，免得对着数字查手册。 */
static const char *Logger_FrString(FRESULT Fr)
{
	switch (Fr) {
	case FR_OK:                  return "FR_OK";
	case FR_DISK_ERR:            return "FR_DISK_ERR";
	case FR_INT_ERR:             return "FR_INT_ERR";
	case FR_NOT_READY:           return "FR_NOT_READY";
	case FR_NO_FILE:             return "FR_NO_FILE";
	case FR_NO_PATH:             return "FR_NO_PATH";
	case FR_INVALID_NAME:        return "FR_INVALID_NAME";
	case FR_DENIED:              return "FR_DENIED";
	case FR_EXIST:               return "FR_EXIST";
	case FR_INVALID_OBJECT:      return "FR_INVALID_OBJECT";
	case FR_WRITE_PROTECTED:     return "FR_WRITE_PROTECTED";
	case FR_INVALID_DRIVE:       return "FR_INVALID_DRIVE";
	case FR_NOT_ENABLED:         return "FR_NOT_ENABLED";
	case FR_NO_FILESYSTEM:       return "FR_NO_FILESYSTEM";
	case FR_MKFS_ABORTED:        return "FR_MKFS_ABORTED";
	case FR_TIMEOUT:             return "FR_TIMEOUT";
	case FR_LOCKED:              return "FR_LOCKED";
	case FR_NOT_ENOUGH_CORE:     return "FR_NOT_ENOUGH_CORE";
	case FR_TOO_MANY_OPEN_FILES: return "FR_TOO_MANY_OPEN_FILES";
	case FR_INVALID_PARAMETER:   return "FR_INVALID_PARAMETER";
	default:                     return "unknown";
	}
}

static void Logger_PrintResult(const char *What, FRESULT Fr)
{
	xil_printf("LOG: %s failed, FRESULT=%d (%s)\r\n",
			What, (int)Fr, Logger_FrString(Fr));
}

/*
 * 挂载 SD 卡。
 *
 * 只做挂载（读操作），不建文件、不写卡。真正的记录要等上位机发
 * writemotordata 命令，由 Logger_StartRecording() 去打开文件。
 * 这样默认状态下完全不碰卡，插一张坏卡也不影响接收和每秒打印。
 */
int Logger_Init(void)
{
	FRESULT fr;

	Mounted = 0U;
	Recording = 0U;
	RingHead = 0U;
	RingTail = 0U;
	Pending = 0U;
	Dropped = 0U;
	Written = 0U;
	WriteCount = 0U;
	ErrorCount = 0U;
	LatestValid = 0U;

	/* 第三个参数 1 = 立即挂载。挂载时才真正初始化 SD 控制器。 */
	fr = f_mount(&FatFs, "0:/", 1);
	if (fr != FR_OK) {
		Logger_PrintResult("mount", fr);

		/*
		 * 最常见的三种情况，直接把排查方向打出来，省得对着数字查手册。
		 */
		if (fr == FR_NO_FILESYSTEM) {
			xil_printf("LOG: card is alive but has no FAT volume.\r\n");
			xil_printf("LOG: format it as FAT32 (MBR). exFAT/NTFS will NOT work.\r\n");
		} else if (fr == FR_NOT_READY) {
			xil_printf("LOG: no card detected, check it is inserted.\r\n");
		} else if (fr == FR_DISK_ERR) {
			xil_printf("LOG: card init or sector read failed, try another card.\r\n");
		}

		return XST_FAILURE;
	}

	Mounted = 1U;
	return XST_SUCCESS;
}

/*
 * 开始记录：建文件、写 512 字节文件头，之后 Logger_Process() 才开始落盘。
 *
 * 命令到来之前收到的帧已经被丢掉了，所以文件里只有命令之后的帧。
 * 重复调用是安全的，已经在录就直接返回成功。
 */
int Logger_StartRecording(void)
{
	static LogHeader Header;
	FRESULT fr;
	UINT bw = 0U;

	if (Recording != 0U) {
		return XST_SUCCESS;
	}

	/* 上电时没插卡或者卡没格好，这里再试一次。 */
	if (Mounted == 0U) {
		fr = f_mount(&FatFs, "0:/", 1);
		if (fr != FR_OK) {
			Logger_PrintResult("mount", fr);
			return XST_FAILURE;
		}
		Mounted = 1U;
	}

	/* FA_CREATE_ALWAYS：每次开始记录都重建文件，不追加历史数据。 */
	fr = f_open(&File, LOG_FILE_NAME, FA_CREATE_ALWAYS | FA_WRITE);
	if (fr != FR_OK) {
		Logger_PrintResult("open", fr);
		return XST_FAILURE;
	}

	memcpy(Header.Magic, "CANLOG01", 8U);
	Header.RecordSize = LOG_RECORD_SIZE;
	Header.Version = 1U;
	Header.Reserved = 0U;
	Header.StartTick = (u32)Logger_Now();

	fr = f_write(&File, &Header, LOG_HEADER_SIZE, &bw);
	if ((fr != FR_OK) || (bw != LOG_HEADER_SIZE)) {
		Logger_PrintResult("write header", fr);
		(void)f_close(&File);
		return XST_FAILURE;
	}

	/* 立刻同步一次，保证文件头和长度已经落到卡上。 */
	fr = f_sync(&File);
	if (fr != FR_OK) {
		Logger_PrintResult("sync header", fr);
		(void)f_close(&File);
		return XST_FAILURE;
	}

	/* 从零开始计数，环形缓冲里命令之前的旧帧不要带进新文件。 */
	Pending = 0U;
	Dropped = 0U;
	Written = LOG_HEADER_SIZE;
	WriteCount = 0U;
	ErrorCount = 0U;
	RingTail = RingHead;
	LastFlushTick = Logger_Now();
	Recording = 1U;

	return XST_SUCCESS;
}

int Logger_IsRecording(void)
{
	return (Recording != 0U) ? 1 : 0;
}

/*
 * 停止记录：把 RAM 里剩下的数据全部写完再关文件。
 *
 * 为什么不能直接断电：落盘是批量做的，停止的那一刻有两块数据还在内存里——
 *   ① 环形缓冲 Ring，最多 2048 条（2 秒）
 *   ② 落盘缓冲 FlushBuf 的零头，最多 127 条
 * 断电就全没了。这个函数把它们一条不剩地写下去。
 */
int Logger_StopRecording(void)
{
	u32 Flushed = 0U;
	u32 Before;
	u32 Head;
	FRESULT fr;

	if (Recording == 0U) {
		xil_printf("LOG: not recording\r\n");
		return XST_SUCCESS;
	}

	/*
	 * 先把环形缓冲搬空。
	 *
	 * **先取 RingHead 快照**：如果电机还在转、还在上传，CAN 中断会继续
	 * 往环里塞新数据，不取快照的话 RingTail 永远追不上 RingHead，
	 * 这个循环就退不出来了。快照之后新进来的帧直接丢弃——它们本来就
	 * 属于「停止之后」的数据，不该进这个文件。
	 */
	Head = RingHead;
	while (RingTail != Head) {
		while ((Pending < LOG_RECORDS_PER_FLUSH) && (RingTail != Head)) {
			FlushBuf[Pending] = Ring[RingTail & LOG_RING_MASK];
			++RingTail;
			++Pending;
			++Flushed;
		}
		Logger_Flush();
	}

	/*
	 * 再把落盘缓冲里的零头写干净。
	 *
	 * Logger_Flush() 正常只写整扇区，不足一扇区的零头留在缓冲里；
	 * 但 Pending 小于一扇区时它会整批写掉，所以循环一两次就空了。
	 * Before 判断是防止写失败时死循环。
	 */
	while (Pending > 0U) {
		Before = Pending;
		Logger_Flush();
		if (Pending >= Before) {
			xil_printf("LOG: flush stalled at %u records\r\n",
					(unsigned int)Pending);
			break;
		}
	}

	/* 同步目录项和 FAT，保证文件长度已经落到卡上。 */
	fr = f_sync(&File);
	if (fr != FR_OK) {
		Logger_PrintResult("sync on stop", fr);
	}

	/* 关闭文件，释放 FatFs 资源，目录项也就此写死。 */
	fr = f_close(&File);
	if (fr != FR_OK) {
		Logger_PrintResult("close", fr);
	}

	Recording = 0U;

	xil_printf("LOG: stopped, %u more records flushed, %u KB total\r\n",
			(unsigned int)Flushed, (unsigned int)(Written / 1024U));

	return XST_SUCCESS;
}

void Logger_Push(u32 Id, u8 Ide, u8 Rtr, u8 Dlc, const u8 *Data, u32 Ts)
{
	u32 Head = RingHead;
	LogRecord *Rec;

	/* 满了一律丢弃并计数，中断里不能等。 */
	if ((Head - RingTail) >= LOG_RING_RECORDS) {
		++Dropped;
		return;
	}

	Rec = &Ring[Head & LOG_RING_MASK];
	Rec->Ts = Ts;
	Rec->Id = Id;
	Rec->Dlc = Dlc;
	Rec->Ide = Ide;
	Rec->Rtr = Rtr;
	Rec->Flag = 0U;
	memcpy(Rec->Data, Data, 8U);

	/* 记录体全部写完，才允许消费者看到这一条。 */
	LOG_BARRIER();
	RingHead = Head + 1U;
}

void Logger_Flush(void)
{
	UINT Recs;
	UINT Want;
	UINT bw = 0U;
	UINT Remain;
	FRESULT fr;

	if ((Recording == 0U) || (Pending == 0U)) {
		return;
	}

	/*
	 * 尽量只写整扇区。
	 *
	 * 文件头占满 512 字节，所以只要每次写的都是 512 的整数倍，文件
	 * 偏移就一直扇区对齐，FatFs 不会为了补半个扇区去做「读-改-写」。
	 * 零头留在缓冲里等下一批，只有总线彻底静默时才会写一次半扇区。
	 */
	Recs = (Pending / LOG_RECORDS_PER_SECTOR) * LOG_RECORDS_PER_SECTOR;
	if (Recs == 0U) {
		Recs = Pending;
	}
	Want = Recs * LOG_RECORD_SIZE;

	/*
	 * f_write 内部按扇区下发，最终由 XSdPs 用 ADMA2 从 DDR 搬到卡上。
	 * 这一步是阻塞的，但期间 CAN 中断照常往环形缓冲里塞数据。
	 */
	fr = f_write(&File, FlushBuf, Want, &bw);
	if ((fr != FR_OK) || (bw != Want)) {
		++ErrorCount;
		Logger_PrintResult("write", fr);
	} else {
		Written += bw;
		++WriteCount;
	}

	/* 同步目录项和 FAT，掉电最多损失最后这一批。 */
	fr = f_sync(&File);
	if (fr != FR_OK) {
		++ErrorCount;
		Logger_PrintResult("sync", fr);
	}

	/* 把没写出去的零头挪到缓冲头部。 */
	Remain = Pending - Recs;
	if (Remain > 0U) {
		memmove(&FlushBuf[0], &FlushBuf[Recs],
				(Remain * LOG_RECORD_SIZE));
	}
	Pending = Remain;
	LastFlushTick = Logger_Now();
}

void Logger_Process(void)
{
	u64 Now;
	u32 Copied = 0U;

	if (Recording == 0U) {
		/*
		 * 没在记录：数据直接丢掉，但"最新一帧"要更新，否则每秒打印
		 * 里看不到任何内容。心跳里的 tot 照样在涨，能看出确实在收。
		 */
		if (RingTail != RingHead) {
			Latest = Ring[(RingHead - 1U) & LOG_RING_MASK];
			LatestValid = 1U;
		}
		RingTail = RingHead;
		return;
	}

	while ((Pending < LOG_RECORDS_PER_FLUSH) && (RingTail != RingHead)) {
		FlushBuf[Pending] = Ring[RingTail & LOG_RING_MASK];
		++RingTail;
		++Pending;
		++Copied;
	}

	/* 记下这一轮搬出来的最后一帧，供每秒打印用。
	 * 从 FlushBuf 取而不是再从 Ring 取，那个槽位已经交还给中断了。 */
	if (Copied > 0U) {
		Latest = FlushBuf[Pending - 1U];
		LatestValid = 1U;
	}

	if (Pending == 0U) {
		return;
	}

	/* 攒满一批就写。 */
	if (Pending >= LOG_RECORDS_PER_FLUSH) {
		Logger_Flush();
		return;
	}

	/* 没攒满：等超时再写，避免最后一点数据一直压在 RAM 里。 */
	Now = Logger_Now();
	if ((Now - LastFlushTick) >= LOG_TIMEOUT_TICKS) {
		Logger_Flush();
	}
}

u32 Logger_GetDropped(void)
{
	return Dropped;
}

u32 Logger_GetWritten(void)
{
	return Written;
}

u32 Logger_GetWrites(void)
{
	return WriteCount;
}

u32 Logger_GetErrors(void)
{
	return ErrorCount;
}

u32 Logger_GetFileFrames(void)
{
	DWORD Size;

	if (Recording == 0U) {
		return 0U;
	}

	/*
	 * BSP 里的 xilffs 是 FatFs R0.10b：文件长度是 FIL 上直接挂的 fsize
	 * 字段（DWORD），既没有 FSIZE_t 类型，也没有 f_size() 宏。
	 * 每次落盘后都 f_sync 过，所以这个数和卡上文件的真实长度一致。
	 */
	Size = File.fsize;
	if (Size <= (DWORD)LOG_HEADER_SIZE) {
		return 0U;
	}

	return (u32)((Size - (DWORD)LOG_HEADER_SIZE) / (DWORD)LOG_RECORD_SIZE);
}

int Logger_GetLatest(LogRecord *Out)
{
	if ((Out == NULL) || (LatestValid == 0U)) {
		return 0;
	}

	*Out = Latest;
	return 1;
}
