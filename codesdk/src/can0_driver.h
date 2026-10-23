/*
 * PS CAN0 驱动对外接口
 *
 * 函数定义全部放在 can0_driver.c；main.c 只调用这里声明的接口。
 *
 * 接收走中断：Can0_Init() 只配硬件，Can0_SetupInterrupt() 才挂 GIC 并
 * 打开 CAN 中断。中断里把帧解码后直接推给 logger，主循环只负责落盘。
 */

#ifndef CAN0_DRIVER_H
#define CAN0_DRIVER_H

/* 该头文件独立使用时也能看到 SEND_SAFE_PROBE 等配置。 */
#include "can0_config.h"
#include "xcanps_hw.h"
#include "xil_exception.h"

/*
 * 解码后的一帧 CAN 报文。
 *
 * PS CAN 的 RX FIFO 每个报文占 4 个 32 位字：
 *   raw[0] = IDR   : ID 被拆成 ID1/ID2 两段，另有 SRR/IDE/RTR 标志位
 *   raw[1] = DLCR  : DLC 在高 4 位 [31:28]，低 16 位是接收时间戳
 *   raw[2] = DW1R  : 数据字节 0~3
 *   raw[3] = DW2R  : 数据字节 4~7
 *
 * 注意 IDR 不是线性 ID，不能直接 & 0x1FFFFFFF，详见 Can0_DecodeFrame()。
 */
typedef struct {
	u32 Id;        /* 仲裁 ID，已从 ID1/ID2 分片还原为线性值 */
	u8  Ide;       /* 1 = 扩展帧(FF)，0 = 标准帧 */
	u8  Rtr;       /* 1 = 远程帧 */
	u8  Dlc;       /* 数据长度 0~8 */
	u16 Timestamp; /* PS CAN 内部 16 位接收时间戳 */
	u8  Data[8];   /* 数据字节，Data[0] 为总线上的第 1 个字节 */
} Can0Frame;

/*
 * 接收统计，用来判断「有没有丢帧」。
 *
 *   Frames       : 中断里成功取到的总帧数
 *   Gaps         : 相邻帧时间戳差落在丢帧窗口内的次数（疑似丢帧）
 *   FifoOverflow : 硬件 RX FIFO 溢出次数，>0 说明中断没来得及搬
 *   BusOff       : 进入总线关闭状态的次数
 *   Errors       : 其它总线错误次数
 *
 * 三个计数都为 0、且 Gaps 为 0，才算真的没丢帧。
 */
typedef struct {
	u32 Frames;
	u32 Gaps;
	u32 FifoOverflow;
	u32 BusOff;
	u32 Errors;
} Can0RxStats;

/* 初始化 PS CAN0、配置位时序并进入正常模式。此时还没有开中断。 */
int Can0_Init(void);

/*
 * 挂 GIC、注册回调、打开 CAN 接收中断。
 *
 * 必须在 Can0_Init() 之后、且 logger 已经就绪之后调用，否则中断里
 * 入队的数据没地方放。
 */
int Can0_SetupInterrupt(void);

/* 读取统计快照。 */
void Can0_GetRxStats(Can0RxStats *Stats);

/*
 * 读 CAN 错误计数器寄存器（ECR）。
 *
 *   返回值 bit[7:0]   = TEC，发送错误计数
 *   返回值 bit[15:8]  = REC，接收错误计数
 *
 * 排查总线问题很有用：
 *   TEC 一直涨到 128 以上 -> 发出去的帧没人 ACK（电机没上电 / 没接线）
 *   TEC = 0 且收不到帧    -> 总线正常，是对面不说话
 */
u32 Can0_GetErrorCounters(void);

/*
 * 取走中断里记录的错误类型并清零（和 ESR 位定义一致）：
 *   bit0 CRC 错  bit1 格式错  bit2 填充错  bit3 位错  bit4 ACK 错
 *
 * 必须在中断里抓——驱动拿到错误后会清 ESR，主循环再读就是 0 了。
 *   bit4（ACK 错）  -> 发出去没人应答，总线上没有别的节点
 *   bit0~bit2      -> 波特率不匹配或物理层干扰（线长、终端电阻、屏蔽）
 *   bit3（位错）    -> 总线冲突或收发器异常
 */
u32 Can0_GetLastErrorMask(void);

/*
 * 如果发生过总线关闭，尝试恢复控制器。
 * 返回 1 表示这次真的做了恢复，0 表示什么都没做。
 * 放在主循环里调用，不要在中断里做复位。
 */
int Can0_RecoverIfNeeded(void);

/*
 * 无条件复位并重新配置 CAN 控制器。
 *
 * 会把 TEC/REC 错误计数清零、重新配位时序、重开中断。
 * 用于：REC/TEC 涨到错误被动（>127）之后手动清一次，
 * 或者怀疑控制器状态异常时（不必重启整个板子）。
 */
void Can0_Restart(void);

/*
 * 把 RX FIFO 原始 4 字内容解码成 Can0Frame。
 * raw 必须是 XCanPs_Recv() 填充的缓冲区，不能自己读寄存器。
 */
void Can0_DecodeFrame(const u32 *raw, Can0Frame *frame);

/*
 * 发送一帧 CAN 报文。
 *
 *   id   ：标准帧填 11 位 ID，扩展帧填 29 位 ID（线性值，不是 IDR 分片格式）
 *   ide  ：1 = 扩展帧，0 = 标准帧
 *   rtr  ：1 = 远程帧，0 = 数据帧
 *   data ：数据字节，data[0] 是总线上第 1 个字节
 *   dlc  ：数据长度 0~8
 *
 * TX FIFO 满时会在内部短暂重试，超时返回 XST_FAILURE。
 */
int Can0_SendFrame(u32 id, u8 ide, u8 rtr, const u8 *data, u8 dlc);

/*
 * 同上，但 TX FIFO 满时立即返回失败，不重试、不 usleep。
 *
 * **只能在中断上下文用**：Can0_SendFrame() 里的重试循环带 usleep，
 * 放进中断会把系统卡死。
 */
int Can0_SendFrameNoWait(u32 id, u8 ide, u8 rtr, const u8 *data, u8 dlc);

/*
 * 把另一个中断源挂到 CAN 用的那个 GIC 实例上。
 *
 * GIC 在 Can0_SetupInterrupt() 里已经初始化过，这个函数复用同一个实例，
 * 不会破坏已连好的 CAN 中断。调用前必须已经执行过 Can0_SetupInterrupt()。
 */
int Can0_ConnectIrq(u32 IrqId, Xil_ExceptionHandler Handler, void *Ref);

/*
 * 按 PS CAN 的 IDR 分片格式拼出扩展帧 ID 寄存器值。
 *
 * 直接写 XCanPs_CreateIdValue(ext_id, ...) 是错的：该宏的第 1 个参数是
 * 11 位标准 ID，会写进 [31:21]，29 位扩展 ID 必须拆成 ID1(高 11 位)
 * 和 ID2(低 18 位) 两段。
 */
static inline u32 Can0_MakeExtIdValue(u32 ext_id, u32 remote)
{
	ext_id &= 0x1FFFFFFFU;

	return ((((ext_id >> 18) & 0x7FFU) << XCANPS_IDR_ID1_SHIFT) |
			(1U << XCANPS_IDR_SRR_SHIFT) |
			(1U << XCANPS_IDR_IDE_SHIFT) |
			(((ext_id & 0x3FFFFU) << XCANPS_IDR_ID2_SHIFT) &
					XCANPS_IDR_ID2_MASK) |
			(remote ? 1U : 0U));
}

/* 按 PS CAN 的 IDR 格式拼出标准帧 ID 寄存器值。 */
static inline u32 Can0_MakeStdIdValue(u32 std_id, u32 remote)
{
	return ((((std_id & 0x7FFU) << XCANPS_IDR_ID1_SHIFT) &
			XCANPS_IDR_ID1_MASK) |
			(remote ? 1U : 0U));
}

/*
 * 电机手册第 7.2 节的报文描述格式：13 个字节。
 *
 *   字节1      帧信息：bit7=FF(帧格式)，bit6=RTR(帧类型)，bit3..0=DLC
 *   字节2..5   报文识别码：29 位 ID 左对齐到 32 位，低 3 位无效
 *   字节6..13  数据
 *
 * 这是上位机/USB-CAN 的封装格式，不是 PS CAN 寄存器格式。
 * 把收到的帧还原成这 13 个字节，就能和 USB-CAN 抓包逐字节对照。
 */
#define CAN_MANUAL_FRAME_BYTES 13U

static inline void Can0_FrameToManual(const Can0Frame *frame, u8 *out)
{
	u32 id_field;
	u8 i;

	/* 标准帧 ID 占基本 ID 的位置（bit31..21），扩展帧左对齐到 bit31..3。 */
	if (frame->Ide != 0U) {
		id_field = (frame->Id & 0x1FFFFFFFU) << 3U;
	} else {
		id_field = (frame->Id & 0x7FFU) << 21U;
	}

	out[0] = (u8)(((frame->Ide != 0U) ? 0x80U : 0x00U) |
			((frame->Rtr != 0U) ? 0x40U : 0x00U) |
			(frame->Dlc & 0x0FU));

	out[1] = (u8)(id_field >> 24);
	out[2] = (u8)(id_field >> 16);
	out[3] = (u8)(id_field >> 8);
	out[4] = (u8)id_field;

	for (i = 0U; i < 8U; ++i) {
		out[5U + i] = frame->Data[i];
	}
}

/*
 * 根据 SEND_SAFE_PROBE 配置发送一次测试帧。
 * 当前默认关闭，只有显式打开发送测试时才在实现中提供该接口。
 */
#if SEND_SAFE_PROBE
int Can0_SendSafeProbe(void);
#endif

#endif /* CAN0_DRIVER_H */
