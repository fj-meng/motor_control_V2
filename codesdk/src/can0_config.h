/*
 * PS CAN0 测试程序配置
 *
 * 这里集中放置会随硬件平台或测试目标变化的宏。CAN 驱动函数本身不应
 * 直接散落这些数值，修改 CAN 时钟或 CAN ID 时只检查本文件即可。
 */

#ifndef CAN0_CONFIG_H
#define CAN0_CONFIG_H

#include "xparameters.h"

/* Vivado 导出的 PS CAN0 设备编号。 */
#define CAN_DEVICE_ID       XPAR_XCANPS_0_DEVICE_ID

/* SDK 2017.4 的 XCanPs 驱动用 4 个 32 位字保存一帧 CAN 数据。 */
#define CAN_FRAME_WORDS     4U

/*
 * CAN 位时序：CAN CLK=100 MHz，目标波特率=1 Mbps。
 *
 * 驱动接口接收的是寄存器编码值，硬件实际值比宏值大 1：
 *   实际分频系数 = 4 + 1 = 5
 *   实际 TSEG1    = 14 + 1 = 15 TQ
 *   实际 TSEG2    = 3 + 1 = 4 TQ
 *   同步段       = 1 TQ
 *   总 TQ        = 1 + 15 + 4 = 20 TQ
 *   波特率       = 100 MHz / (5 * 20) = 1 Mbps
 *   采样点       = (1 + 15) / 20 = 80%
 *   SJW          = 1 + 1 = 2 TQ
 *
 * **2026-10-23 调整过**：原来 TSEG1=15 / TSEG2=2 / SJW=0，
 * 采样点 85%、同步宽度只有 1 TQ。总 TQ 仍是 20，波特率不变，
 * 只是把采样点挪到 80%（CAN 推荐 75%~87.5% 的中段），
 * SJW 提到 2 TQ，给相位抖动留出裕量。
 * 动机：心跳里 emask 长期有 bit0（CRC 错），REC 会累积到 128 变错误被动，
 * 之后收不到帧。采样点太靠后 + SJW 太小是最常见的两个诱因。
 *
 * 如果 Vivado 中 CAN CLK=24 MHz，应改为：
 *   CAN_BRPR_1MBPS=1U，CAN_BTR_SJW=2U，CAN_BTR_TSEG2=2U，
 *   CAN_BTR_TSEG1=7U。
 */
#define CAN_BRPR_1MBPS      4U
#define CAN_BTR_SJW         1U
#define CAN_BTR_TSEG2       3U
#define CAN_BTR_TSEG1       14U

/*
 * 电机和主机使用的 CAN 标准帧 ID。
 *
 * 按「ID = 目的地址」约定：
 *   MOTOR_CAN_ID = 1    目的地是电机，主机发送命令帧时填这个
 *   HOST_CAN_ID  = 100  目的地是主机，电机反馈帧填这个
 *
 * 接收侧：ID=100 是电机发给主机的，ID=1 是别的主机发给电机的。
 */
#define MOTOR_CAN_ID        1U
#define HOST_CAN_ID         100U

/*
 * 发送测试开关：
 *   0 = 不主动发送受控测试帧；
 *   1 = 初始化后发送驱动文件中的受控测试帧。
 */
#define SEND_SAFE_PROBE     0

/*
 * 接收滤波器开关：
 *   0 = 关闭，RX FIFO 接收总线上所有帧（复位后的默认状态）；
 *   1 = 只接收 RX_FILTER_ID/RX_FILTER_MASK 命中的帧，其余由硬件丢弃。
 *
 * 注意：AFIR/AFMR 和 IDR 是同一套位布局，不是线性 ID。
 * 掩码位为 1 表示该位必须匹配，为 0 表示不关心。
 * 下面这组配置表示：只收「标准帧、仲裁 ID=HOST_CAN_ID(100)」的报文。
 *
 * 标准帧应答是 ID=HOST_CAN_ID 的标准帧，开滤波正好命中；
 * 但自动上传等参数类命令走扩展帧，开滤波会把它们的应答挡掉，调试时保持 0。
 */
#define USE_RX_FILTER       0

/* 只比较 11 位标准 ID，并要求 IDE=0（标准帧）。 */
#define RX_FILTER_MASK      (((u32)0x7FFU << 21) | ((u32)1U << 19))
#define RX_FILTER_ID        ((u32)HOST_CAN_ID << 21)

/* 发送重试：TX FIFO 满时的重试次数和退避时间。 */
#define CAN_TX_RETRY_LIMIT  50U
#define CAN_TX_RETRY_DELAY_US  10U

/*
 * 丢帧自检窗口（微秒）。
 *
 * 电机自动上传是固定 1ms 一帧，所以相邻两帧的时间戳差应该约等于 1000。
 * 差值落在 [GAP_MIN, GAP_MAX] 之间说明中间少了一帧以上，计入 gap。
 * 上限取 10ms，是为了把「电机停发一段较长时间后再恢复」和「真的丢帧」
 * 区分开——长时间静默不算丢帧。
 */
#define CAN_RX_GAP_MIN_US   1500U
#define CAN_RX_GAP_MAX_US   10000U

/*
 * CAN 接收时间戳计数器的时钟频率。
 *
 * **这个值不是猜的，是 2026-10-23 用实测数据反推出来的。**
 *
 * 手册说时间戳「按位时间递增」，1Mbps 下 1 个计数应该正好是 1us。
 * 但实测不是。用 50RPM 稳态跑的一段数据核对：
 *   位置每帧增加 865 码 = 865/1048576 = 0.000825 圈
 *   50RPM = 0.8333 转/秒，所以帧间隔 = 0.000825/0.8333 = 1.000 ms
 *   而时间戳每帧增加 33329 个计数
 *   33329 / 1000 = 33.33
 *
 * 33.33 正好是 PS_CLK 33.333MHz 与 1MHz 的比值，说明这个计数器是用
 * PS_CLK 驱动的，不是位时间。所以 1 个计数 = 30ns，不是 1us。
 *
 * 不做这个换算的后果：文件里的时间戳被放大约 33 倍，1ms 的帧间隔显示成
 * 33ms，丢帧判断（1.5~10ms 窗口）会全部落到「总线静默」里，gap 永远不涨，
 * 看起来像是从来没丢过帧。
 *
 * 如果以后换了板子、PS_CLK 不同，用同样的方法重新反推这个值。
 */
#define CAN_TS_CLK_HZ       33333333U

/* ---------------------------------------------------------------------------
 * 记录器（CAN → RAM 环形缓冲 → SD 卡）
 * ------------------------------------------------------------------------- */

/*
 * 环形缓冲深度，必须是 2 的幂。
 *
 * 每条记录 32 字节，2048 条 = 64 KB，按 1kHz 算能缓存 2 秒。
 * SD 卡单次写 4 KB 通常几毫秒，个别劣质卡可能卡顿上百毫秒，
 * 2 秒的余量足够扛过去，这是「不丢帧」的主要保障。
 */
#define LOG_RING_RECORDS        2048U

/*
 * 每次落盘写多少个扇区。
 *
 * 512 字节/扇区 × 8 = 4 KB 一次写完，16 条记录/扇区 → 128 条记录。
 * 写得越大，SD 卡的命令开销摊得越薄，但数据在 RAM 里停留越久。
 * 128 条 = 128ms，兼顾效率和安全。
 */
#define LOG_SECTORS_PER_FLUSH   8U

/*
 * 攒不满一次也要落盘的超时（毫秒）。
 *
 * 电机停转后环形缓冲不再有数据，剩下的零头不能一直压在 RAM 里，
 * 否则断电就丢了，文件里也看不到最后一段。
 */
#define LOG_FLUSH_TIMEOUT_MS    200U

/* 记录文件名。FatFs 的卷标前缀 0: 对应 SD0。 */
#define LOG_FILE_NAME           "0:/CANLOG.BIN"

/* 心跳打印周期（毫秒）。串口 115200 下 1 秒一行不影响实时性。 */
#define LOG_STATS_PERIOD_MS     1000U

#endif /* CAN0_CONFIG_H */
