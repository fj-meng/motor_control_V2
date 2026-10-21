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
 * CAN 位时序：当前假定 CAN CLK=100 MHz，目标波特率=1 Mbps。
 *
 * 驱动接口接收的是寄存器编码值，硬件实际值比宏值大 1：
 *   实际分频系数 = 4 + 1 = 5
 *   实际 TSEG1    = 15 + 1 = 16 TQ
 *   实际 TSEG2    = 2 + 1 = 3 TQ
 *   同步段       = 1 TQ
 *   总 TQ        = 1 + 16 + 3 = 20 TQ
 *   波特率       = 100 MHz / (5 * 20) = 1 Mbps
 *   采样点       = (1 + 16) / 20 = 85%
 *
 * SJW 不参与标称波特率计算；当前宏值 0 对应实际 SJW=1 TQ。
 *
 * 如果 Vivado 中 CAN CLK=24 MHz，应改为：
 *   CAN_BRPR_1MBPS=1U，CAN_BTR_SJW=2U，CAN_BTR_TSEG2=2U，
 *   CAN_BTR_TSEG1=7U。
 */
#define CAN_BRPR_1MBPS      4U
#define CAN_BTR_SJW         0U
#define CAN_BTR_TSEG2       2U
#define CAN_BTR_TSEG1       15U

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
 *   0 = 只接收，不主动发送电机控制帧；
 *   1 = 初始化后发送驱动文件中的受控测试帧。
 */
#define SEND_SAFE_PROBE     0

/*
 * 手册 13 字节报文格式打印开关：
 *   0 = 只打印解码字段和 raw_idr/raw_dlcr；
 *   1 = 额外打印一行手册格式的 13 个字节，便于和 USB-CAN 抓包对照。
 * 打开后串口输出量约翻倍，高帧率调试时注意 RX FIFO 溢出。
 */
#define PRINT_MANUAL_FORMAT 0

/*
 * 接收滤波器开关：
 *   0 = 关闭，RX FIFO 接收总线上所有帧（复位后的默认状态）；
 *   1 = 只接收 RX_FILTER_ID/RX_FILTER_MASK 命中的帧，其余由硬件丢弃。
 *
 * 注意：AFIR/AFMR 和 IDR 是同一套位布局，不是线性 ID。
 * 掩码位为 1 表示该位必须匹配，为 0 表示不关心。
 * 下面这组配置表示：只收「标准帧、仲裁 ID=HOST_CAN_ID(100)」的报文。
 */
#define USE_RX_FILTER       0

/* 只比较 11 位标准 ID，并要求 IDE=0（标准帧）。 */
#define RX_FILTER_MASK      (((u32)0x7FFU << 21) | ((u32)1U << 19))
#define RX_FILTER_ID        ((u32)HOST_CAN_ID << 21)

/* 发送重试：TX FIFO 满时的重试次数和退避时间。 */
#define CAN_TX_RETRY_LIMIT  50U
#define CAN_TX_RETRY_DELAY_US  10U

/*
 * 接收打印限流：每 N 帧打印一帧，0 = 完全不打印。
 *
 * 115200 波特率下打印一行约 60 字符 ≈ 5ms，而电机自动上传是 1ms 一帧，
 * 逐帧打印必然导致 RX FIFO 溢出（只有 4 帧深度）。
 * 打开自动上传时务必把它调到 0 或 50 以上。
 */
#define CAN_RX_PRINT_EVERY_N    1U

#endif /* CAN0_CONFIG_H */
