/*
 * PS CAN0 测试程序配置
 *
 * 这里集中放置会随硬件平台或测试目标变化的宏。CAN 驱动函数本身不应
 * 直接散落这些数值，修改 CAN 时钟或 CAN ID 时只检查本文件即可。
 */

#ifndef CAN0_CONFIG_H
#define CAN0_CONFIG_H

#include "xparameters.h"
#include "motor_can_commands.h"

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
 * CAN 仲裁 ID 与反馈数据中的模块 ID 分开配置。
 * 当前抓包观察到：控制帧 ID=8、反馈帧 ID=50、模块 ID=1。
 * 如果电机参数中配置过其他 ID，只修改这里，不要修改指令码字典。
 */
#define CAN_COMMAND_ID      MOTOR_CAN_DEFAULT_COMMAND_ID
#define CAN_FEEDBACK_ID     MOTOR_CAN_DEFAULT_FEEDBACK_ID
#define MOTOR_MODULE_ID     MOTOR_CAN_DEFAULT_MODULE_ID

/* 旧代码兼容别名：MOTOR_CAN_ID 不再表示电机模块 ID。 */
#define MOTOR_CAN_ID        CAN_COMMAND_ID
#define HOST_CAN_ID         CAN_FEEDBACK_ID

/*
 * 发送测试开关：
 *   0 = 只接收，不主动发送电机控制帧；
 *   1 = 初始化后发送驱动文件中的受控测试帧。
 */
#define SEND_SAFE_PROBE     0

#endif /* CAN0_CONFIG_H */
