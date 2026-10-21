/*
 * 机致电机控制模块
 *
 * 当前只实现「多圈绝对位置模式 + 速度控制」这一条路径：
 *   上电初始化时把电机切到多圈绝对值速度模式，并把目标速度和限流写进去，
 *   但**不使能**（ZZ=0），电机保持静止。真正让它转需要显式调用
 *   Motor_StartSpeed()。
 *
 * 协议依据：手册 7.7「多圈绝对位置模式的发送和接收」。
 * 该模式下发送和应答都是扩展帧，仲裁 ID = 电机 ID。
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "xil_types.h"

/*
 * 上电自动初始化开关：
 *   1 = main() 里调用 Motor_InitSpeedMode()，只建模式不使能；
 *   0 = 不碰电机，只跑 CAN 接收。
 */
#define MOTOR_AUTO_INIT         1

/*
 * 初始化时写入电机的运行参数。使能后电机就按这里的速度运行，
 * 所以改速度只需要改 MOTOR_TARGET_SPEED_RPM。
 */
#define MOTOR_TARGET_SPEED_RPM  10.0f  /* 目标转速，单位 RPM */
#define MOTOR_TARGET_CURRENT_A  1.0f   /* 运行最大电流，单位 A */

/* 多圈模式的位置环 Kp / Kd，取值 0~255。 */
#define MOTOR_TARGET_POS_KP     25U
#define MOTOR_TARGET_POS_KD     20U

/*
 * 把电机切换到多圈绝对值速度模式，写入目标速度和限流，但不使能。
 *
 * 按手册 7.7.2 ① 的要求，依次发送 Cmd=100、101、102 三条扩展帧，
 * 三条的模式字段相同（速度模式）且使能位都为 0。发送前会先发一次
 * 标准帧停止指令，确保电机处于自由状态（7.3.1 的建议）。
 *
 * 返回 XST_SUCCESS 只表示三帧都已成功发出，不代表电机一定接受了；
 * 电机是否进入该模式要看应答帧。
 */
int Motor_InitSpeedMode(float speed_rpm, float current_a);

/*
 * 使能电机，按 Motor_InitSpeedMode 写入的速度运行。
 * 调用后电机会立刻转起来。
 */
int Motor_StartSpeed(void);

/* 失能电机，停止输出，处于自由状态。 */
int Motor_StopSpeed(void);

/* 改目标速度但保持失能状态。 */
int Motor_SetSpeedRpm(float speed_rpm);

/*
 * 打开 / 关闭电机主动上传（手册 7.2.2 ③ 序号 9、7.5.5）。
 *
 *   enable = 1：电机每 1ms 主动发一帧实时数据，期间仍会应答主机指令
 *   enable = 0：关闭
 *
 * 发送的是扩展帧 00 05 01 00 00 00 00 00（关时第三字节为 00）。
 *
 * 两个必须注意的点：
 *   1. 1ms 一帧意味着每秒 1000 帧，RX FIFO 只有 4 帧深度。上层必须用
 *      中断 + 环形缓冲，且把 CAN_RX_PRINT_EVERY_N 调成 0 或 50 以上，
 *      否则必然溢出、丢帧。
 *   2. 手册把自动上传写在 7.5（扩展帧）一节，没说多圈绝对位置模式下
 *      是否生效，也没说上传帧是标准帧还是扩展帧。需要用抓包确认。
 */
int Motor_SetAutoUpload(u8 enable);

#endif /* MOTOR_DRIVER_H */
