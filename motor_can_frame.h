#ifndef MOTOR_CAN_FRAME_H
#define MOTOR_CAN_FRAME_H

/*
 * 电机 CAN 帧的 ARM 侧软件表示。
 *
 * 注意：这是供程序内部使用的 13 字节描述格式，不是 CAN 总线上的
 * 单帧负载。经典 CAN 总线每帧仍最多传输 8 字节数据。
 * PPS 时间戳不放在此结构或 13 字节描述帧中。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_CAN_DATA_MAX_LEN       8U
#define MOTOR_CAN_DESC_HEADER_LEN    5U
#define MOTOR_CAN_DESC_FRAME_LEN     13U

/* 图示字节1的帧信息位 */
#define MOTOR_CAN_DESC_FLAG_EXTENDED 0x80U  /* FF=1：扩展帧 */
#define MOTOR_CAN_DESC_FLAG_RTR      0x40U  /* RTR=1：远程帧 */
#define MOTOR_CAN_DESC_DLC_MASK      0x0FU

#define MOTOR_CAN_STANDARD_ID_MAX   0x7FFU
#define MOTOR_CAN_EXTENDED_ID_MAX   0x1FFFFFFFU

typedef struct {
    uint8_t is_extended; /* 0：标准帧；非0：扩展帧 */
    uint8_t is_remote;   /* 0：数据帧；非0：远程帧 */
    uint8_t dlc;         /* 经典 CAN 数据长度，0～8 */
    uint32_t can_id;     /* 已解码的 CAN 仲裁 ID，不是 XCANPS 原始寄存器值 */
    uint8_t data[MOTOR_CAN_DATA_MAX_LEN];
} MotorCanFrame;

/*
 * 将已经从 XCANPS 接收缓冲区解码出的字段，打包成图示 13 字节格式。
 * 返回 0 表示成功；参数不合法返回负值。
 * out_frame 的布局：
 *   [0]      FF/RTR/保留位/DLC
 *   [1..4]   29 位 CAN ID，按高字节在前放置
 *   [5..12]  8 字节数据；DLC 以外部分填 0
 */
int MotorCanFrame_PackDescription(const MotorCanFrame *frame,
                                  uint8_t out_frame[MOTOR_CAN_DESC_FRAME_LEN]);

/* 将 13 字节描述格式解回字段；返回 0 成功，负值表示格式或参数错误。 */
int MotorCanFrame_UnpackDescription(
    const uint8_t in_frame[MOTOR_CAN_DESC_FRAME_LEN], MotorCanFrame *frame);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CAN_FRAME_H */
