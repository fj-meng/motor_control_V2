/*
 * 机致电机 CAN 指令字典
 *
 * 依据电机控制说明书第 7.2 节整理，适用于经典 CAN 2.0：
 *
 *   1. 标准帧：用于常规通信和 8 字节特殊指令；
 *   2. 扩展帧：用于模式切换、参数读写和多圈绝对位置控制。
 *
 * 注意：
 *   - 本文件定义的是 CAN 负载和协议字段，不是 USB-CAN 串口封装格式。
 *   - 标准帧/扩展帧的仲裁 ID 应由实际电机配置确定。当前工程抓包观察到
 *     控制帧 ID=8、反馈帧 ID=50，但这里不把它们写死为协议常量。
 *   - 下面的 XX、YY、ZZ 等字段必须按电机手册和当前运行模式填写；没有
 *     明确给出的编码不在本字典中擅自推断。
 */

#ifndef MOTOR_CAN_COMMANDS_H
#define MOTOR_CAN_COMMANDS_H

/* 电机协议使用的 CAN 帧格式。 */
typedef enum {
	MOTOR_CAN_FRAME_STANDARD = 0U, /* 11 位标准帧 */
	MOTOR_CAN_FRAME_EXTENDED = 1U  /* 29 位扩展帧 */
} MotorCanFrameFormat;

/*
 * 当前工程抓包得到的仲裁 ID，仅作为默认工程配置，不能替代电机设置。
 * 反馈数据第 1 字节中的 1 是模块 ID，与 CAN 仲裁 ID 不是同一个字段。
 */
#define MOTOR_CAN_DEFAULT_COMMAND_ID       8U
#define MOTOR_CAN_DEFAULT_FEEDBACK_ID      50U
#define MOTOR_CAN_DEFAULT_MODULE_ID         1U

/* 标准帧特殊指令：8 字节数据，前 7 字节为 FF，最后 1 字节为命令码。 */
#define MOTOR_STD_CMD_START                0xFCU
#define MOTOR_STD_CMD_STOP                 0xFDU
#define MOTOR_STD_CMD_TORQUE_MODE          0xF9U
#define MOTOR_STD_CMD_SPEED_TORQUE_MODE    0xFAU
#define MOTOR_STD_CMD_POS_SPEED_TORQUE     0xFBU
#define MOTOR_STD_CMD_SET_MECH_ZERO        0xFEU

/* 标准帧特殊指令的固定长度和填充值。 */
#define MOTOR_STD_CMD_DLC                  8U
#define MOTOR_STD_CMD_FILL                 0xFFU

/*
 * 扩展帧的第 1 个数据字节 Cmd。
 * 扩展帧负载通常为：Cmd + 可选 Index + Data[0..5]。
 */
#define MOTOR_EXT_CMD_SET_MODE             0x02U
#define MOTOR_EXT_CMD_SET_PARAMETER        0x00U
#define MOTOR_EXT_CMD_READ_PARAMETER       0x01U

/* 扩展帧模式切换的 Index。Cmd=0x02 时使用。 */
#define MOTOR_EXT_MODE_SINGLE_TURN_ABS     0x00U
#define MOTOR_EXT_MODE_MULTI_TURN_ABS      0x01U
#define MOTOR_EXT_MODE_INCREMENTAL_POS     0x02U
#define MOTOR_EXT_MODE_INCREMENTAL_COMPAT  0x00U

/* 扩展帧参数索引：Cmd=0x00 设置，Cmd=0x01 读取。 */
#define MOTOR_EXT_PARAM_SPEED_KP           0x01U
#define MOTOR_EXT_PARAM_SPEED_KI           0x02U
#define MOTOR_EXT_PARAM_CURRENT_KP         0x03U
#define MOTOR_EXT_PARAM_CURRENT_KI         0x04U
#define MOTOR_EXT_PARAM_AUTO_UPLOAD        0x05U
#define MOTOR_EXT_PARAM_SPEED_SCALE        0x06U
#define MOTOR_EXT_PARAM_CLEAR_MULTI_TURN   0x08U

/* 扩展帧参数读写和模式切换均使用 8 字节 CAN 数据。 */
#define MOTOR_EXT_CMD_DLC                  8U

/* 自动上传参数：Cmd=0x00，Index=0x05，数据第 1 字节取以下值。 */
#define MOTOR_AUTO_UPLOAD_DISABLE          0x00U
#define MOTOR_AUTO_UPLOAD_ENABLE           0x01U
#define MOTOR_AUTO_UPLOAD_PERIOD_MS        1U

/*
 * 多圈绝对位置模式的扩展帧 Cmd。
 *
 * Cmd=100：位置控制
 *   Data[0..4] = 5 字节位置命令 XX
 *   Data[5]    = 模式 YY：2=位置，1=速度，0=电流
 *   Data[6]    = 使能 ZZ：1=使能，0=失能
 *   Data[7]    = 按手册保留/填充
 *
 * Cmd=101：速度控制
 *   Data[0..3] = 4 字节速度命令 XX
 *   Data[4]    = 0（手册示例保留字节）
 *   Data[5]    = 模式 YY
 *   Data[6]    = 使能 ZZ
 *   Data[7]    = 按手册保留/填充
 *
 * Cmd=102：电流控制
 *   Data[0..1] = 2 字节电流命令 XX
 *   Data[2]    = 位置环 Kp
 *   Data[3]    = 位置环 Kd
 *   Data[4]    = Zero：0=清除多圈计数，1=设置当前位置
 *   Data[5]    = 模式 YY（具体编码按手册/当前模式填写）
 *   Data[6]    = 使能 ZZ：1=使能，0=失能
 *   Data[7]    = 按手册保留/填充
 */
#define MOTOR_EXT_CMD_MULTI_POS            100U
#define MOTOR_EXT_CMD_MULTI_SPEED          101U
#define MOTOR_EXT_CMD_MULTI_CURRENT        102U

/* 多圈控制命令中的模式和使能字段。 */
#define MOTOR_MULTI_MODE_CURRENT            0U
#define MOTOR_MULTI_MODE_SPEED              1U
#define MOTOR_MULTI_MODE_POSITION           2U
#define MOTOR_MULTI_DISABLE                 0U
#define MOTOR_MULTI_ENABLE                  1U

/* Cmd=102 的 Zero 字段。 */
#define MOTOR_MULTI_ZERO_CLEAR_COUNT        0U
#define MOTOR_MULTI_ZERO_SET_CURRENT_POS    1U

/* 扩展帧数据中常见的保留字节值。 */
#define MOTOR_EXT_RESERVED                 0x00U

/*
 * 标准帧特殊指令生成规则：
 *   data[0..6] = 0xFF
 *   data[7]    = 上述 MOTOR_STD_CMD_* 命令码
 *
 * 该宏只表达协议规则，实际数组填充放在驱动函数中，避免头文件依赖 u8。
 */

/*
 * 扩展帧的基本组帧规则：
 *   data[0] = Cmd
 *   data[1] = Index（有 Index 的命令使用；无 Index 时按手册填充）
 *   data[2..7] = 参数数据
 *
 * 多圈 Cmd=100/101/102 的 Cmd 和字段布局见上方注释；它们不是 Cmd+Index
 * 的参数读写格式，不能把多圈命令误套用成普通参数设置格式。
 */

#endif /* MOTOR_CAN_COMMANDS_H */
