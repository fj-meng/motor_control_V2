/*
 * 机致电机（HO / SO 中空系列）CAN 指令字典
 *
 * 依据：《中空系列电机控制使用说明 20250926》第七章「电机 Can 模式控制指南」
 *       （7.1 ~ 7.8）。CANFD 协议（第八章）不在本文件范围内。
 *
 * 本文件只定义协议常量、字节布局和换算宏，不含驱动实现，也不依赖
 * Xilinx 类型，任何 C 工程都能直接包含。
 *
 * ---------------------------------------------------------------------------
 * 一、CAN ID 约定（7.1 节）
 * ---------------------------------------------------------------------------
 * 电机（从机）自身的 ID 出厂默认 1，可配置 0~100；
 * 主机 ID 出厂默认 100，写在电机里，电机用它作为应答帧的仲裁 ID。
 *
 * 约定是「仲裁 ID = 目的地址」：
 *   主机 → 电机：仲裁 ID = 电机 ID（1）
 *   电机 → 主机：仲裁 ID = 主机 ID（100）
 *
 * 因此接收侧：ID=100 才是发给主机的应答帧；ID=1 是别的主机发给电机的。
 * 电机 ID 同时出现在应答帧的第 1 个数据字节，用于多机鉴别（7.4.4）。
 *
 * 通讯为一问一答：主机不发命令，从机静默。电机应答有 500us 延迟，
 * 最大通讯间隔 500us（2kHz）。
 *
 * 标准帧和扩展帧使用同一个 ID 下发（7.2 节）。
 */

#ifndef MOTOR_CAN_COMMANDS_H
#define MOTOR_CAN_COMMANDS_H

/* ---------------------------------------------------------------------------
 * 二、节点地址
 * ------------------------------------------------------------------------- */

/* 电机（从机）ID，出厂默认 1，可设 0~100，总线上不可重复。 */
#define MOTOR_ID_DEFAULT                    1U
/* 主机 ID，出厂默认 100，电机应答帧的仲裁 ID。 */
#define HOST_ID_DEFAULT                     100U

/* 电机 ID 的合法范围（7.1）。 */
#define MOTOR_ID_MIN                        0U
#define MOTOR_ID_MAX                        100U

/* ---------------------------------------------------------------------------
 * 三、帧格式
 * ------------------------------------------------------------------------- */

typedef enum {
	MOTOR_CAN_FRAME_STANDARD = 0U, /* 11 位标准帧，常规通讯 */
	MOTOR_CAN_FRAME_EXTENDED = 1U  /* 29 位扩展帧，模式切换与参数读写 */
} MotorCanFrameFormat;

/* 经典 CAN 一帧最多 8 字节。 */
#define MOTOR_CAN_DATA_MAX                  8U

/* ---------------------------------------------------------------------------
 * 四、标准帧特殊指令（7.2.1，序号 1~6）
 *
 * data[0..6] = 0xFF，data[7] = 命令码，共 8 字节。
 * ------------------------------------------------------------------------- */

#define MOTOR_STD_CMD_FILL                  0xFFU
#define MOTOR_STD_CMD_DLC                   8U

#define MOTOR_STD_CMD_START                 0xFCU /* 启动，按已设定的模式/位置/速度/力矩运行 */
#define MOTOR_STD_CMD_STOP                  0xFDU /* 停止，电机无输出力，处于自由状态 */
#define MOTOR_STD_CMD_TORQUE_MODE           0xF9U /* 切换为力矩控制模式 */
#define MOTOR_STD_CMD_SPEED_TORQUE_MODE     0xFAU /* 切换为速度力矩控制模式 */
#define MOTOR_STD_CMD_POS_SPEED_TORQUE      0xFBU /* 切换为位置速度力矩三闭环模式 */
#define MOTOR_STD_CMD_SET_MECH_ZERO         0xFEU /* 设当前位置为机械零位并清多圈计数；
                                                   * 必须在电机未使能时执行，掉电不保存 */

/*
 * 启动指令的前置条件（7.2.1 序号 1）：
 * 启动前必须先下发过一次位置/速度/力矩设定，否则电机的运动状态未定义。
 */

/* ---------------------------------------------------------------------------
 * 五、标准帧普通指令的 16 位数据布局（7.4.1）
 *
 * 8 个字节打包了位置、速度、位置环 Kp、位置环 Kd、力矩 共 5 项：
 *
 *   Byte1       位置命令 [15:8]
 *   Byte2       位置命令 [7:0]
 *   Byte3       速度命令 [11:4]
 *   Byte4       速度命令 [3:0]  |  位置增益 Kp [11:8]
 *   Byte5       位置增益 Kp [7:0]
 *   Byte6       位置阻尼 Kd [11:8]
 *   Byte7       位置阻尼 Kd [7:4]  |  力矩命令 [11:8]
 *   Byte8       力矩命令 [7:0]
 *
 * 校验例（7.4.2 ⑤）：位置 180°、速度 10rad/s、Kp=25、Kd=20、电流 2A
 *   -> C0 00 95 D0 19 01 4C 00
 *
 * 只有当前模式下有效的字段才会被电机采用：位置模式下位置/Kp/Kd 有效，
 * 速度模式下速度/力矩有效，力矩模式下只有力矩有效。
 * ------------------------------------------------------------------------- */

/* 各字段的零点偏移：位置 16 位、速度与力矩 12 位。 */
#define MOTOR_POS_ZERO                      0x8000U
#define MOTOR_VEL_ZERO                      0x0800U
#define MOTOR_CUR_ZERO                      0x0800U

#define MOTOR_POS_CODE_MAX                  0xFFFFU
#define MOTOR_VEL_CODE_MAX                  0x0FFFU
#define MOTOR_CUR_CODE_MAX                  0x0FFFU
#define MOTOR_KP_CODE_MAX                   0x0FFFU
#define MOTOR_KD_CODE_MAX                   0x0FFFU

/* 把各字段打包成 8 字节标准帧普通指令。 */
#define MOTOR_STD_PACK_B1_POS_HI(pos) \
	((unsigned char)(((pos) >> 8) & 0xFFU))
#define MOTOR_STD_PACK_B2_POS_LO(pos) \
	((unsigned char)((pos) & 0xFFU))
#define MOTOR_STD_PACK_B3_VEL_HI(vel) \
	((unsigned char)(((vel) >> 4) & 0xFFU))
#define MOTOR_STD_PACK_B4_VEL_KP(vel, kp) \
	((unsigned char)((((vel) & 0x0FU) << 4) | (((kp) >> 8) & 0x0FU)))
#define MOTOR_STD_PACK_B5_KP_LO(kp) \
	((unsigned char)((kp) & 0xFFU))
#define MOTOR_STD_PACK_B6_KD_HI(kd) \
	((unsigned char)(((kd) >> 4) & 0xFFU))
#define MOTOR_STD_PACK_B7_KD_CUR(kd, cur) \
	((unsigned char)((((kd) & 0x0FU) << 4) | (((cur) >> 8) & 0x0FU)))
#define MOTOR_STD_PACK_B8_CUR_LO(cur) \
	((unsigned char)((cur) & 0xFFU))

/* ---------------------------------------------------------------------------
 * 六、码值换算（7.4.2 / 7.4.5）
 *
 * 下发：code = (物理值 / 满量程) * 半量程 + 零点偏移
 * 解析：物理值 = (code - 零点偏移) / 半量程 * 满量程
 *
 * 位置：半量程 0x8000，满量程 = Pmax*360 度（Pmax 默认 1，即 ±360°）
 * 速度：半量程 0x0800，满量程 = 该型号速度常数（rad/s）
 * 力矩：半量程 0x0800，满量程 = 该型号电流常数（A）
 *
 * 注意 Kp / Kd 恒为正数，不做偏移，范围 0~4095。
 * ------------------------------------------------------------------------- */

/*
 * 各型号满量程常数（6.3 节查表，CAN 协议）。
 *
 * 型号        位置常数(Pmax)  速度常数(rad/s)  电流常数(A)
 * HO2206          1             1047.2            4
 * HO3507          1             58.639            4
 * HO4307          1             58.639            6
 * HO5515          1             58.639            6
 * HO6213          1             58.639            4
 * HO7213          1             58.639            4
 * HO8110          1             58.639            15
 * HO10010         1             58.639            32
 * HO10025         1             58.639            32
 * HO10045         1             58.639            32
 * HO13710         1             58.639            32
 * HO13750         1             58.639            32
 * SO4310          1             58.639            3
 * SO4315          1             58.639            3
 *
 * CANFD 协议下常数不同，不要混用。
 */

/* 本工程电机型号 HO10010 的常数。换型号时改这三行。 */
#define MOTOR_PMAX                          1.0f    /* 位置满量程，单位「圈」 */
#define MOTOR_SPEED_FULL_SCALE_RAD_S        58.639f /* 速度满量程 rad/s */
#define MOTOR_CURRENT_FULL_SCALE_A          32.0f   /* 电流满量程 A */

/* ---------------------------------------------------------------------------
 * 七、标准帧应答帧布局（7.4.4 / 7.6.3）
 *
 * 普通模式 6 字节，增量模式 8 字节（位置扩展到 32 位、20bit 单圈分辨率）。
 *
 * 普通模式：
 *   Byte1      电机模块 ID（用于多机鉴别）
 *   Byte2,3    当前位置 [15:8], [7:0]
 *   Byte4      当前速度 [11:4]
 *   Byte5      当前速度 [3:0] | 当前力矩 [11:8]
 *   Byte6      当前力矩 [7:0]
 *
 * 增量模式（在普通模式基础上追加 Byte7,8）：
 *   Byte7,8    当前位置 [31:24], [23:16]（32 位有符号，1048576 = 1 圈）
 * ------------------------------------------------------------------------- */

#define MOTOR_FB_DLC_NORMAL                 6U
#define MOTOR_FB_DLC_INCREMENTAL            8U

/* 单圈编码数：增量模式 20bit。 */
#define MOTOR_FB_COUNT_PER_TURN             1048576L

/* 应答帧解包：d0 为数据字节 0（即手册 Byte1）。 */
#define MOTOR_FB_ID(d0)                     ((unsigned int)(d0))
#define MOTOR_FB_POS(d1, d2) \
	((unsigned int)((((unsigned int)(d1)) << 8) | (unsigned int)(d2)))
#define MOTOR_FB_VEL(d3, d4) \
	((unsigned int)((((unsigned int)(d3)) << 4) | \
	(((unsigned int)(d4) >> 4) & 0x0FU)))
#define MOTOR_FB_CUR(d4, d5) \
	((unsigned int)(((((unsigned int)(d4)) & 0x0FU) << 8) | \
	(unsigned int)(d5)))

/* 增量模式 32 位位置：[d6][d7][d1][d2]，有符号。 */
#define MOTOR_FB_POS32(d1, d2, d6, d7) \
	((long)((((unsigned long)(d6)) << 24) | (((unsigned long)(d7)) << 16) | \
	(((unsigned long)(d1)) << 8) | ((unsigned long)(d2))))

/* ---------------------------------------------------------------------------
 * 八、扩展帧 Cmd 分类（7.2.2 ①）
 *
 * 扩展帧数据区：data[0] = Cmd，data[1] = Index（可选），data[2..7] = 数据。
 * ------------------------------------------------------------------------- */

#define MOTOR_EXT_CMD_SET_MODE              0x02U /* 切换模式，Index 指定目标模式 */
#define MOTOR_EXT_CMD_SET_PARAMETER         0x00U /* 设置参数，Index 指定参数类型 */
#define MOTOR_EXT_CMD_READ_PARAMETER        0x01U /* 读取参数，Index 指定参数类型 */
#define MOTOR_EXT_CMD_MULTI_POS             100U  /* 0x64 多圈位置控制 */
#define MOTOR_EXT_CMD_MULTI_SPEED           101U  /* 0x65 多圈速度控制 */
#define MOTOR_EXT_CMD_MULTI_CURRENT         102U  /* 0x66 多圈电流控制 */

#define MOTOR_EXT_CMD_DLC                   8U

/* ---------------------------------------------------------------------------
 * 九、扩展帧切换模式（7.2.2 ②，Cmd=0x02）
 *
 * 数据区：02 | Index | 00 00 00 00 00 00
 * 电机应答为标准帧，格式见第七节。
 * ------------------------------------------------------------------------- */

#define MOTOR_EXT_MODE_SINGLE_TURN_ABS      0x00U /* 02 00：单圈绝对位置模式（上电默认） */
#define MOTOR_EXT_MODE_MULTI_TURN_ABS       0x01U /* 02 01：多圈绝对位置模式 */
#define MOTOR_EXT_MODE_INCREMENTAL_POS      0x02U /* 02 02：增量位置模式 */

/*
 * 兼容旧协议的切换方式：整帧 8 个字节全 0，同样进入增量位置模式。
 * 即 00 00 00 00 00 00 00 00。
 */
#define MOTOR_EXT_MODE_INCREMENTAL_COMPAT   0x00U

/*
 * 切换模式的注意事项（7.3.1 / 7.5.2）：
 *   1. 切换前先停止电机，并发送清零或清多圈指令，否则底层寄存器保留旧
 *      位置数据，切换后电机会反转一大段距离。
 *   2. 增量模式掉电不保持，每次上电需要重新切换一次。
 *   3. 切换为增量模式时电机需处于静置状态。
 *   4. 应答帧长度由 6 字节变 8 字节，可用来确认切换成功。
 */

/* ---------------------------------------------------------------------------
 * 十、扩展帧设置参数（7.2.2 ③，Cmd=0x00）
 *
 * 数据区：00 | Index | XX XX XX XX 00 00
 * Kp/Ki 与满量程系数为 IEEE754 单精度浮点，高字节在前（3F 80 00 00 = 1.0）。
 * 这类参数掉电不保存，每次上电需重新设置。
 * ------------------------------------------------------------------------- */

#define MOTOR_EXT_PARAM_SPEED_KP            0x01U /* 速度环 Kp */
#define MOTOR_EXT_PARAM_SPEED_KI            0x02U /* 速度环 Ki */
#define MOTOR_EXT_PARAM_CURRENT_KP          0x03U /* 电流环 Kp */
#define MOTOR_EXT_PARAM_CURRENT_KI          0x04U /* 电流环 Ki */
#define MOTOR_EXT_PARAM_AUTO_UPLOAD         0x05U /* 自动上传开关 */
#define MOTOR_EXT_PARAM_SPEED_SCALE         0x06U /* 速度满量程系数 */
#define MOTOR_EXT_PARAM_CLEAR_MULTI_TURN    0x08U /* 清除内部多圈计数 */

/* 自动上传：00 05 XX 00 00 00 00 00 */
#define MOTOR_AUTO_UPLOAD_DISABLE           0x00U
#define MOTOR_AUTO_UPLOAD_ENABLE            0x01U
#define MOTOR_AUTO_UPLOAD_PERIOD_MS         1U    /* 固定 1ms 周期 */
/*
 * 自动上传开启期间电机仍会应答主机指令，但 1ms 周期 + 1Mbps 已接近
 * 总线负载上限，主机侧必须用中断 + 环形缓冲，不能在中断里打印。
 */

/* ---------------------------------------------------------------------------
 * 十一、扩展帧读取参数（7.2.2 ④，Cmd=0x01）
 *
 * 请求为扩展帧，数据区：01 | Index | 00 00 00 00 00 00
 * 应答为标准帧，ID=主机 ID：
 *
 *   data[0] = 电机 ID
 *   data[1] = 0x01（Cmd 回显）
 *   data[2] = Index 回显
 *   data[3..] = 数据
 *   data[7] = 0x00
 *
 * 注意发送和应答的起始字节不一样：请求的 data[0] 是 Cmd，应答的 data[0]
 * 是电机 ID，整体后移一格。按请求的偏移去解析应答会全部读错。
 *
 *   读温度  请求 01 00 ...  应答 XX 01 00 YY 00 00 00 00   YY=int8
 *   读 Kp/Ki 请求 01 0n ... 应答 XX 01 0n YY YY YY YY 00   YY=float 大端
 *
 * 手册没有「读取实时位置/速度/电流」的参数读取指令。这三项只能从下面
 * 三个途径拿到：
 *   1. 任何一条普通指令的应答帧（第七节，6 或 8 字节）；
 *   2. 打开自动上传（00 05 01），电机每 1ms 主动发一帧；
 *   3. 多圈模式下发 Cmd=100/101/102 后的扩展帧应答。
 * ------------------------------------------------------------------------- */

#define MOTOR_EXT_READ_TEMPERATURE          0x00U /* 实时温度，int8，有符号 */
#define MOTOR_EXT_READ_SPEED_KP             0x01U /* 当前速度环 Kp，float 大端 */
#define MOTOR_EXT_READ_SPEED_KI             0x02U /* 当前速度环 Ki，float 大端 */
#define MOTOR_EXT_READ_CURRENT_KP           0x03U /* 当前电流环 Kp，float 大端 */
#define MOTOR_EXT_READ_CURRENT_KI           0x04U /* 当前电流环 Ki，float 大端 */

/* 读取应答帧的字段偏移（下标基于数据区第 0 字节）。 */
#define MOTOR_READ_FB_ID_IDX                0U /* 电机 ID */
#define MOTOR_READ_FB_CMD_IDX               1U /* 0x01 */
#define MOTOR_READ_FB_INDEX_IDX             2U /* 请求时的 Index 回显 */
#define MOTOR_READ_FB_DATA_IDX              3U /* 温度 1 字节 / 参数 4 字节 */

/* ---------------------------------------------------------------------------
 * 十二、多圈绝对位置模式（7.2.2 ⑤ / 7.7，Cmd=100/101/102）
 *
 * 注意 Cmd 本身占 data[0]，有效载荷从 data[1] 开始。
 *
 * Cmd=100（0x64）位置控制，共 8 字节：
 *   data[0] = 0x64
 *   data[1..5] = 5 字节位置命令，40 位有符号补码，1048576 = 1 圈
 *   data[6] = YY 模式：2=位置，1=速度，0=电流
 *   data[7] = ZZ 使能：1=使能，0=失能
 *
 * Cmd=101（0x65）速度控制，共 8 字节：
 *   data[0] = 0x65
 *   data[1..4] = 4 字节速度命令，int32，电频率表示
 *   data[5] = 00（保留）
 *   data[6] = YY 模式
 *   data[7] = ZZ 使能
 *
 * Cmd=102（0x66）电流控制，共 8 字节：
 *   data[0] = 0x66
 *   data[1..2] = 2 字节电流命令，int16
 *   data[3] = 位置环 Kp，0~255
 *   data[4] = 位置环 Kd，0~255
 *   data[5] = Zero 命令：1=设当前位置为 0 点，2=清除多圈计数
 *   data[6] = YY 模式
 *   data[7] = ZZ 使能
 * ------------------------------------------------------------------------- */

#define MOTOR_MULTI_POS_BYTES               5U
#define MOTOR_MULTI_SPEED_BYTES             4U
#define MOTOR_MULTI_CURRENT_BYTES           2U

/* YY 模式字段。 */
#define MOTOR_MULTI_MODE_CURRENT            0U
#define MOTOR_MULTI_MODE_SPEED              1U
#define MOTOR_MULTI_MODE_POSITION           2U

/* ZZ 使能字段。 */
#define MOTOR_MULTI_DISABLE                 0U
#define MOTOR_MULTI_ENABLE                  1U

/* Zero 字段（Cmd=102 的 data[5]）。 */
#define MOTOR_MULTI_ZERO_NONE               0U /* 不清零 */
#define MOTOR_MULTI_ZERO_SET_CURRENT_POS    1U /* 设置当前位置为 0 点 */
#define MOTOR_MULTI_ZERO_CLEAR_COUNT        2U /* 清除多圈计数值 */

/* 位置环 Kp / Kd 取值范围（Cmd=102）。 */
#define MOTOR_MULTI_POS_KP_MIN              0U
#define MOTOR_MULTI_POS_KP_MAX              255U
#define MOTOR_MULTI_POS_KD_MIN              0U
#define MOTOR_MULTI_POS_KD_MAX              255U

/* 多圈模式的速度与电流码值换算（7.7.2）。 */
#define MOTOR_MULTI_VEL_FULL_SCALE_HZ       1000.0f /* 速度码满量程 8388608 对应 1000Hz */
#define MOTOR_MULTI_VEL_CODE_MAX            8388608L
#define MOTOR_MULTI_CUR_FULL_SCALE_A        100.0f  /* 电流码满量程 32768 对应 100A */
#define MOTOR_MULTI_CUR_CODE_MAX            32768L

/*
 * 电机极对数，用于 RPM <-> 电频率换算（7.7.2）。
 *
 * 手册只直接说明了「14 是 3507 电机极对数」。CANFD 常数表（6.3）里
 * HO3507 的速度常数为 14、HO7213 与 HO10010 为 21，与极对数吻合，
 * 因此 HO10010 取 21。换型号时按同一张表改。
 *
 * 验证方法：设 10RPM 使能后实测转速，若实际转速是设定值的 21/14 倍
 * 或 14/21 倍，说明这个值取错了。
 */
#define MOTOR_POLE_PAIRS                    21U

/*
 * 进入多圈绝对位置模式（7.7.2 ①）：
 * 必须依次发送 Cmd=100、101、102 各一次，且三条指令的 YY 模式相同、
 * ZZ 使能都为 0，电机才会进入该模式。任一条不满足则进入失败。
 *
 * 【实测补充】只发这三条，电机不会切模式（应答一直是 6 字节标准帧）。
 * 必须先发一条扩展帧 02 01 00 00 00 00 00 00 显式切模式（7.2.2 ② 序号 2），
 * 7.8.2 的控制实例也是这么写的。02 01 本身的应答是普通标准帧，
 * 不要把它当成切换失败。
 *
 * 进入后只需再发其中任意一条且 ZZ=1 即可启动，电机使用进入时设定的
 * 位置/速度/电流/Kp/Kd 中未被新指令覆盖的部分。
 *
 * 该模式下的应答也是扩展帧（7.7.3），一次只返回一项：
 *   Cmd=100 回 5 字节实时位置，Cmd=101 回 4 字节实时速度，
 *   Cmd=102 回 2 字节实时电流。响应末尾的 ZZ 是从机 ID，YY 是温度。
 */

/* ---------------------------------------------------------------------------
 * 十三、协议时序约束（7.1）
 * ------------------------------------------------------------------------- */

/* 电机从收到指令到应答的延迟。 */
#define MOTOR_RESPONSE_DELAY_US             500U
/* 主机两条指令之间的最小间隔，对应最大 2kHz 通讯频率。 */
#define MOTOR_MIN_CMD_INTERVAL_US           500U

#endif /* MOTOR_CAN_COMMANDS_H */
