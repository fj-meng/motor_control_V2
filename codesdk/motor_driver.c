/*
 * 机致电机控制模块实现
 *
 * 只实现多圈绝对位置模式下的速度控制。所有帧都是扩展帧，
 * 仲裁 ID 填电机 ID（手册 7.7.1：该模式收发都用扩展帧）。
 */

#include "motor_driver.h"
#include "motor_can_commands.h"
#include "can0_driver.h"
#include "can0_config.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "sleep.h"

/*
 * 两条指令之间的间隔。
 * 手册 7.1：电机从收到指令到应答需要 500us，最小通讯间隔也是 500us。
 * 这里留一倍余量，避免下一条指令和上一条的应答撞在一起。
 */
#define MOTOR_CMD_GAP_US    1000U

/* 上一次设定的速度码，Motor_StartSpeed() 和改速时复用。 */
static s32 MotorSpeedCode;

/* 发送一帧 8 字节扩展帧给电机。 */
static int Motor_SendExtended(const u8 *data)
{
	return Can0_SendFrame(MOTOR_CAN_ID, 1U, 0U, data, MOTOR_EXT_CMD_DLC);
}

/* 发送标准帧停止指令（0xFF×7 + 0xFD）。 */
static int Motor_SendStopStandard(void)
{
	u8 data[8];
	u8 i;

	for (i = 0U; i < 7U; ++i) {
		data[i] = MOTOR_STD_CMD_FILL;
	}
	data[7] = MOTOR_STD_CMD_STOP;

	return Can0_SendFrame(MOTOR_CAN_ID, 0U, 0U, data, MOTOR_STD_CMD_DLC);
}

/*
 * RPM 换算成多圈模式的速度码（7.7.2）。
 *
 *   Freq(Hz) = RPM * 极对数 / 60
 *   code     = Freq / 1000 * 8388608
 *
 * 例：HO3507（14 极对）10RPM -> 2.3333Hz -> 19573 = 0x00004C75
 */
static s32 Motor_RpmToSpeedCode(float rpm)
{
	float freq_hz;
	float code;

	freq_hz = rpm * (float)MOTOR_POLE_PAIRS / 60.0f;
	code = freq_hz / MOTOR_MULTI_VEL_FULL_SCALE_HZ *
			(float)MOTOR_MULTI_VEL_CODE_MAX;

	if (code > 2147483647.0f) {
		code = 2147483647.0f;
	} else if (code < -2147483648.0f) {
		code = -2147483648.0f;
	}

	return (s32)code;
}

/* 电流（A）换算成多圈模式的电流码：code = A / 100 * 32768。 */
static s16 Motor_AmpToCurrentCode(float amp)
{
	float code;

	code = amp / MOTOR_MULTI_CUR_FULL_SCALE_A *
			(float)MOTOR_MULTI_CUR_CODE_MAX;

	if (code > 32767.0f) {
		code = 32767.0f;
	} else if (code < -32768.0f) {
		code = -32768.0f;
	}

	return (s16)code;
}

/*
 * Cmd=100（0x64）位置命令帧。
 *   data[1..5] 40 位位置，data[6] 模式，data[7] 使能
 * 速度模式下位置命令无效，固定填 0。
 */
static void Motor_FillPosFrame(u8 *d, u8 enable)
{
	d[0] = (u8)MOTOR_EXT_CMD_MULTI_POS;
	d[1] = 0U;
	d[2] = 0U;
	d[3] = 0U;
	d[4] = 0U;
	d[5] = 0U;
	d[6] = (u8)MOTOR_MULTI_MODE_SPEED;
	d[7] = enable;
}

/*
 * Cmd=101（0x65）速度命令帧。
 *   data[1..4] int32 速度，data[5] 保留 0，data[6] 模式，data[7] 使能
 */
static void Motor_FillSpeedFrame(u8 *d, s32 speed_code, u8 enable)
{
	u32 v = (u32)speed_code;

	d[0] = (u8)MOTOR_EXT_CMD_MULTI_SPEED;
	d[1] = (u8)(v >> 24);
	d[2] = (u8)(v >> 16);
	d[3] = (u8)(v >> 8);
	d[4] = (u8)v;
	d[5] = 0U;
	d[6] = (u8)MOTOR_MULTI_MODE_SPEED;
	d[7] = enable;
}

/*
 * Cmd=102（0x66）电流命令帧。
 *   data[1..2] int16 电流，data[3] Kp，data[4] Kd，
 *   data[5] Zero，data[6] 模式，data[7] 使能
 */
static void Motor_FillCurrentFrame(u8 *d, s16 cur_code, u8 kp, u8 kd,
		u8 enable)
{
	u16 c = (u16)cur_code;

	d[0] = (u8)MOTOR_EXT_CMD_MULTI_CURRENT;
	d[1] = (u8)(c >> 8);
	d[2] = (u8)c;
	d[3] = kp;
	d[4] = kd;
	d[5] = (u8)MOTOR_MULTI_ZERO_NONE;
	d[6] = (u8)MOTOR_MULTI_MODE_SPEED;
	d[7] = enable;
}

int Motor_InitSpeedMode(float speed_rpm, float current_a)
{
	u8 frame[8];
	s32 speed_code;
	s16 cur_code;
	int status;

	speed_code = Motor_RpmToSpeedCode(speed_rpm);
	cur_code = Motor_AmpToCurrentCode(current_a);

	/* 先停一次，确保电机处于自由状态再切换模式（7.3.1 建议）。 */
	(void)Motor_SendStopStandard();
	usleep(MOTOR_CMD_GAP_US);

	/*
	 * 进入多圈绝对位置模式：Cmd=100、101、102 必须依次各发一次，
	 * 三条的模式字段相同、使能位都为 0，否则电机不会进入该模式。
	 * 任意一条不满足，前功尽弃。
	 */
	Motor_FillPosFrame(frame, (u8)MOTOR_MULTI_DISABLE);
	status = Motor_SendExtended(frame);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: Cmd100 send failed: %d\r\n", status);
		return status;
	}
	usleep(MOTOR_CMD_GAP_US);

	Motor_FillSpeedFrame(frame, speed_code, (u8)MOTOR_MULTI_DISABLE);
	status = Motor_SendExtended(frame);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: Cmd101 send failed: %d\r\n", status);
		return status;
	}
	usleep(MOTOR_CMD_GAP_US);

	Motor_FillCurrentFrame(frame, cur_code, (u8)MOTOR_TARGET_POS_KP,
			(u8)MOTOR_TARGET_POS_KD, (u8)MOTOR_MULTI_DISABLE);
	status = Motor_SendExtended(frame);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: Cmd102 send failed: %d\r\n", status);
		return status;
	}
	usleep(MOTOR_CMD_GAP_US);

	MotorSpeedCode = speed_code;

	xil_printf("Motor: multi-turn abs SPEED mode set, "
			"vel_code=%d cur_code=%d, ENABLE=0\r\n",
			(int)speed_code, (int)cur_code);

	return XST_SUCCESS;
}

int Motor_StartSpeed(void)
{
	u8 frame[8];
	int status;

	Motor_FillSpeedFrame(frame, MotorSpeedCode, (u8)MOTOR_MULTI_ENABLE);
	status = Motor_SendExtended(frame);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: start failed: %d\r\n", status);
		return status;
	}

	xil_printf("Motor: enabled, vel_code=%d\r\n", (int)MotorSpeedCode);
	return XST_SUCCESS;
}

int Motor_StopSpeed(void)
{
	u8 frame[8];
	int status;

	Motor_FillSpeedFrame(frame, MotorSpeedCode, (u8)MOTOR_MULTI_DISABLE);
	status = Motor_SendExtended(frame);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: stop failed: %d\r\n", status);
		return status;
	}

	xil_printf("Motor: disabled\r\n");
	return XST_SUCCESS;
}

int Motor_SetSpeedRpm(float speed_rpm)
{
	u8 frame[8];
	int status;

	MotorSpeedCode = Motor_RpmToSpeedCode(speed_rpm);

	/* 保持失能，只更新速度设定。 */
	Motor_FillSpeedFrame(frame, MotorSpeedCode, (u8)MOTOR_MULTI_DISABLE);
	status = Motor_SendExtended(frame);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: set speed failed: %d\r\n", status);
		return status;
	}

	xil_printf("Motor: speed updated, vel_code=%d (still disabled)\r\n",
			(int)MotorSpeedCode);
	return XST_SUCCESS;
}

/*
 * 打开 / 关闭主动上传。
 *
 * 帧格式（手册 7.2.2 ③ 序号 9）：
 *   00 05 01 00 00 00 00 00   打开，1ms 周期
 *   00 05 00 00 00 00 00 00   关闭
 * 即 Cmd=0x00（设置参数）、Index=0x05（自动上传）、data[2] 为开关。
 */
int Motor_SetAutoUpload(u8 enable)
{
	u8 frame[8];
	u8 i;
	int status;

	frame[0] = (u8)MOTOR_EXT_CMD_SET_PARAMETER;
	frame[1] = (u8)MOTOR_EXT_PARAM_AUTO_UPLOAD;
	frame[2] = (enable != 0U) ? (u8)MOTOR_AUTO_UPLOAD_ENABLE
			: (u8)MOTOR_AUTO_UPLOAD_DISABLE;
	for (i = 3U; i < MOTOR_EXT_CMD_DLC; ++i) {
		frame[i] = 0U;
	}

	status = Motor_SendExtended(frame);
	if (status != XST_SUCCESS) {
		xil_printf("Motor: auto upload %s failed: %d\r\n",
				(enable != 0U) ? "on" : "off", status);
		return status;
	}

	if (enable != 0U) {
		xil_printf("Motor: auto upload ON, %d ms period. "
				"Set CAN_RX_PRINT_EVERY_N to 0 to avoid RX overflow.\r\n",
				MOTOR_AUTO_UPLOAD_PERIOD_MS);
	} else {
		xil_printf("Motor: auto upload OFF\r\n");
	}

	return XST_SUCCESS;
}
