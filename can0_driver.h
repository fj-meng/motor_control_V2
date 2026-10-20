/*
 * PS CAN0 驱动对外接口
 *
 * 函数定义全部放在 can0_driver.c；main.c 只调用这里声明的接口。
 */

#ifndef CAN0_DRIVER_H
#define CAN0_DRIVER_H

/* 该头文件独立使用时也能看到 SEND_SAFE_PROBE 配置。 */
#include "can0_config.h"

/* 初始化 PS CAN0、配置位时序并进入正常模式。 */
int Can0_Init(void);

/*
 * 根据 SEND_SAFE_PROBE 配置发送一次测试帧。
 * 当前默认关闭，只有显式打开发送测试时才在实现中提供该接口。
 */
#if SEND_SAFE_PROBE
int Can0_SendSafeProbe(void);
#endif

/* 轮询接收 FIFO，打印收到的 CAN 帧和硬件接收时间戳。该函数不会返回。 */
void Can0_RunReceiveLoop(void);

#endif /* CAN0_DRIVER_H */
