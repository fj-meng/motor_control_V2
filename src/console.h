/*
 * 串口命令行对外接口
 *
 * 支持的命令（输入后回车）：
 *   writemotordata   开始把 CAN 帧写进 SD 卡（默认不写，帧收到就丢）
 *   motormod2init    切单圈绝对位置模式的速度力矩控制，设定转速，但不启动
 *   motoron          启动电机，按 motormod2init 设定的转速转
 *   motoroff         停止电机（自由状态，参数仍保留，可直接再 motoron）
 *   motorupload      打开电机自动上传（掉电不保持，重新上电后要补发）
 *   motoruploadoff   关闭电机自动上传
 *   help             列出所有命令
 */

#ifndef CONSOLE_H
#define CONSOLE_H

/* 确认串口收发使能，清空行缓冲。上电时调用一次。 */
void Console_Init(void);

/*
 * 轮询串口接收 FIFO，收满一行就执行命令。
 *
 * 主循环里反复调用。每次最多读固定个数的字符就返回，不会长时间阻塞。
 */
void Console_Poll(void);

#endif /* CONSOLE_H */
