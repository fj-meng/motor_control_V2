# 电机 CAN 帧软件格式使用说明

`motor_can_frame.c/.h` 实现截图中的 13 字节描述格式，供 ARM 程序内部存储、打印或交给上层协议处理。它不改变 CAN 总线报文：经典 CAN 总线上的数据仍最多 8 字节。

## 13 字节布局

| 索引 | 内容 |
|---:|---|
| 0 | bit7：FF（1 扩展帧，0 标准帧）；bit6：RTR；bit5..4 保留；bit3..0：DLC |
| 1..4 | 29 位 ID 字段，高字节在前；字节4低 3 位保留为 0。扩展帧使用完整 29 位 ID；标准帧 11 位 ID 放在 ID28..ID18（即左移 18 位） |
| 5..12 | 8 字节 CAN 数据区 |

时间戳不属于该描述帧。若需要 PPS 时间戳，请在独立的接收记录结构中保存，例如 `{捕获时间, 13字节描述帧}`；捕获时间应来自 PPS 计时逻辑，而非 CAN 的 DLC/ID/数据字段。

## 在 SDK 工程中使用

把 `motor_can_frame.c` 和 `motor_can_frame.h` 添加到 SDK 工程。CAN 驱动取到接收缓冲区后，先用 SDK 2017.4 的 XCANPS 定义解析出 **数值 CAN ID、帧格式、RTR、DLC**，并按数据字节顺序准备 `data[0..7]`。然后调用：

```c
MotorCanFrame rx;
uint8_t software_frame[MOTOR_CAN_DESC_FRAME_LEN];
int status;

/* 以下字段应由 XCANPS 接收帧解析得到。can_id 必须是解码后的 ID，不能直接填 frame[0] raw_id。 */
rx.is_extended = decoded_is_extended;
rx.is_remote = decoded_is_remote;
rx.dlc = decoded_dlc;
rx.can_id = decoded_can_id;
for (i = 0U; i < MOTOR_CAN_DATA_MAX_LEN; ++i) {
    rx.data[i] = (i < rx.dlc) ? can_data_bytes[i] : 0U;
}

status = MotorCanFrame_PackDescription(&rx, software_frame);
if (status == 0) {
    /* software_frame[0..12] 是 ARM 内部的 13 字节描述格式。 */
}
```

`decoded_is_extended` 等名称只是示例变量。需要用工程所用的 `xcanps.h` 宏和当前 SDK 驱动的帧格式完成解码；本模块不猜测 XCANPS 原始寄存器位域，也不从 `frame[1]` 提取时间戳。

## 编码示例

扩展数据帧 ID `0x0648`、DLC=8，数据为 `65 00 00 00 3A 00 1A 01` 时，描述帧为：

```text
88 00 00 32 40 65 00 00 00 3A 00 1A 01
```

其中 `0x88` 表示扩展帧（bit7=1）、数据帧（bit6=0）、DLC=8。ID 和数据按截图定义编码。作为标准帧示例，ID=`0x0648` 时 13 字节描述帧开头应为 `08 32 40 00 00`，即该标准 ID 放在 29 位 ID 字段的高 11 位。
