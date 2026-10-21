#include "motor_can_frame.h"

/* 按截图的字节2～5规则，将 29 位 ID 高位优先放入 4 字节。 */
static void MotorCanFrame_EncodeId(uint32_t can_id, uint8_t *dst)
{
    dst[0] = (uint8_t)((can_id >> 21) & 0xFFU); /* ID28..ID21 */
    dst[1] = (uint8_t)((can_id >> 13) & 0xFFU); /* ID20..ID13 */
    dst[2] = (uint8_t)((can_id >> 5) & 0xFFU);  /* ID12..ID5  */
    dst[3] = (uint8_t)((can_id & 0x1FU) << 3);  /* ID4..ID0，低3位保留 */
}

static uint32_t MotorCanFrame_DecodeId(const uint8_t *src)
{
    return (((uint32_t)src[0]) << 21) |
           (((uint32_t)src[1]) << 13) |
           (((uint32_t)src[2]) << 5) |
           (((uint32_t)src[3]) >> 3);
}

int MotorCanFrame_PackDescription(const MotorCanFrame *frame,
                                  uint8_t out_frame[MOTOR_CAN_DESC_FRAME_LEN])
{
    uint32_t i;
    uint32_t max_id;

    if ((frame == 0) || (out_frame == 0)) {
        return -1;
    }
    if ((frame->dlc > MOTOR_CAN_DATA_MAX_LEN) ||
        (frame->is_extended > 1U) || (frame->is_remote > 1U)) {
        return -2;
    }

    max_id = (frame->is_extended != 0U) ? MOTOR_CAN_EXTENDED_ID_MAX
                                        : MOTOR_CAN_STANDARD_ID_MAX;
    if (frame->can_id > max_id) {
        return -3;
    }

    out_frame[0] = (uint8_t)(frame->dlc & MOTOR_CAN_DESC_DLC_MASK);
    if (frame->is_extended != 0U) {
        out_frame[0] |= MOTOR_CAN_DESC_FLAG_EXTENDED;
    }
    if (frame->is_remote != 0U) {
        out_frame[0] |= MOTOR_CAN_DESC_FLAG_RTR;
    }

    /* 图示中标准 ID 使用 29 位字段的 ID28..ID18，即左移 18 位。 */
    if (frame->is_extended != 0U) {
        MotorCanFrame_EncodeId(frame->can_id, &out_frame[1]);
    } else {
        MotorCanFrame_EncodeId(frame->can_id << 18, &out_frame[1]);
    }

    /* 远程帧没有数据区；数据帧将 DLC 以外的槽位清零，避免残留旧数据。 */
    for (i = 0U; i < MOTOR_CAN_DATA_MAX_LEN; ++i) {
        if ((frame->is_remote == 0U) && (i < frame->dlc)) {
            out_frame[MOTOR_CAN_DESC_HEADER_LEN + i] = frame->data[i];
        } else {
            out_frame[MOTOR_CAN_DESC_HEADER_LEN + i] = 0U;
        }
    }

    return 0;
}

int MotorCanFrame_UnpackDescription(
    const uint8_t in_frame[MOTOR_CAN_DESC_FRAME_LEN], MotorCanFrame *frame)
{
    uint32_t i;
    uint32_t max_id;

    if ((in_frame == 0) || (frame == 0)) {
        return -1;
    }

    frame->is_extended =
        ((in_frame[0] & MOTOR_CAN_DESC_FLAG_EXTENDED) != 0U) ? 1U : 0U;
    frame->is_remote =
        ((in_frame[0] & MOTOR_CAN_DESC_FLAG_RTR) != 0U) ? 1U : 0U;
    frame->dlc = (uint8_t)(in_frame[0] & MOTOR_CAN_DESC_DLC_MASK);
    if (frame->dlc > MOTOR_CAN_DATA_MAX_LEN) {
        return -2;
    }

    frame->can_id = MotorCanFrame_DecodeId(&in_frame[1]);
    if (frame->is_extended != 0U) {
        max_id = MOTOR_CAN_EXTENDED_ID_MAX;
        if (frame->can_id > max_id) {
            return -3;
        }
    } else {
        /* 标准 ID 必须占据 ID28..ID18；低 18 位应为 0。 */
        if ((frame->can_id & 0x3FFFFU) != 0U) {
            return -3;
        }
        frame->can_id >>= 18;
        if (frame->can_id > MOTOR_CAN_STANDARD_ID_MAX) {
            return -3;
        }
    }

    for (i = 0U; i < MOTOR_CAN_DATA_MAX_LEN; ++i) {
        frame->data[i] = ((frame->is_remote == 0U) && (i < frame->dlc))
                             ? in_frame[MOTOR_CAN_DESC_HEADER_LEN + i]
                             : 0U;
    }

    return 0;
}
