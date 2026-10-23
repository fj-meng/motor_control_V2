#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CANLOG.BIN 解析工具

文件由 motor_can20 的 logger.c 生成，布局：

    偏移 0      512 字节文件头（占满一个扇区，保证记录区扇区对齐）
                  char Magic[8]     "CANLOG01"
                  u32  RecordSize   32
                  u32  Version      1
                  u32  Reserved
                  u32  StartTick
                  u8   Pad[488]
    偏移 512    之后全是 32 字节记录，小端：
                  u32 Ts            微秒时间戳（已扩展为 32 位）
                  u32 Id            线性仲裁 ID
                  u8  Dlc / Ide / Rtr / Flag
                  u8  Data[8]
                  u8  Pad[12]

用法：
    python decode_canlog.py CANLOG.BIN
    python decode_canlog.py CANLOG.BIN -o canlog.csv

不指定 -o 时只打印统计，不写 CSV。
"""

import argparse
import math
import struct
import sys

HEADER_SIZE = 512
RECORD_FMT = "<IIBBBB8s12s"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
MAGIC = b"CANLOG01"

# 电机 1ms 自动上传时，正常帧间隔约 1000us。
# 超过 1.5ms 认为中间少了一帧，超过 10ms 认为是总线静默而不是丢帧。
GAP_MIN_US = 1500
GAP_MAX_US = 10000

# ---- 电机协议常数（手册 7.4 / 7.6） ----
VEL_ZERO = 2048              # 12 位速度码零点
CUR_ZERO = 2048              # 12 位电流码零点
VEL_FULL_RADS = 58.639       # 半量程对应的 rad/s（HO10010 速度常数）
CUR_FULL_AMP = 32.0          # 半量程对应的电流 A
INC_COUNTS_PER_TURN = 1048576  # 增量模式：1048576 码 = 1 圈
HOST_ID = 100                # 电机回给主机的帧用这个仲裁 ID


def decode_motor(rec):
    """
    把一条电机应答帧解成物理量。不是电机应答帧就返回 None。

    6 字节（单圈绝对位置模式）：
        data[0]     电机 ID
        data[1..2]  位置 16 位，0x8000 = 0 度，32768 码 = 360 度
        data[3]     速度 [11:4]
        data[4]     速度 [3:0] | 电流 [11:8]
        data[5]     电流 [7:0]

    8 字节（增量位置模式）：速度和电流的位置完全一样，只是位置扩成 32 位，
        data[1..2] 是位置低 16 位，data[6..7] 是位置高 16 位。
        位置 = data[6]<<24 | data[7]<<16 | data[1]<<8 | data[2]
    """
    if (rec["ide"] != 0 or rec["id"] != HOST_ID
            or rec["dlc"] < 6 or rec["rtr"] != 0):
        return None

    d = rec["data"]

    if rec["dlc"] >= 8:
        pos32 = (d[6] << 24) | (d[7] << 16) | (d[1] << 8) | d[2]
        if pos32 >= 2 ** 31:
            pos32 -= 2 ** 32
        turns = pos32 / INC_COUNTS_PER_TURN
        frac = pos32 % INC_COUNTS_PER_TURN
        if frac < 0:
            frac += INC_COUNTS_PER_TURN
        angle = frac / INC_COUNTS_PER_TURN * 360.0
    else:
        p16 = (d[1] << 8) | d[2]
        turns = None
        # 单圈模式：对 0x8000 的偏移量按 32768 取模，得到 0~360 的锯齿波
        angle = (((p16 - 32768) & 0x7FFF) * 360.0) / 32768.0

    vel_raw = (d[3] << 4) | (d[4] >> 4)
    cur_raw = ((d[4] & 0x0F) << 8) | d[5]
    rads = (vel_raw - VEL_ZERO) / VEL_ZERO * VEL_FULL_RADS

    return {
        "turns": turns,
        "angle": angle,
        "rads": rads,
        "rpm": rads * 60.0 / (2.0 * math.pi),
        "amp": (cur_raw - CUR_ZERO) / CUR_ZERO * CUR_FULL_AMP,
    }


def load(path):
    with open(path, "rb") as f:
        blob = f.read()

    if len(blob) < HEADER_SIZE:
        raise SystemExit("文件太短，不是 CANLOG 文件")

    magic = blob[:8]
    if magic != MAGIC:
        raise SystemExit("文件头不匹配：期望 %r，实际 %r" % (MAGIC, magic))

    rec_size, version, _reserved, start_tick = struct.unpack_from("<IIII", blob, 8)
    if rec_size != RECORD_SIZE:
        raise SystemExit("记录长度 %d 与本脚本的 %d 不一致" % (rec_size, RECORD_SIZE))

    print("文件头: magic=%s version=%d record=%d bytes start_tick=%d"
          % (magic.decode(), version, rec_size, start_tick))

    body = blob[HEADER_SIZE:]
    count = len(body) // RECORD_SIZE
    tail = len(body) % RECORD_SIZE
    if tail:
        print("警告: 尾部有 %d 字节残片（可能是掉电时写到一半）" % tail)

    records = []
    for i in range(count):
        off = i * RECORD_SIZE
        ts, can_id, dlc, ide, rtr, flag, data, _pad = struct.unpack_from(
            RECORD_FMT, body, off)
        records.append({
            "ts": ts,
            "id": can_id,
            "dlc": dlc,
            "ide": ide,
            "rtr": rtr,
            "flag": flag,
            "data": data[:dlc] if dlc <= 8 else data,
        })
    return records


def analyse(records):
    if not records:
        print("没有任何记录")
        return

    # 32 位微秒时间戳约 71.6 分钟回绕一次，这里累加成 64 位绝对时间。
    base = records[0]["ts"]
    wrap = 0
    prev_raw = base
    timeline = []

    for r in records:
        raw = r["ts"]
        if raw < prev_raw:
            wrap += 1 << 32
        prev_raw = raw
        timeline.append((raw + wrap) - base)

    deltas = [timeline[i] - timeline[i - 1] for i in range(1, len(timeline))]
    gaps = [d for d in deltas if GAP_MIN_US < d < GAP_MAX_US]
    silences = [d for d in deltas if d >= GAP_MAX_US]

    duration_us = timeline[-1] if timeline else 0
    print()
    print("记录数      : %d" % len(records))
    print("时间跨度    : %.3f s" % (duration_us / 1e6))
    if duration_us > 0:
        print("平均帧率    : %.1f 帧/秒" % (len(records) / (duration_us / 1e6)))

    if deltas:
        print("帧间隔      : min=%d us  mean=%.1f us  max=%d us"
              % (min(deltas), sum(deltas) / len(deltas), max(deltas)))
        print("疑似丢帧    : %d 次（间隔在 %d~%d us 之间）"
              % (len(gaps), GAP_MIN_US, GAP_MAX_US))
        print("总线静默    : %d 次（间隔 >= %d us，不算丢帧）"
              % (len(silences), GAP_MAX_US))

        if len(gaps) > 3:
            print("  前几次丢帧的位置:")
            shown = 0
            for i, d in enumerate(deltas):
                if GAP_MIN_US < d < GAP_MAX_US:
                    print("    第 %d 条之后，间隔 %d us" % (i + 1, d))
                    shown += 1
                    if shown >= 5:
                        break

    # 按 ID 分布，快速看出总线上都有什么
    stat = {}
    for r in records:
        key = (r["id"], r["ide"])
        stat[key] = stat.get(key, 0) + 1
    print("ID 分布     :")
    for (can_id, ide), n in sorted(stat.items(), key=lambda kv: -kv[1]):
        print("    id=0x%08X %s  %d 帧"
              % (can_id, "EXT" if ide else "STD", n))


def write_csv(records, path):
    """
    导出 CSV，电机应答帧会多出 turns / angle_deg / rpm / current_a 四列。

    时间列给两个：ts_us 是文件里的原始 32 位微秒时间戳（约 71.6 分钟回绕
    一次），time_s 是处理过回绕的、相对首帧的秒数，画图用这个。
    """
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write("index,ts_us,time_s,id_hex,ide,dlc,"
                "turns,angle_deg,rpm,current_a,data_hex\n")

        base = records[0]["ts"] if records else 0
        wrap = 0
        prev = base

        for i, r in enumerate(records):
            raw = r["ts"]
            if raw < prev:
                wrap += 1 << 32
            prev = raw
            t_us = raw + wrap - base

            m = decode_motor(r)
            if m is None:
                turns = angle = rpm = amp = ""
            else:
                turns = "" if m["turns"] is None else "%.6f" % m["turns"]
                angle = "%.3f" % m["angle"]
                rpm = "%.4f" % m["rpm"]
                amp = "%.4f" % m["amp"]

            f.write("%d,%d,%.6f,0x%08X,%d,%d,%s,%s,%s,%s,%s\n"
                    % (i, r["ts"], t_us / 1e6, r["id"], r["ide"], r["dlc"],
                       turns, angle, rpm, amp, r["data"].hex()))

    print()
    print("CSV 已写入 %s" % path)


def main():
    ap = argparse.ArgumentParser(description="解析 CANLOG.BIN")
    ap.add_argument("path", help="CANLOG.BIN 路径")
    ap.add_argument("-o", "--output", help="导出 CSV 的路径")
    args = ap.parse_args()

    try:
        records = load(args.path)
    except OSError as exc:
        raise SystemExit("打不开文件: %s" % exc)

    analyse(records)

    if args.output:
        write_csv(records, args.output)

    return 0


if __name__ == "__main__":
    sys.exit(main())
