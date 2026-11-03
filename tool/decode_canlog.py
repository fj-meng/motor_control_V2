#!/usr/bin/env python3
"""
CANLOG.BIN 解算工具 —— 把每条记录解成「时间 + 角度 + 速度 + 电流」

文件结构
    0   ~ 511 : 文件头（"CANLOG01"）
    512 ~     : 32 字节一条记录，16 条一扇区

文件头（版本 2）
    off 0   char[8] Magic         "CANLOG01"
    off 8   u32     RecordSize    32
    off 12  u32     Version       1 / 2
    off 16  u32     Flags         bit0 = UtcOffsetSec 有效（版本 2 才有）
    off 20  i32     UtcOffsetSec  UTC_sec = sec_cnt + offset
    off 24  u32     StartTick

记录
    off 0   u32  Ts        CAN 硬件时间戳，微秒（30ns 分辨率，1.966ms 回绕）
    off 4   u32  Id        线性仲裁 ID
    off 8   u8   Dlc
    off 9   u8   Ide       1 = 扩展帧
    off 10  u8   Rtr       1 = 远程帧
    off 11  u8   Flag
    off 12  u8x8 Data
    off 20  u32  AbsNsLo   PL 上电后的纳秒数（低 32 位）
    off 24  u32  AbsNsHi   高 32 位
    off 28  u8x4 Pad

应答解码（手册 7.4.4 单圈 / 7.6.3 增量）
    data[0]     电机 ID
    data[1..2]  位置：单圈模式下是 16 位绝对位置；增量模式下是 32 位的低 16 位
    data[3]     速度 [11:4]
    data[4]     速度 [3:0] | 电流 [11:8]
    data[5]     电流 [7:0]
    data[6..7]  位置 [31:16]（只有 DLC=8 的增量模式才有）

    速度码 0x800 = 0，满量程 58.639 rad/s
    电流码 0x800 = 0，满量程 32 A
    增量位置 1048576 码 = 1 圈，±2048 圈内连续累加

用法
    python decode_canlog.py CANLOG.BIN                # 摘要 + 前 20 条
    python decode_canlog.py CANLOG.BIN -n 200         # 摘要 + 前 200 条
    python decode_canlog.py CANLOG.BIN -o out.csv     # 导出全部为 CSV
    python decode_canlog.py CANLOG.BIN --txt out.txt  # 导出全部为 TXT
    python decode_canlog.py CANLOG.BIN -o a.csv -n 500   # 只导前 500 条
    python decode_canlog.py CANLOG.BIN --all          # 表格打印全部
"""

import argparse
import datetime
import struct
import sys

HEADER_SIZE = 512
RECORD_SIZE = 32

# ---- 电机协议常量（和 motor_can_commands.h 保持一致）----
VEL_ZERO = 0x0800           # 速度码零点
CUR_ZERO = 0x0800           # 电流码零点
POS_ZERO = 0x8000           # 单圈位置零点（0°）
SPEED_FULL_SCALE = 58.639   # rad/s
CURRENT_FULL_SCALE = 32.0   # A
COUNT_PER_TURN = 1048576    # 增量位置每圈码数


def load(path):
    with open(path, "rb") as f:
        raw = f.read()

    if len(raw) < HEADER_SIZE:
        raise ValueError(f"文件只有 {len(raw)} 字节，小于一个扇区的文件头，不是合法日志")

    magic = raw[:8].decode("ascii", "replace").rstrip("\x00")
    if magic != "CANLOG01":
        raise ValueError(f"文件头不是 'CANLOG01'，实际是 {magic!r}")

    rec_size, version = struct.unpack_from("<II", raw, 8)
    if rec_size != RECORD_SIZE:
        raise ValueError(f"RecordSize={rec_size}，期望 {RECORD_SIZE}")

    if version >= 2:
        flags, utc_off = struct.unpack_from("<Ii", raw, 16)
        start_tick = struct.unpack_from("<I", raw, 24)[0]
    else:
        flags, utc_off, start_tick = 0, 0, struct.unpack_from("<I", raw, 20)[0]

    hdr = {
        "magic": magic,
        "version": version,
        "record_size": rec_size,
        "utc_valid": (flags & 1) != 0,
        "utc_offset": utc_off,
        "start_tick": start_tick,
        "file_size": len(raw),
    }

    body = raw[HEADER_SIZE:]
    n = len(body) // RECORD_SIZE
    recs = []
    for i in range(n):
        off = i * RECORD_SIZE
        ts, can_id, dlc, ide, rtr, flag = struct.unpack_from("<IIBBBB", body, off)
        data = body[off + 12: off + 20]
        lo, hi = struct.unpack_from("<II", body, off + 20)
        abs_ns = (hi << 32) | lo

        r = {
            "idx": i,
            "ts_us": ts,
            "id": can_id,
            "dlc": dlc,
            "ide": ide,
            "rtr": rtr,
            "flag": flag,
            "data": data,
            "abs_ns": abs_ns,
            "utc_ns": (abs_ns + utc_off * 1_000_000_000) if (hdr["utc_valid"] and abs_ns) else 0,
            # 解码结果先占位，后面填
            "deg": None,
            "turns": None,
            "total_deg": None,
            "rad_s": None,
            "amp": None,
        }
        decode(r)
        recs.append(r)

    return hdr, recs


def decode(r):
    """把一条记录的 data 解成角度/速度/电流。解不出来就留 None。"""
    d = r["data"]
    if (r["ide"] != 0) or (r["rtr"] != 0) or (r["dlc"] < 6):
        return

    # 速度 12 位：[3] 是高位，[4] 的高 4 位是低位
    vel = (d[3] << 4) | (d[4] >> 4)
    # 电流 12 位：[4] 的低 4 位是高位，[5] 是低位
    cur = ((d[4] & 0x0F) << 8) | d[5]

    r["rad_s"] = (vel - VEL_ZERO) / VEL_ZERO * SPEED_FULL_SCALE
    r["amp"] = (cur - CUR_ZERO) / CUR_ZERO * CURRENT_FULL_SCALE

    if r["dlc"] >= 8:
        # 增量位置：32 位有符号，1048576 码一圈，±2048 圈
        pos32 = (d[6] << 24) | (d[7] << 16) | (d[1] << 8) | d[2]
        if pos32 & 0x80000000:
            pos32 -= 0x100000000
        turns, rem = divmod(pos32, COUNT_PER_TURN)   # Python 是地板除，负数也对
        r["turns"] = turns
        r["deg"] = rem * 360.0 / COUNT_PER_TURN      # 圈内角度 0~360
        r["total_deg"] = pos32 * 360.0 / COUNT_PER_TURN  # 累计角度，转多少圈都不回绕
    else:
        # 单圈绝对位置：16 位，0x8000 = 0°
        pos16 = (d[1] << 8) | d[2]
        r["turns"] = 0
        r["deg"] = ((pos16 - POS_ZERO) & 0x7FFF) * 360.0 / POS_ZERO
        r["total_deg"] = r["deg"]


def fmt_utc(ns):
    sec, rem = divmod(int(ns), 1_000_000_000)
    dt = datetime.datetime.fromtimestamp(sec, datetime.timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S") + f".{rem:09d}Z"


def fmt_uptime(ns):
    sec, rem = divmod(int(ns), 1_000_000_000)
    return f"{sec}.{rem:09d}"


def print_summary(hdr, recs):
    print(f"文件        : {hdr['magic']}  v{hdr['version']}  "
          f"{hdr['file_size'] / 1024:.1f} KB")
    print(f"记录数      : {len(recs)}")
    if not recs:
        return

    if hdr["utc_valid"]:
        print(f"UTC 偏移    : {hdr['utc_offset']:+d} s")
    else:
        print("UTC 偏移    : 未设置（文件里只有「上电后过了多久」）")

    good = [r for r in recs if r["abs_ns"] > 0]
    if not good:
        print("AbsNs 全是 0 —— PL 时间基准没读到")
        return

    span = (good[-1]["abs_ns"] - good[0]["abs_ns"]) / 1e9
    print(f"时间跨度    : {span:.3f} s")
    if hdr["utc_valid"]:
        print(f"UTC 起      : {fmt_utc(good[0]['utc_ns'])}")
        print(f"UTC 止      : {fmt_utc(good[-1]['utc_ns'])}")
    else:
        print(f"上电起      : {fmt_uptime(good[0]['abs_ns'])} s")
        print(f"上电止      : {fmt_uptime(good[-1]['abs_ns'])} s")

    if span > 0:
        print(f"平均帧率    : {len(good) / span:.1f} 帧/秒")

    # 帧间隔（用 CAN 硬件时间戳，30ns 精度）
    gaps = []
    for i in range(1, len(good)):
        d = good[i]["ts_us"] - good[i - 1]["ts_us"]
        if d > 0:
            gaps.append(d)
    if gaps:
        gaps.sort()
        print(f"帧间隔      : 中位数 {gaps[len(gaps) // 2]} us, "
              f"范围 {gaps[0]} ~ {gaps[-1]} us")

    ids = sorted({r["id"] for r in recs})
    print(f"CAN ID      : {', '.join(hex(x) for x in ids)}")

    # 角度范围
    ang = [r["total_deg"] for r in recs if r["total_deg"] is not None]
    if ang:
        print(f"累计角度    : {min(ang):+.2f}° ~ {max(ang):+.2f}° "
              f"(跨度 {max(ang) - min(ang):.2f}°)")

    spd = [r["rad_s"] for r in recs if r["rad_s"] is not None]
    if spd:
        print(f"速度        : {min(spd):+.2f} ~ {max(spd):+.2f} rad/s")


HEAD = ("idx", "utc", "uptime_s", "ts_us", "id", "dlc",
        "turns", "deg", "total_deg", "rad_s", "amp", "data_hex")


def row_of(r):
    utc = fmt_utc(r["utc_ns"]) if r["utc_ns"] else ""
    return (
        r["idx"],
        utc,
        fmt_uptime(r["abs_ns"]) if r["abs_ns"] else "",
        r["ts_us"],
        f"0x{r['id']:X}",
        r["dlc"],
        "" if r["turns"] is None else r["turns"],
        "" if r["deg"] is None else f"{r['deg']:.3f}",
        "" if r["total_deg"] is None else f"{r['total_deg']:.3f}",
        "" if r["rad_s"] is None else f"{r['rad_s']:.3f}",
        "" if r["amp"] is None else f"{r['amp']:.3f}",
        r["data"][:r["dlc"]].hex().upper(),
    )


def print_table(recs, limit):
    print()
    print(f"{'idx':>5} {'UTC':<30} {'上电(s)':>14} {'角度(°)':>10} "
          f"{'累计(°)':>12} {'rad/s':>9} {'A':>7}  data")
    print("-" * 118)
    for r in recs[:limit]:
        utc = fmt_utc(r["utc_ns"]) if r["utc_ns"] else "-"
        up = fmt_uptime(r["abs_ns"]) if r["abs_ns"] else "-"
        deg = "-" if r["deg"] is None else f"{r['deg']:.3f}"
        tot = "-" if r["total_deg"] is None else f"{r['total_deg']:.3f}"
        rs = "-" if r["rad_s"] is None else f"{r['rad_s']:.3f}"
        am = "-" if r["amp"] is None else f"{r['amp']:.3f}"
        print(f"{r['idx']:>5} {utc:<30} {up:>14} {deg:>10} {tot:>12} "
              f"{rs:>9} {am:>7}  {r['data'][:r['dlc']].hex().upper()}")


def export_csv(path, recs, limit):
    # utf-8-sig：让 Excel 双击打开不乱码
    with open(path, "w", encoding="utf-8-sig", newline="") as f:
        f.write(",".join(HEAD) + "\n")
        for r in recs[:limit]:
            f.write(",".join(str(x) for x in row_of(r)) + "\n")


def export_txt(path, recs, limit):
    with open(path, "w", encoding="utf-8") as f:
        f.write("CANLOG 解算结果\n")
        f.write(f"记录数: {min(limit, len(recs))}\n")
        f.write("字段: idx / utc / uptime_s / ts_us / id / dlc / "
                "turns / deg / total_deg / rad_s / amp / data_hex\n")
        f.write("=" * 118 + "\n")
        for r in recs[:limit]:
            f.write("  ".join(str(x) for x in row_of(r)) + "\n")


def main():
    ap = argparse.ArgumentParser(
        description="CANLOG.BIN 解算：时间 + 角度 + 速度 + 电流",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("用法")[1] if "用法" in __doc__ else "")
    ap.add_argument("path", help="CANLOG.BIN 路径")
    ap.add_argument("-n", "--limit", type=int, default=20,
                    help="打印/导出多少条，默认 20；-1 表示全部")
    ap.add_argument("--all", action="store_true", help="表格打印全部记录")
    ap.add_argument("-o", "--csv", help="导出 CSV 到指定文件")
    ap.add_argument("--txt", help="导出 TXT 到指定文件")
    args = ap.parse_args()

    try:
        hdr, recs = load(args.path)
    except (OSError, ValueError) as exc:
        print(f"读取失败: {exc}", file=sys.stderr)
        return 1

    print_summary(hdr, recs)
    if not recs:
        return 0

    limit = len(recs) if (args.all or args.limit < 0) else args.limit
    print_table(recs, limit)
    if limit < len(recs):
        print(f"... 还有 {len(recs) - limit} 条未显示，用 --all 或 -n 查看")

    if args.csv:
        export_csv(args.csv, recs, len(recs))
        print(f"\nCSV 已写出: {args.csv}（{len(recs)} 条）")
    if args.txt:
        export_txt(args.txt, recs, len(recs))
        print(f"TXT 已写出: {args.txt}（{len(recs)} 条）")

    return 0


if __name__ == "__main__":
    sys.exit(main())
