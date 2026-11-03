#!/usr/bin/env python3
"""
解析 CANLOG.BIN，并算出每帧的精确绝对时间

文件结构：
    0   ~ 511  : 512 字节文件头（"CANLOG01"，前 28 字节有用）
    512 ~      : 32 字节一条记录

文件头（版本 2）：
    off 0   char[8] Magic       "CANLOG01"
    off 8   u32     RecordSize  32
    off 12  u32     Version     2
    off 16  u32     Flags       bit0 = UtcOffsetSec 有效
    off 20  i32     UtcOffsetSec  UTC_sec = sec_cnt + UtcOffsetSec
    off 24  u32     StartTick

记录结构（小端）：
    off 0   u32  Ts         CAN 硬件时间戳，微秒（30ns 分辨率，存成 us）
    off 4   u32  Id         线性仲裁 ID
    off 8   u8   Dlc
    off 9   u8   Ide        1 = 扩展帧
    off 10  u8   Rtr
    off 11  u8   Flag
    off 12  u8x8 Data
    off 20  u32  AbsNsLo    PL 时间低 32 位（ns），上电后的单调时间
    off 24  u32  AbsNsHi    PL 时间高 32 位（ns）
    off 28  u8x4 Pad

核心原理：
    Ts     精度高（30ns）但只是相对计数器，且只有 1.966ms 窗口
    AbsNs  上电后的单调时间，有 1~3us 的中断抖动，不知道是几点
    UTC    = AbsNs + 文件头里的 UtcOffsetSec × 1e9（setutc 命令设置）

    对全文件做一次最小二乘拟合  AbsNs = a + b * Ts
      b = PL 时钟 / PS_CLK 的精确频率比
      a = 常数偏置（含中断延迟，对同一批数据是固定的）
    拟合直线 a + b*Ts 就是「去掉了抖动的绝对时间」。

用法：
    python parse_log.py CANLOG.BIN              # 统计 + 拟合诊断
    python parse_log.py CANLOG.BIN -v           # 额外打印前 20 条
    python parse_log.py CANLOG.BIN -o out.csv   # 导出，含 abs_fit_ns 列
    python parse_log.py CANLOG.BIN -w 5000      # 滑动窗口拟合，窗口 5000 帧
"""

import datetime
import struct
import sys

HEADER_SIZE = 512
RECORD_SIZE = 32


def load(path):
    with open(path, "rb") as f:
        raw = f.read()

    if len(raw) < HEADER_SIZE:
        raise ValueError("文件小于 512 字节，不是合法的日志")

    magic = raw[:8].decode("ascii", "replace").rstrip("\x00")
    rec_size, version = struct.unpack_from("<II", raw, 8)

    if magic != "CANLOG01":
        raise ValueError(f"magic 不对：{magic!r}，期望 'CANLOG01'")
    if rec_size != RECORD_SIZE:
        raise ValueError(f"RecordSize={rec_size}，期望 {RECORD_SIZE}")

    # 版本 2 起，文件头偏移 16 处是 Flags，20 处是 UtcOffsetSec。
    # 版本 1 那里是 Reserved + StartTick，没有 UTC 信息。
    if version >= 2:
        flags, utc_off = struct.unpack_from("<Ii", raw, 16)
    else:
        flags, utc_off = 0, 0
    utc_valid = (flags & 1) != 0

    body = raw[HEADER_SIZE:]
    n = len(body) // RECORD_SIZE

    out = []
    for i in range(n):
        off = i * RECORD_SIZE
        ts, can_id, dlc, ide, rtr, flag = struct.unpack_from("<IIBBBB", body, off)
        data = body[off + 12: off + 20]
        lo, hi = struct.unpack_from("<II", body, off + 20)
        abs_ns = (hi << 32) | lo
        out.append({
            "idx": i,
            "ts_us": ts,
            "id": can_id,
            "dlc": dlc,
            "ide": ide,
            "rtr": rtr,
            "flag": flag,
            "data": data,
            "abs_ns": abs_ns,
            # 记录里存的是「上电后过了多少纳秒」，加上文件头的偏移量才是 UTC
            "utc_ns": abs_ns + utc_off * 1_000_000_000 if (utc_valid and abs_ns) else 0,
            "abs_fit": 0,
        })

    return {"magic": magic, "version": version,
            "utc_valid": utc_valid, "utc_offset": utc_off}, out


def fmt_utc(ns):
    """纳秒时间戳 -> 'YYYY-MM-DDTHH:MM:SS.nnnnnnnnnZ'。"""
    sec, rem = divmod(int(ns), 1_000_000_000)
    dt = datetime.datetime.fromtimestamp(sec, datetime.timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S") + f".{rem:09d}Z"


def linfit(xs, ys):
    """最小二乘 y = a + b*x，不依赖 numpy。返回 (b, a)。"""
    n = len(xs)
    if n < 2:
        return 0.0, 0.0
    sx = sum(xs)
    sy = sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    den = n * sxx - sx * sx
    if den == 0:
        return 0.0, float(sy) / n
    b = (n * sxy - sx * sy) / den
    a = (sy - b * sx) / n
    return b, a


def fit_records(recs, window=0):
    """
    给每条记录填上 abs_fit。

    window = 0  -> 全文件一次性拟合
    window > 0  -> 按窗口滑动拟合，抵抗晶振温漂
    """
    valid = [r for r in recs if r["abs_ns"] > 0]
    if len(valid) < 8:
        return None

    xs = [float(r["ts_us"]) for r in valid]
    ys = [float(r["abs_ns"]) for r in valid]

    if window <= 0 or window >= len(valid):
        b, a = linfit(xs, ys)
        for r in valid:
            r["abs_fit"] = a + b * r["ts_us"]
        resid = [r["abs_ns"] - r["abs_fit"] for r in valid]
        return {"b": b, "a": a, "resid": resid, "mode": "全局"}

    # 滑动窗口：每个窗口单独拟合，只用于算该窗口内记录的 abs_fit
    half = window // 2
    resid = []
    for i, r in enumerate(valid):
        lo = max(0, i - half)
        hi = min(len(valid), i + half + 1)
        b, a = linfit(xs[lo:hi], ys[lo:hi])
        r["abs_fit"] = a + b * r["ts_us"]
        resid.append(r["abs_ns"] - r["abs_fit"])

    b, a = linfit(xs, ys)
    return {"b": b, "a": a, "resid": resid, "mode": f"滑动窗口 {window} 帧"}


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    path = sys.argv[1]
    verbose = "-v" in sys.argv
    csv_path = None
    if "-o" in sys.argv:
        csv_path = sys.argv[sys.argv.index("-o") + 1]

    window = 0
    if "-w" in sys.argv:
        window = int(sys.argv[sys.argv.index("-w") + 1])

    hdr, recs = load(path)

    print(f"文件      : {path}")
    print(f"记录数    : {len(recs)}")
    print(f"数据区大小: {len(recs) * RECORD_SIZE / 1024:.1f} KB")
    print(f"文件版本  : {hdr['version']}")

    if hdr["utc_valid"]:
        print(f"UTC 偏移  : {hdr['utc_offset']:+d} s"
              f"（UTC = AbsNs + 偏移，setutc 设置）")
    else:
        print("UTC 偏移  : 未设置 —— 文件里只有「上电后过了多久」，不知道几点")
        print("            下次录制前先发：setutc <unix_seconds>")

    if not recs:
        return 0

    n_bad = sum(1 for r in recs if r["abs_ns"] == 0)
    if n_bad:
        print(f"AbsNs=0  : {n_bad} 条（PL 未就绪，已排除出拟合）")

    valid = [r for r in recs if r["abs_ns"] > 0]
    if not valid:
        print("\n!! AbsNs 全是 0 —— PL 时间基准没读到")
        print("   检查：bitstream 是否下载、基地址是否 0x43C00000、M_AXI_GP0 是否使能")
        return 0

    ts0, ts1 = valid[0]["ts_us"], valid[-1]["ts_us"]
    abs0, abs1 = valid[0]["abs_ns"], valid[-1]["abs_ns"]
    print(f"时间跨度  : Ts {(ts1 - ts0) / 1e6:.3f} s, "
          f"AbsNs {(abs1 - abs0) / 1e9:.3f} s")

    if hdr["utc_valid"]:
        print(f"UTC 起    : {fmt_utc(valid[0]['utc_ns'])}")
        print(f"UTC 止    : {fmt_utc(valid[-1]['utc_ns'])}")

    # ---- 帧间隔 ----
    gaps = [valid[i]["ts_us"] - valid[i - 1]["ts_us"] for i in range(1, len(valid))]
    gaps = sorted(g for g in gaps if g > 0)
    if gaps:
        print(f"Ts 帧间隔 : 中位数 {gaps[len(gaps) // 2]} us, "
              f"范围 {gaps[0]} ~ {gaps[-1]} us")

    # ---- 拟合 ----
    info = fit_records(recs, window)
    if info is None:
        print("\n有效记录少于 8 条，无法拟合")
        return 0

    b, a, resid = info["b"], info["a"], info["resid"]
    mean = sum(resid) / len(resid)
    var = sum((r - mean) ** 2 for r in resid) / len(resid)
    std = var ** 0.5

    print()
    print(f"---- 拟合 AbsNs = a + b·Ts（{info['mode']}）----")
    print(f"斜率 b    : {b:.4f} ns/us")
    print(f"          （理论 1000.0，偏差 {(b / 1000.0 - 1) * 1e6:+.2f} ppm"
          f" = 两个晶振的相对频差）")
    print(f"截距 a    : {a:.0f} ns  （= 常数偏置，含中断延迟，固定值）")
    print(f"残差 std  : {std:.0f} ns  <- 中断抖动，拟合后归零")
    print(f"残差峰峰  : {max(resid) - min(resid):.0f} ns")

    if std > 10000:
        print("  !! 抖动超过 10us，检查 ISR 里是不是做了耗时操作")
    elif std < 200:
        print("  抖动很小，说明中断响应很稳定")

    print()
    print("---- 每帧的精确时间戳 ----")
    print("  abs_fit_ns = a + b * Ts   （推荐，抖动已消除）")
    print(f"  分辨率 ≈ 1 us（Ts 是 us 整数），偏置 a 对整份文件是常数")
    print("  如果只算差值（帧间隔、请求→响应延迟），偏置自动抵消")

    # ---- 抽样打印 ----
    if verbose:
        print()
        print("idx      Ts(us)   AbsNs(us)   abs_fit(us)   残差(us)  data")
        for r in valid[:20]:
            hexd = " ".join(f"{b_:02x}" for b_ in r["data"][:r["dlc"]])
            print(f"{r['idx']:6d} {r['ts_us']:11d} "
                  f"{r['abs_ns'] / 1000:11.1f} "
                  f"{r['abs_fit'] / 1000:12.1f} "
                  f"{(r['abs_ns'] - r['abs_fit']) / 1000:+9.3f}   {hexd}")

    # ---- 导出 ----
    if csv_path:
        with open(csv_path, "w", encoding="utf-8") as f:
            f.write("idx,utc_iso,utc_ns,ts_us,id,ide,rtr,dlc,"
                    "abs_ns,abs_fit_ns,resid_ns,data_hex\n")
            for r in recs:
                utc_iso = fmt_utc(r["utc_ns"]) if r["utc_ns"] else ""
                f.write(f"{r['idx']},{utc_iso},{r['utc_ns']},{r['ts_us']},"
                        f"0x{r['id']:x},{r['ide']},"
                        f"{r['rtr']},{r['dlc']},{r['abs_ns']},"
                        f"{r['abs_fit']:.0f},"
                        f"{r['abs_ns'] - r['abs_fit']:.0f},"
                        f"{r['data'][:r['dlc']].hex()}\n")
        print(f"\nCSV 已写出：{csv_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
