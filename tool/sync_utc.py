#!/usr/bin/env python3
"""
用电脑给 PL 时间基准对表（自动取时间 + 自动发送）

为什么需要脚本
--------------
手工「看一眼表 → 敲秒数 → 回车」要花好几秒。等命令到板子上时，
你敲进去的那个秒数早就过期了，offset 会整整差好几秒。

而且这个误差不是「小偏差」，是整数秒的偏差 —— 因为 offset 只对齐
「整秒」，秒沿由 GNSS 1PPS 定义。你敲了 8 秒，就错 8 秒。

脚本做的事
----------
1. 等到秒的中间（默认 0.25 ~ 0.65）再取值，这样串口传输延迟几毫秒
   根本不可能跨过秒边界
2. 立刻取 time.time()，发送 setutc <秒>
3. 读回板子的确认，失败自动重试一次

对表精度
--------
offset 只对齐整秒，亚秒由 PL 的 tick 提供（已做 ppm 校正）。
所以对表精度与串口传输延迟无关，只取决于 PPS 检测延迟（约 40 ns，固定）。
电脑时间只要准到 ±400 ms 就不会跨秒出错，Windows 开着自动同步就够。

用法
----
    python sync_utc.py -p COM3              # 对表
    python sync_utc.py -p COM3 --check      # 只看板子当前 UTC，不改
    python sync_utc.py -p COM3 --dry-run    # 只打印要发的命令，不连串口

    # Linux / macOS 端口名长这样
    python sync_utc.py -p /dev/ttyUSB0

依赖
----
    pip install pyserial
    （--dry-run 不需要 pyserial）
"""

import argparse
import sys
import time

DEFAULT_BAUD = 115200

# 板子每行都带这些前缀，其余（心跳 rx=、latest:、字符回显）一律丢掉
KEEP_PREFIX = ("CMD:", "LOG:", "Motor:", "TIME:", "FAIL", "TIME")


def wait_mid_second(lo, hi):
    """
    等到「秒内比例」落在 [lo, hi] 再返回。

    这是整个脚本最关键的一步：在秒的中间发，几毫秒的传输延迟就
    不可能把命令推过秒边界，offset 一定是整秒正确的。
    """
    while True:
        frac = time.time() % 1.0
        if lo <= frac <= hi:
            return
        if frac > hi:
            # 已经过了窗口，睡到下一秒的窗口开头
            time.sleep(min(1.0 - frac + lo, 0.5))
        else:
            time.sleep(0.002)


def read_reply(ser, timeout=2.5):
    """读到第一条 CMD:/FAIL 开头的行就返回。最多等 timeout 秒。"""
    end = time.time() + timeout
    buf = b""
    lines = []

    while time.time() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            s = raw.decode("ascii", "replace").strip()
            if not s:
                continue
            if s.startswith(KEEP_PREFIX):
                lines.append(s)
                if s.startswith(("CMD:", "FAIL")):
                    return lines
    return lines


def fmt_utc(t):
    return time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(t))


def fmt_local(t):
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t))


def tz_label(t):
    """返回 'UTC+8' 这种时区标签，跟着夏令时走。"""
    try:
        is_dst = time.localtime(t).tm_isdst > 0
    except (ValueError, OSError):
        is_dst = False
    offset = -(time.altzone if is_dst else time.timezone)
    sign = "+" if offset >= 0 else "-"
    hours, minutes = divmod(abs(offset) // 60, 60)
    return f"UTC{sign}{hours}" + (f":{minutes:02d}" if minutes else "")


def show_time(t, prefix=""):
    """同时打印 UTC 和本地时间，避免把 UTC 当成自己手表上的时间。"""
    print(f"{prefix}UTC 时间: {fmt_utc(t)}")
    print(f"{prefix}本地时间: {fmt_local(t)} ({tz_label(t)})")


def main():
    ap = argparse.ArgumentParser(
        description="用电脑给 Zynq PL 时间基准对表（自动取时 + 自动发送）")
    ap.add_argument("-p", "--port", required=True,
                    help="串口，Windows 写 COM3，Linux 写 /dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--check", action="store_true",
                    help="只读板子当前 UTC，不对表")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印要发的命令，不打开串口")
    ap.add_argument("--lo", type=float, default=0.25,
                    help="秒内取值窗口下界，默认 0.25")
    ap.add_argument("--hi", type=float, default=0.65,
                    help="秒内取值窗口上界，默认 0.65")
    args = ap.parse_args()

    if args.dry_run:
        wait_mid_second(args.lo, args.hi)
        t = time.time()
        print(f"[dry-run] 秒内比例 {t % 1:.3f}")
        print(f"[dry-run] 要发的命令: setutc {int(t)}")
        print(f"[dry-run] 这个秒数对应：")
        show_time(t, "           ")
        return 0

    try:
        import serial
    except ImportError:
        print("需要 pyserial:  pip install pyserial", file=sys.stderr)
        return 1

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as exc:                      # noqa: BLE001
        print(f"打开串口 {args.port} 失败: {exc}", file=sys.stderr)
        print("检查：端口号对不对？串口助手是不是还占着？", file=sys.stderr)
        return 1

    with ser:
        time.sleep(0.1)
        ser.reset_input_buffer()

        # ---- 只看状态 ----
        if args.check:
            ser.write(b"setutc\r\n")
            lines = read_reply(ser)
            if not lines:
                print("没收到回复，检查波特率是不是 115200", file=sys.stderr)
                return 1
            for line in lines:
                print(line)
            return 0

        # ---- 对表，最多试两次 ----
        for attempt in (1, 2):
            wait_mid_second(args.lo, args.hi)

            # 取值和发送之间只隔一次 write()，几毫秒，跨不了秒
            t = time.time()
            sec = int(t)
            ser.write(f"setutc {sec}\r\n".encode())

            lines = read_reply(ser)
            for line in lines:
                print(line)

            if any("FAILED" in line for line in lines):
                print(f"[重试 {attempt}/2] 太靠近秒边界，换个时机再来")
                continue

            if any(line.startswith("CMD: UTC =") for line in lines):
                print()
                print("[OK] 已对表，电脑这一时刻是：")
                show_time(t, "     ")
                print(f"     秒内比例 {t % 1:.3f}")
                print()
                print("     注意：板子上打印的 CMD: UTC now = ... 是 UTC，")
                print("     不是本地时间，别拿去和自己的手表对。")
                return 0

            if not lines:
                print("没收到回复。检查：波特率 115200？板子在跑？"
                      "串口助手没占着？", file=sys.stderr)
            else:
                print("回复里没有 UTC 确认行，看上面打印", file=sys.stderr)
            return 1

    return 1


if __name__ == "__main__":
    sys.exit(main())
