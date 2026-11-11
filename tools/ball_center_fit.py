#!/usr/bin/env python3
"""采集并拟合受约束钢球中心辨识实验。

MCU 每收到一帧新视觉数据就打印一条 BCTRL 记录。本工具不手工选点，而是对
完整的连续 ± 三角波响应拟合：
    x_ddot = k * pipe_angle + c * x_dot
并给出以固件 mm 为单位的保守 P/D 初始参数。
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from pathlib import Path

LINE = re.compile(
    r"BCTRL,t=(?P<t>\d+),x=(?P<x>[+-]?\d+(?:\.\d+)?),"
    r"v=(?P<v>[+-]?\d+(?:\.\d+)?),u=(?P<u>[+-]?\d+(?:\.\d+)?)"
    r"[^\r\n]*?,id=(?P<identify>\d+)"
)


def capture(port: str, baud: int, seconds: float, output: Path) -> None:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("pyserial missing: python -m pip install pyserial") from exc

    output.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + seconds
    count = 0
    with serial.Serial(port, baudrate=baud, timeout=0.25) as ser, \
            output.open("w", encoding="utf-8", newline="") as log:
        while time.monotonic() < deadline:
            line = ser.readline().decode("ascii", errors="replace")
            if not line:
                continue
            log.write(line)
            if LINE.search(line):
                count += 1
    print(f"saved {count} BCTRL frames to {output}")


def load(path: Path) -> list[tuple[float, float, float, int]]:
    rows: list[tuple[float, float, float, int]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = LINE.search(line)
        if not match:
            continue
        # 日志中 x 单位是 cm；控制器和拟合均使用 mm。
        rows.append((float(match["t"]) / 1000.0,
                     float(match["x"]) * 10.0,
                     float(match["u"]),
                     int(match["identify"])))
    return rows


def fit(rows: list[tuple[float, float, float, int]]) -> tuple[float, float, int]:
    # 用中心差分计算速度/加速度。忽略零指令平台段，
    # 因为它无法辨识角度到加速度的增益（angle authority）。
    design: list[tuple[float, float, float]] = []
    for index in range(1, len(rows) - 1):
        t0, x0, _, identify0 = rows[index - 1]
        t1, x1, u1, identify1 = rows[index]
        t2, x2, _, identify2 = rows[index + 1]
        dt_left = t1 - t0
        dt_right = t2 - t1
        if (identify0 != 0 or identify1 != 0 or identify2 != 0 or
                dt_left <= 0.02 or dt_right <= 0.02 or abs(u1) < 0.025):
            continue
        v_left = (x1 - x0) / dt_left
        v_right = (x2 - x1) / dt_right
        acceleration = 2.0 * (v_right - v_left) / (dt_left + dt_right)
        velocity = (x2 - x0) / (dt_left + dt_right)
        if not (math.isfinite(acceleration) and math.isfinite(velocity)):
            continue
        design.append((u1, velocity, acceleration))

    if len(design) < 8:
        raise ValueError(f"only {len(design)} usable frames; need at least 8")

    # 对 a = k*u + c*v 做最小二乘，由 2x2 正规方程求解。
    suu = sum(u * u for u, _, _ in design)
    suv = sum(u * v for u, v, _ in design)
    svv = sum(v * v for _, v, _ in design)
    sua = sum(u * a for u, _, a in design)
    sva = sum(v * a for _, v, a in design)
    determinant = suu * svv - suv * suv
    if abs(determinant) < 1e-6:
        raise ValueError("excitation is degenerate; ball did not respond enough")
    k = (sua * svv - sva * suv) / determinant
    c = (suu * sva - suv * sua) / determinant
    return k, c, len(design)


def report(path: Path) -> None:
    rows = load(path)
    if len(rows) < 10:
        raise SystemExit(f"only {len(rows)} BCTRL records in {path}")
    try:
        authority, velocity_term, count = fit(rows)
    except ValueError as exc:
        raise SystemExit(f"fit rejected: {exc}") from exc

    if abs(authority) < 0.2:
        raise SystemExit("fit rejected: angle authority is too small")

    # x_ddot = k*u + c*x_dot。采用有意放慢的临界阻尼设计
    # （omega=1 rad/s，zeta=0.9），首次闭环不保留 I 项。
    plant = abs(authority)
    kp = max(0.004, min(0.030, 1.0 / plant))
    kd = max(0.010, min(0.150, (1.8 + velocity_term) / plant))
    direction = 1 if authority > 0.0 else -1
    print(f"usable_frames={count}")
    print(f"plant: x_ddot = {authority:+.4f} * theta {velocity_term:+.4f} * x_dot")
    print(f"pipe_command_sign={direction:+d}")
    print("recommended g_static_center_config:")
    print(f"  .kp_pipe_deg_per_mm = {kp:.4f}f,")
    print("  .ki_pipe_deg_per_mm_s = 0.0000f,")
    print(f"  .kd_pipe_deg_per_mm_s = {kd:.4f}f,")
    print("  .integral_limit_deg = 0.00f,")
    print("Keep pipe_angle_limit_deg <= 0.50f for the next run.")


def main() -> None:
    parser = argparse.ArgumentParser()
    command = parser.add_subparsers(dest="command", required=True)
    cap = command.add_parser("capture")
    cap.add_argument("--port", required=True)
    cap.add_argument("--baud", type=int, default=115200)
    cap.add_argument("--seconds", type=float, default=12.0)
    cap.add_argument("--out", type=Path, required=True)
    fit_cmd = command.add_parser("fit")
    fit_cmd.add_argument("log", type=Path)
    args = parser.parse_args()
    if args.command == "capture":
        capture(args.port, args.baud, args.seconds, args.out)
    else:
        report(args.log)


if __name__ == "__main__":
    main()
