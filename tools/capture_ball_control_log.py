"""采集 UART1 CTRL 遥测数据并生成 VOFA 风格的调参曲线图。"""
from __future__ import annotations

import argparse
import csv
import re
import time
from pathlib import Path

import serial


CTRL = re.compile(
    r"^CTRL M=(?P<mode>\d+) S=(?P<state>\d+) "
    r"X=(?P<x>[+-]\d+\.\d+) V=(?P<v>[+-]\d+\.\d+) "
    r"R=(?P<ref>[+-]\d+\.\d+) E=(?P<error>[+-]\d+\.\d+) "
    r"U=(?P<u>[+-]\d+\.\d+) P=(?P<pipe>[+-]\d+\.\d+) "
    r"Q=(?P<motor>[+-]\d+\.\d+) W=(?P<wheel>[+-]\d+\.\d+) "
    r"L=(?P<distance>[+-]\d+\.\d+) F=(?P<frames>\d+) C=(?P<confidence>\d+) "
    r"OK=(?P<valid>\d+) A=(?P<vision_age>\d+) MR=(?P<motor_rx>\d+)"
    r"(?: .*? MP=(?P<motor_actual>[+-]\d+\.\d+))?(?: .*)?$"
)
FAST = re.compile(
    r"^D,(?P<mode>\d+),(?P<state>\d+),(?P<x>[+-]\d+\.\d+),"
    r"(?P<v>[+-]\d+\.\d+),(?P<ref>[+-]\d+\.\d+),(?P<error>[+-]\d+\.\d+),"
    r"(?P<u>[+-]\d+\.\d+),(?P<pipe>[+-]\d+\.\d+),"
    r"(?P<motor>[+-]\d+\.\d+),(?P<motor_actual>[+-]\d+\.\d+)"
    r",(?P<frames>\d+),(?P<vision_age>\d+)"
    r",(?P<vision_bytes>\d+),(?P<vision_crc>\d+),(?P<vision_overflow>\d+)"
    r",(?P<vision_timestamp>\d+),(?P<vision_confidence>\d+)"
    r",(?P<vision_sequence>\d+),(?P<pulse_phase>\d+),(?P<raw_x>-?\d+)"
    r",(?P<vision_stable>\d+),(?P<valid>\d+)"
    r",(?P<vision_pixel_x>\d+),(?P<vision_pixel_y>\d+)"
    r",(?P<control_elapsed_ms>\d+),(?P<probe_start_ms>\d+)"
    r",(?P<probe_active>\d+),(?P<probe_complete>\d+)"
    r",(?P<setup_step>\d+),(?P<motor_options>\d+),(?P<motor_status>\d+)"
    r",(?P<motor_bytes>\d+),(?P<motor_frames>\d+),(?P<motor_fn>\d+),(?P<motor_reply>\d+),(?P<emm_rpm>[+-]\d+\.\d+)$"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM12")
    parser.add_argument("--seconds", type=float, default=35.0)
    parser.add_argument("--tag", default="latest")
    args = parser.parse_args()

    out_dir = Path("artifacts")
    out_dir.mkdir(exist_ok=True)
    safe_tag = re.sub(r"[^A-Za-z0-9_.-]", "_", args.tag)
    stem = "ball_control_capture" if safe_tag == "latest" else f"ball_control_{safe_tag}"
    csv_path = out_dir / f"{stem}.csv"
    plot_path = out_dir / f"{stem}.svg"
    samples: list[dict[str, float | int]] = []

    with serial.Serial(args.port, 115200, timeout=0.25) as port:
        port.reset_input_buffer()
        start = time.monotonic()
        while time.monotonic() - start < args.seconds:
            line = port.readline().decode("utf-8", errors="replace").strip()
            match = CTRL.match(line) or FAST.match(line)
            if not match:
                continue
            row: dict[str, float | int] = {"t_s": time.monotonic() - start,
                                            "ref": 0.0, "error": 0.0, "wheel": 0.0,
                                            "distance": 0.0, "frames": 0, "confidence": 0,
                                            "valid": -1, "vision_age": 0, "motor_rx": 0,
                                            "vision_bytes": 0, "vision_crc": 0,
                                            "vision_overflow": 0, "vision_timestamp": 0,
                                            "vision_confidence": 0, "vision_sequence": 0,
                                            "vision_stable": -1, "vision_pixel_x": -1,
                                            "vision_pixel_y": -1, "control_elapsed_ms": -1,
                                            "probe_start_ms": -1, "probe_active": -1,
                                            "probe_complete": -1,
                                            "setup_step": -1, "motor_options": -1,
                                            "motor_status": -1, "motor_bytes": -1,
                                            "motor_frames": -1, "motor_fn": -1,
                                            "motor_reply": -1,
                                            "pulse_phase": 0, "raw_x": 0.0,
                                            "emm_rpm": 0.0}
            for key, value in match.groupdict().items():
                if value is None:
                    row[key] = float("nan")
                else:
                    row[key] = int(value) if key in {"mode", "state", "frames", "confidence", "valid", "vision_age", "motor_rx", "vision_bytes", "vision_crc", "vision_overflow", "vision_timestamp", "vision_confidence", "vision_sequence", "vision_stable", "vision_pixel_x", "vision_pixel_y", "control_elapsed_ms", "probe_start_ms", "probe_active", "probe_complete", "setup_step", "motor_options", "motor_status", "motor_bytes", "motor_frames", "motor_fn", "motor_reply", "pulse_phase"} else float(value)
            samples.append(row)

    if not samples:
        print("No CTRL samples captured.")
        return 2

    fields = ["t_s", "mode", "state", "x", "v", "raw_x", "ref", "error", "u", "pipe", "motor", "motor_actual", "emm_rpm", "wheel", "distance", "frames", "confidence", "valid", "vision_age", "motor_rx", "vision_bytes", "vision_crc", "vision_overflow", "vision_timestamp", "vision_confidence", "vision_sequence", "vision_stable", "vision_pixel_x", "vision_pixel_y", "control_elapsed_ms", "probe_start_ms", "probe_active", "probe_complete", "setup_step", "motor_options", "motor_status", "motor_bytes", "motor_frames", "motor_fn", "motor_reply", "pulse_phase"]
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(samples)

    width, panel_h, margin = 1200, 220, 55
    total_h = panel_h * 3 + margin * 2
    t_max = max(float(row["t_s"]) for row in samples) or 1.0

    def polyline(key: str, top: float, scale: float, color: str,
                 width_px: float = 1.5, opacity: float = 1.0) -> str:
        points = []
        for row in samples:
            x = 55.0 + float(row["t_s"]) / t_max * (width - 80.0)
            y = top + panel_h / 2.0 - float(row[key]) * scale
            points.append(f"{x:.1f},{y:.1f}")
        return (f'<polyline fill="none" stroke="{color}" '
                f'stroke-width="{width_px}" opacity="{opacity}" '
                f'points="{" ".join(points)}"/>')

    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{total_h}" viewBox="0 0 {width} {total_h}">',
           '<rect width="100%" height="100%" fill="white"/>',
           '<style>text{font:14px sans-serif;fill:#24292f}.grid{stroke:#d0d7de;stroke-width:1}.limit{stroke:#cf222e;stroke-dasharray:4 4}</style>',
           '<text x="55" y="26" font-size="20">H problem ball-control telemetry</text>']
    for i, label in enumerate(["ball x/ref (mm)", "velocity/error (mm/s, mm)", "PD/pipe/motor (deg)"]):
        top = margin + i * panel_h
        mid = top + panel_h / 2.0
        svg.append(f'<line class="grid" x1="55" y1="{mid}" x2="{width - 25}" y2="{mid}"/>')
        svg.append(f'<rect x="55" y="{top}" width="{width - 80}" height="{panel_h}" fill="none" class="grid"/>')
        svg.append(f'<text x="62" y="{top + 18}">{label}</text>')
    top = margin
    for value in (-10, 10):
        y = top + panel_h / 2.0 - value * 1.65
        svg.append(f'<line class="limit" x1="55" y1="{y:.1f}" x2="{width - 25}" y2="{y:.1f}"/>')
    svg += [polyline("raw_x", margin, 1.65, "#8c959f", 1.0, 0.55),
            polyline("x", margin, 1.65, "#0969da", 2.0), polyline("ref", margin, 1.65, "#8250df"),
            polyline("v", margin + panel_h, 1.2, "#bf8700"), polyline("error", margin + panel_h, 1.2, "#cf222e"),
            polyline("u", margin + panel_h * 2, 110.0, "#1a7f37"), polyline("pipe", margin + panel_h * 2, 110.0, "#0550ae"), polyline("motor", margin + panel_h * 2, 18.0, "#953800"), polyline("motor_actual", margin + panel_h * 2, 18.0, "#cf222e"), polyline("emm_rpm", margin + panel_h * 2, 7.0, "#8250df"), '</svg>']
    plot_path.write_text("\n".join(svg), encoding="utf-8")

    x = [float(row["x"]) for row in samples]
    print(f"samples={len(samples)} peak_abs_x_mm={max(map(abs, x)):.1f} csv={csv_path} plot={plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
