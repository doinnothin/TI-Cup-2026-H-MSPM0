#!/usr/bin/env python3
'''Q6 台架日志的 VOFA 风格多通道绘图工具（Q45 行）。

每次运行绘制子图：目标偏差（含 ±1cm 评分带与 ±0.5cm 台架判据阴影）、
指令角与实际执行器角、视觉置信度、视觉帧龄。输出：<capture>_run<N>.svg。

用法：python tools/plot_q6_trial.py artifacts/q6_auto_static_trial1.txt [...]
'''
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FIELD_RE = re.compile(r"(\w+)=(-?\d+)")


def load(path):
    runs = {}
    order = []
    with open(path, encoding="utf-8-sig", errors="replace") as handle:
        for line in handle:
            line = line.strip()
            if not line.startswith("Q45 "):
                continue
            f = dict(FIELD_RE.findall(line[4:]))
            if "run" not in f or "t" not in f:
                continue
            run = int(f["run"])
            if run not in runs:
                runs[run] = []
                order.append(run)
            runs[run].append({k: int(v) for k, v in f.items()})
    return runs, order


def plot(path, run, frames):
    t0 = frames[0]["t"]
    t = [(f["t"] - t0) / 1000.0 for f in frames]
    dev = [f.get("dev10", 0) / 10.0 for f in frames]
    cmd = [f.get("cmd10", 0) / 10.0 for f in frames]
    act = [f.get("act10", 0) / 10.0 for f in frames]
    conf = [f.get("conf", 0) for f in frames]
    age = [f.get("age", 0) for f in frames]

    fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    fig.suptitle("Q6 static hold  " + path + "  run=" + str(run))

    a = ax[0]
    a.plot(t, dev, lw=1, color="tab:blue")
    a.axhline(1.0, color="red", ls="--", lw=0.8)
    a.axhline(-1.0, color="red", ls="--", lw=0.8)
    a.axhspan(-0.5, 0.5, color="green", alpha=0.15)
    a.set_ylabel("dev (cm)")
    a.set_title("target deviation (+-1cm band red, +-0.5cm bench green)")

    a = ax[1]
    a.plot(t, cmd, lw=1, color="tab:orange", label="cmd")
    a.plot(t, act, lw=1, color="tab:green", label="act")
    a.legend(loc="upper right", fontsize=8)
    a.set_ylabel("angle (deg)")
    a.set_title("actuator command vs actual")

    a = ax[2]
    a.plot(t, conf, lw=1, color="tab:purple")
    a.set_ylabel("conf (%)")
    a.set_title("vision confidence")

    a = ax[3]
    a.plot(t, age, lw=1, color="tab:gray")
    a.set_ylabel("age (ms)")
    a.set_xlabel("time (s) since first Q45 frame")
    a.set_title("vision frame age")

    out = "{}_run{}.svg".format(path[:-4] if path.lower().endswith(".txt") else path, run)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    print("wrote", out)


def main():
    for path in sys.argv[1:]:
        runs, order = load(path)
        for run in order:
            plot(path, run, runs[run])


if __name__ == "__main__":
    main()
