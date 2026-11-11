#!/usr/bin/env python3
'''Q6 台架日志离线分析工具（分析 DebugLog 输出的 Q45 / Q5D 行）。

按 run= 字段分组，每组输出：锁定的目标、原始偏差峰值（tmax10）、
评分带越界计数（|dev| > 10 即 ±1cm）、视觉帧覆盖率，以及终止状态/停止原因。

用法：python tools/analyze_q6_log.py artifacts/q6_run.csv [more.csv ...]
非 Q45/Q5D 开头的行会被忽略，因此可直接喂入原始串口采集数据。
'''
import re
import sys

Q45_RE = re.compile(r'^Q45\s+(.*)$')
FIELD_RE = re.compile(r'(\w+)=(-?\d+)')


def parse_fields(blob):
    out = {}
    for key, value in FIELD_RE.findall(blob):
        out[key] = int(value)
    return out


def analyze(path):
    runs = {}
    order = []
    with open(path, encoding='utf-8', errors='replace') as handle:
        for raw_line in handle:
            m = Q45_RE.match(raw_line.strip())
            if not m:
                continue
            f = parse_fields(m.group(1))
            if 'run' not in f:
                continue
            run = f['run']
            if run not in runs:
                runs[run] = []
                order.append(run)
            runs[run].append(f)

    print('== %s ==' % path)
    if not runs:
        print('  no Q45 records found')
        return
    for run in order:
        rows = runs[run]
        first, last = rows[0], rows[-1]
        target = last.get('tgt10', 0) / 10.0
        peak_dev = max((r.get('dev10', 0) for r in rows), default=0) / 10.0
        reported_peak = last.get('tmax10', 0) / 10.0
        dev_abs = [abs(r.get('dev10', 0)) for r in rows]
        in_band = sum(1 for d in dev_abs if d <= 10)
        over = len(rows) - in_band
        conf_min = min((r.get('conf', 0) for r in rows), default=0)
        max_age = max((r.get('age', 0) for r in rows), default=0)
        print('  run %d  stop=%s why=%s fault=%s' % (
            run, last.get('stop', 0), last.get('why', 0), last.get('fault', 0)))
        print('    target=%+.1fcm  peak_dev=%.1fcm  tmax10=%.1fcm' % (
            target, peak_dev, reported_peak))
        print('    frames=%d  in_band=%d  over_1cm=%d (%.0f%%)  conf_min=%d' % (
            len(rows), in_band, over,
            (100.0 * over / len(rows)) if rows else 0.0, conf_min))
        print('    elapsed=%.1fs  dist=%.1fcm  max_age=%dms' % (
            last.get('elapsed', 0) / 1000.0,
            last.get('dist10', 0) / 10.0, max_age))
    return runs


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    for path in argv[1:]:
        analyze(path)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
