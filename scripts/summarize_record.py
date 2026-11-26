#!/usr/bin/env python3
"""Summarize one recording directory produced by outdoor_nav.recorder.

Standard library only: reads the JSONL streams, prints a short report and can
export an aligned CSV for spreadsheet plotting.
"""

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path

COLUMNS = (
    "t", "mono", "task", "x", "y", "yaw", "odom_x", "odom_y",
    "v_cmd", "w_cmd", "v_safe", "w_safe", "v_out", "w_out",
    "v_actual", "w_actual", "gait", "blockers",
)


def rows(path):
    if not path.is_file():
        return []
    parsed = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            parsed.append(json.loads(line))
        except ValueError:
            continue  # A crash can leave one partial line.
    return parsed


def payload(record, key):
    value = record.get(key)
    return value if isinstance(value, dict) else {}


def distance(states):
    total, jumps = 0.0, 0
    previous = None
    for state in states:
        pose = payload(state, "pose")
        if not {"x", "y"} <= set(pose) or pose["x"] is None or pose["y"] is None:
            continue
        current = (pose["x"], pose["y"])
        if previous is not None:
            step = math.dist(previous, current)
            total += step
            if step > 0.5:
                jumps += 1
        previous = current
    return total, jumps


def tracked_speed(states):
    values = [
        payload(state, "actual").get("v")
        for state in states
        if payload(state, "actual").get("v") is not None
    ]
    return values


def yaw_rate_spread(states):
    rates, previous = [], None
    for state in states:
        yaw, mono = payload(state, "pose").get("yaw"), state.get("mono")
        if yaw is None or mono is None:
            previous = None
            continue
        if previous is not None:
            delta, elapsed = (yaw - previous[0] + math.pi) % (2 * math.pi) - math.pi, mono - previous[1]
            if 0.005 < elapsed < 0.5:
                rates.append(delta / elapsed)
        previous = (yaw, mono)
    if len(rates) < 3:
        return None
    return statistics.pstdev(rates)


def tracking_error(states):
    errors = []
    for state in states:
        if state.get("task") not in ("navigating", "replanning"):
            continue
        command, actual = payload(state, "cmd").get("v"), payload(state, "actual").get("v")
        if command is None or actual is None:
            continue
        errors.append(abs(command - actual))
    if not errors:
        return None, None
    return sum(errors) / len(errors), max(errors)


def blockers(states, events):
    counts = {}
    for event in events:
        if event.get("event") == "blockers":
            for reason in event.get("reasons") or []:
                counts[reason] = counts.get(reason, 0) + 1
    seen = set()
    for state in states:
        current = tuple(state.get("blockers") or ())
        if current and current not in seen:
            seen.add(current)
            for reason in current:
                counts.setdefault(reason, 0)
    return counts


def report(directory):
    meta = json.loads((directory / "meta.json").read_text()) if (directory / "meta.json").is_file() else {}
    try:
        result = json.loads((directory / "result.json").read_text())
    except (OSError, ValueError):
        result = None
    states = rows(directory / "state.jsonl")
    events = rows(directory / "events.jsonl")
    imu = rows(directory / "imu.jsonl")
    scans = rows(directory / "scan.jsonl")

    start = meta.get("started_at_unix_s") or (states[0]["t"] if states else None)
    end = result.get("finished_at_unix_s") if result else (states[-1]["t"] if states else None)
    duration = (end - start) if start and end else 0.0
    total, jumps = distance(states)
    speeds = tracked_speed(states)
    spread = yaw_rate_spread(states)
    mean_error, max_error = tracking_error(states)

    print("记录目录: " + str(directory))
    print("任务 ID : " + str(meta.get("task_id")))
    print("结束状态: " + (str(result.get("outcome")) + (" (" + str(result.get("reason")) + ")" if result.get("reason") else "") if result else "缺失 —— 任务异常退出"))
    print("时长    : %.1f 秒" % duration)
    print("轨迹长度: %.1f 米（定位跳变 %d 次）" % (total, jumps))
    if speeds:
        print("实际速度: 最大 %.2f m/s，平均 %.2f m/s" % (max(speeds), sum(speeds) / len(speeds)))
    if spread is not None:
        print("航向抖动: 角速度标准差 %.3f rad/s" % spread)
    if mean_error is not None:
        print("指令跟踪: 平均偏差 %.2f m/s，最大 %.2f m/s" % (mean_error, max_error))
    counts = blockers(states, events)
    if counts:
        top = sorted(counts.items(), key=lambda item: -item[1])[:5]
        print("停车原因: " + "，".join("%s(%d)" % pair for pair in top))
    print("样本    : state %d，imu %d，scan %d，events %d" % (len(states), len(imu), len(scans), len(events)))
    if result:
        print("写入    : %.1f MB，丢行 %d%s" % (
            (result.get("bytes") or 0) / 1048576,
            result.get("dropped") or 0,
            "，已触发单次上限" if result.get("capped") else "",
        ))
    return states


def export(states, path):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(COLUMNS)
        for state in states:
            pose, odom = payload(state, "pose"), payload(state, "odom")
            command, safe = payload(state, "cmd"), payload(state, "safe")
            out, actual = payload(state, "out"), payload(state, "actual")
            chassis = payload(state, "chassis")
            writer.writerow([
                state.get("t"), state.get("mono"), state.get("task"),
                pose.get("x"), pose.get("y"), pose.get("yaw"),
                odom.get("x"), odom.get("y"),
                command.get("v"), command.get("w"),
                safe.get("v"), safe.get("w"),
                out.get("v"), out.get("w"),
                actual.get("v"), actual.get("w"),
                chassis.get("gait"),
                "|".join(state.get("blockers") or []),
            ])


def runs(path):
    if (path / "state.jsonl").is_file() or (path / "meta.json").is_file():
        return [path]
    return sorted(p for p in path.iterdir() if p.is_dir())


def main():
    parser = argparse.ArgumentParser(description="Summarize outdoor navigation recordings")
    parser.add_argument("path", help="recording directory or a parent of recordings")
    parser.add_argument("--csv", help="write an aligned CSV next to the report")
    args = parser.parse_args()
    path = Path(args.path)
    if not path.is_dir():
        parser.error("not a directory: " + str(path))
    found = runs(path)
    if not found:
        parser.error("no recordings under " + str(path))
    states = []
    for directory in found:
        if len(found) > 1:
            print("=" * 60)
        states = report(directory)
        if args.csv:
            target = Path(args.csv)
            if len(found) > 1:
                target = target.with_name(target.stem + "_" + directory.name + target.suffix)
            export(states, target)
            print("CSV     : " + str(target))
    return 0


if __name__ == "__main__":
    sys.exit(main())
