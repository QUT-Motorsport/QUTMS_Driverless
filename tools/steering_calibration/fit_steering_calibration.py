#!/usr/bin/env python3
"""fit steering centre, gain and slack from a steering calibration bag

    python3 tools/steering_calibration/fit_steering_calibration.py bags/steering_calibration-<date>
 steering_calibration.yaml"""

import argparse
import math
import os

import numpy as np
import yaml

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ["/imu/data", "/imu/odometry", "/control/driving_command", "/calibration/run", "/vehicle/stepper_request"]


def read(path):
    reader = rosbag2_py.SequentialReader()
    if os.path.isdir(path) and not os.path.exists(os.path.join(path, "metadata.yaml")):
        files = sorted(f for f in os.listdir(path) if f.endswith(".mcap"))
        if len(files) == 1:
            path = os.path.join(path, files[0])
    storage = "mcap" if path.endswith(".mcap") else ""
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in TOPICS[:4] if t not in types]
    if missing:
        raise SystemExit(f"bag is missing {missing}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    out = {t: [] for t in TOPICS}
    pose = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        m = deserialize_message(data, get_message(types[topic]))
        t = stamp * 1e-9
        if topic == "/imu/data":
            out[topic].append((t, m.angular_velocity.z))
        elif topic == "/imu/odometry":
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            v = m.twist.twist.linear
            out[topic].append((t, math.cos(yaw) * v.x + math.sin(yaw) * v.y))
            pose.append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
        elif topic == "/control/driving_command":
            out[topic].append((t, m.drive.steering_angle))
        else:
            out[topic].append((t, m.data))
    out = {k: np.array(v, dtype=float).reshape(-1, 2) for k, v in out.items()}
    out["pose"] = np.array(pose, dtype=float).reshape(-1, 4)
    return out


def runs(d, min_speed, settle):
    """curvature for each run while pushed forward"""
    gyro, speed, cmd, run = d["/imu/data"], d["/imu/odometry"], d["/control/driving_command"], d["/calibration/run"]
    t = gyro[:, 0]
    v = np.interp(t, speed[:, 0], speed[:, 1])
    idx = np.searchsorted(run[:, 0], t, side="right") - 1
    ok = (idx >= 0) & (t <= run[-1, 0]) & (t <= speed[-1, 0]) & (t >= speed[0, 0])
    run_id = np.where(idx >= 0, run[np.maximum(idx, 0), 1], -1)
    # wait for the steering to reach each new command
    change = run[np.r_[True, np.diff(run[:, 1]) != 0], 0]
    since = t - change[np.maximum(np.searchsorted(change, t, side="right") - 1, 0)]
    ok &= (v > min_speed) & (since > settle)
    dt = np.r_[np.diff(t), 0.0]
    out = []
    for r in np.unique(run_id[run_id >= 0]).astype(int):
        span = run[run[:, 1] == r, 0]
        c = cmd[(cmd[:, 0] >= span[0]) & (cmd[:, 0] <= span[-1]), 1]
        steps = d["/vehicle/stepper_request"]
        s = steps[(steps[:, 0] >= span[0]) & (steps[:, 0] <= span[-1]), 1] if len(steps) else []
        # the command this run came from, the stop signal lock after the first run
        before = cmd[(cmd[:, 0] < span[0]) & (np.abs(cmd[:, 1] - np.median(c)) > 1e-6), 1] if len(c) else []
        sel = ok & (run_id == r)
        k = gyro[sel, 1] / v[sel]
        out.append({
            "run": int(r) + 1,
            "command": float(np.median(c)) if len(c) else None,
            "previous_command": float(before[-1]) if len(before) else None,
            "stepper": float(np.median(s)) if len(s) else None,
            "samples": int(sel.sum()),
            "distance_m": float(np.sum(v[sel] * dt[sel])),
            "curvature": float(np.median(k)) if sel.any() else None,
            "curvature_iqr": float(np.subtract(*np.percentile(k, [75, 25]))) if sel.sum() > 3 else None,
        })
    return out


def fit(rows, span):
    """curvature = gain * command + intercept + hysteresis * approach direction"""
    for r in rows:
        prev = r["previous_command"]
        r["approach"] = 0 if prev is None or r["command"] is None else int(np.sign(r["command"] - prev))
    good = [r for r in rows if r["curvature"] is not None and r["command"] is not None]
    if len(good) < 4:
        raise SystemExit(f"only {len(good)} usable runs, need at least 4")
    x = np.array([r["command"] for r in good])
    k = np.array([r["curvature"] for r in good])
    a, b = np.polyfit(x, k, 1)
    centre = -b / a
    near = np.abs(x - centre) <= span
    if near.sum() < 4:
        near = np.ones(len(x), dtype=bool)
    d = np.array([r["approach"] for r in good])
    A = np.c_[x, np.ones(len(x)), d][near]
    (gain, intercept, hyst), *_ = np.linalg.lstsq(A, k[near], rcond=None)
    if not np.any(d[near]):
        hyst = 0.0
    resid = k - (gain * x + intercept + hyst * d)
    for r, e in zip(good, resid):
        r["residual"] = float(e)
        r["in_fit"] = bool(near[good.index(r)])
    centre = -intercept / gain
    return {
        "centre_command": float(centre),
        "gain_curvature_per_command": float(gain),
        "slack_command": float(abs(2 * hyst / gain)),
        "hysteresis_curvature": float(hyst),
        "fit_runs": int(near.sum()),
        "fit_rms_curvature": float(np.sqrt(np.mean(resid[near] ** 2))),
        "fit_span_command": span,
    }


def repeat_check(rows, cal):
    """same command run twice, curvature change as a centre shift after taking out slack"""
    by = {}
    for r in rows:
        if r["curvature"] is not None and r.get("approach", 0) != 0:
            by.setdefault(r["command"], []).append(r)
    out = []
    for c, rs in by.items():
        if len(rs) > 1:
            k = [r["curvature"] - cal["hysteresis_curvature"] * r.get("approach", 0) for r in (rs[0], rs[-1])]
            shift = (k[1] - k[0]) / cal["gain_curvature_per_command"]
            out.append({"command": c, "runs": [r["run"] for r in rs], "centre_shift_command": float(shift)})
    return out


def centre_stepper(rows, cal):
    """stepper target at the centre, from the recorded requests"""
    pts = [(r["command"], r["stepper"]) for r in rows if r["stepper"] is not None]
    if len(pts) < 2:
        return None
    x, s = np.array(pts).T
    m, c = np.polyfit(x, s, 1)
    return float(m * cal["centre_command"] + c)


def plot(d, rows, cal, path):
    """each run's path coloured by steering command, and all runs from a common start"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm

    surface, text, text2, grid, muted = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df", "#c3c2b7"
    plt.rcParams.update({"figure.facecolor": surface, "axes.facecolor": surface, "axes.edgecolor": grid,
                         "axes.labelcolor": text2, "xtick.color": text2, "ytick.color": text2, "text.color": text,
                         "axes.grid": True, "grid.color": grid, "grid.linewidth": 0.6, "font.size": 9})
    # blue left of centre, grey at centre, red right of centre
    cmap = LinearSegmentedColormap.from_list("steer", ["#2a78d6", "#8a8984", "#e34948"])
    cmds = [r["command"] for r in rows if r["command"] is not None]
    c0 = cal["centre_command"]
    span = max(abs(min(cmds) - c0), abs(max(cmds) - c0), 1.0)
    norm = TwoSlopeNorm(vcenter=c0, vmin=c0 - span, vmax=c0 + span)

    pose, run = d["pose"], d["/calibration/run"]
    idx = np.searchsorted(run[:, 0], pose[:, 0], side="right") - 1
    rid = np.where(idx >= 0, run[np.maximum(idx, 0), 1], -1)
    fig = plt.figure(figsize=(13, 9))
    gs = fig.add_gridspec(2, 2, height_ratios=[2.2, 1])
    a1, a2, a3 = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, :])
    a1.plot(pose[:, 1], pose[:, 2], color=muted, lw=0.8, zorder=1, label="stop signal and wheel back")
    for r in rows:
        sel = rid == r["run"] - 1
        if not sel.any() or r["command"] is None:
            continue
        x, y, yaw = pose[sel, 1], pose[sel, 2], pose[sel, 3]
        col = cmap(norm(r["command"]))
        a1.plot(x, y, color=col, lw=2, zorder=2, solid_capstyle="round")
        a1.annotate(str(r["run"]), (x[-1], y[-1]), xytext=(4, 2), textcoords="offset points", color=text2, fontsize=8)
        c, s = math.cos(-yaw[0]), math.sin(-yaw[0])
        lx, ly = c * (x - x[0]) - s * (y - y[0]), s * (x - x[0]) + c * (y - y[0])
        a2.plot(lx, ly, color=col, lw=2, solid_capstyle="round")
        a2.annotate(f"{r['run']}: {r['command']:.0f}", (lx[-1], ly[-1]), xytext=(4, 0), textcoords="offset points",
                    color=text2, fontsize=8)
    a1.set_title("Runs as driven", loc="left", color=text)
    a1.legend(loc="best", frameon=False)
    a2.set_title("All runs from the same start, facing right", loc="left", color=text)
    for a in (a1, a2):
        a.set_aspect("equal", adjustable="datalim")
        a.set_xlabel("x (m)")
        a.set_ylabel("y (m)")
    cmd = d["/control/driving_command"]
    t0 = cmd[0, 0]
    stopped = run[:, 1] < 0
    edges = np.flatnonzero(np.diff(np.r_[0, stopped.astype(int), 0]))
    for i, (a, b) in enumerate(zip(edges[::2], edges[1::2])):
        a3.axvspan(run[a, 0] - t0, run[b - 1, 0] - t0, color=grid, lw=0, label="stop signal and wheel back" if i == 0 else None)
    a3.plot(cmd[:, 0] - t0, cmd[:, 1], color=text2, lw=1.2)
    for r in rows:
        span = run[run[:, 1] == r["run"] - 1, 0]
        if len(span) and r["command"] is not None:
            a3.plot([span[0] - t0, span[-1] - t0], [r["command"]] * 2, color=cmap(norm(r["command"])), lw=4,
                    solid_capstyle="round")
            a3.annotate(str(r["run"]), (span[0] - t0, r["command"]), xytext=(0, 5), textcoords="offset points",
                        color=text2, fontsize=8)
    a3.axhline(c0, color=text, lw=0.8, ls=":")
    a3.set_title("Steering command over time: runs coloured, stop signal lock between them", loc="left", color=text)
    a3.set_xlabel("time (s)")
    a3.set_ylabel("steering command")
    a3.legend(loc="upper left", frameon=False)
    sm = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
    bar = fig.colorbar(sm, ax=[a1, a2], shrink=0.8, pad=0.02)
    bar.set_label("steering command (grey = fitted centre)")
    bar.ax.axhline(c0, color=text, lw=1)
    fig.suptitle(f"Steering calibration: centre {c0:.1f}, gain {cal['gain_curvature_per_command']:.5f} 1/m per command, "
                 f"slack {cal['slack_command']:.1f}", x=0.02, ha="left", color=text)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    print(f"wrote {path}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bag")
    p.add_argument("--out", help="output yaml")
    p.add_argument("--min-speed", type=float, default=0.5, help="m/s, slower samples are ignored")
    p.add_argument("--settle", type=float, default=4.0, help="s after a step before samples count")
    p.add_argument("--span", type=float, default=45.0, help="command range either side of centre for the linear fit")
    p.add_argument("--plot", nargs="?", const="", help="save a path image, next to the output yaml by default")
    args = p.parse_args()

    d = read(args.bag)
    rows = runs(d, args.min_speed, args.settle)
    cal = fit(rows, args.span)
    cal["centre_stepper"] = centre_stepper(rows, cal)
    checks = repeat_check(rows, cal)

    print(f"{'run':>4} {'command':>8} {'stepper':>8} {'samples':>8} {'dist m':>7} {'curv 1/m':>9} {'resid':>7} {'dir':>4}")
    for r in rows:
        fmt = lambda v, f: "-" if v is None else format(v, f)
        print(f"{r['run']:>4} {fmt(r['command'], '.0f'):>8} {fmt(r['stepper'], '.0f'):>8} {r['samples']:>8} "
              f"{r['distance_m']:>7.1f} {fmt(r['curvature'], '.4f'):>9} {fmt(r.get('residual'), '.4f'):>7} {r.get('approach', 0):>4}"
              f"{'' if r.get('in_fit', True) else '  (outside fit)'}")
    print(f"\ncentre: {cal['centre_command']:.1f} command"
          + (f", stepper {cal['centre_stepper']:.0f}" if cal["centre_stepper"] is not None else ""))
    print(f"gain: {cal['gain_curvature_per_command']:.5f} 1/m per command")
    print(f"slack: {cal['slack_command']:.1f} command")
    print(f"fit rms: {cal['fit_rms_curvature']:.4f} 1/m over {cal['fit_runs']} runs")
    for c in checks:
        print(f"repeat at {c['command']:.0f} (runs {c['runs']}): centre moved {c['centre_shift_command']:+.1f} command")

    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.bag.rstrip("/"))), "steering_calibration.yaml")
    with open(out, "w") as f:
        yaml.safe_dump({"bag": os.path.abspath(args.bag), "calibration": cal, "repeat_checks": checks, "runs": rows}, f, sort_keys=False)
    print(f"\nwrote {out}")
    if args.plot is not None:
        plot(d, rows, cal, args.plot or os.path.splitext(out)[0] + ".png")


if __name__ == "__main__":
    main()
