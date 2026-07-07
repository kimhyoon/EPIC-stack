#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_session.py — EPIC 비행 세션 시각화 리포트 생성기

records/<세션>/ 폴더(events.log + [선택] bag)를 읽어 <세션>/plots/*.png 와
<세션>/report.html 을 생성한다. 로그 수집은 그대로 두고, 사람이 보기 좋은
그림으로 후처리하는 도구.

사용:
    plot_session.py <세션폴더>          # 특정 세션
    plot_session.py                     # <epic_planner>/records 의 최신 세션
    plot_session.py --all               # 전 세션 일괄

생성물:
    plots/timeline.png            FSM 상태/PX4 모드/회피 밴드 + 이벤트 마커 (한눈 요약)
    plots/trajectory_xy.png       XY 비행 경로 (시간 색상, 시작/끝/홈/STUCK 표시)
    plots/altitude_speed.png      고도 z(t) + 속도(t) + 상태 전이선
    plots/frontier_pipeline.png   클러스터/뷰포인트 수 + 파이프라인 단계별 탈락 + 도달가능 비율
    plots/plan_health.png         GLOBAL/LOCAL 결과 타임라인 + 계획 소요시간
    plots/rth_progress.png        홈까지 거리 (RTH 구간 있을 때)
    plots/battery.png             전압/잔량
    plots/tracking_error.png      |odom - 명령| 추종 오차 (bag 있을 때)
    report.html                   위 전부 + 요약 표를 한 페이지로

bag 이 없으면 bag 기반 플롯(경로/고도/추종오차)은 생략하고 나머지만 생성.
"""

import os
import re
import sys
import glob
import math
import html
import shutil
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.dates  # noqa
from matplotlib.patches import Patch

# ---------------------------------------------------------------- 유틸

STATE_COLORS = {
    "INIT": "#bdbdbd", "WAIT_TRIGGER": "#90a4ae", "TAKEOFF_HOVER": "#4fc3f7",
    "PLAN_TRAJ_EXP": "#66bb6a", "EXEC_TRAJ": "#43a047", "CAUTION": "#ff7043",
    "FINISH": "#ffd54f", "PLAN_TRAJ_RTH": "#7e57c2", "LAND": "#8d6e63",
    "LANDED": "#455a64",
}
PX4_COLORS = {
    "OFFBOARD": "#2e7d32", "POSCTL": "#ef6c00", "AUTO.LAND": "#6a1b9a",
    "ALTCTL": "#f9a825", "STABILIZED": "#c62828", "MANUAL": "#c62828",
}

# 구버전 축약 로그도 읽을 수 있게 정규화
OLD2NEW = [
    ("pipe[tot=", "pipeline[total="), (" dorm=", " dormant="),
    (" unreach=", " prev_unreachable="), (" cons=", " evaluated="),
    (" nocand=", " no_candidate="), (" topo=", " topo_unreachable="),
    (" novis=", " no_visibility="), (" ok=", " survived="),
    ("(reach ", "(reachable "), ("vp=", "viewpoints="),
    ("d2goal=", "dist_to_goal="), ("goal_d=", "goal_dist="),
    ("v=", "voltage="), ("pct=", "percent="),
]

LINE_RE = re.compile(r"^\[(\d+\.\d+)\]\[\s*([+-]?\d+\.?\d*)s\]\[(\w+)\]\s+(\S+)\s*(.*)$")


def normalize(line):
    for a, b in OLD2NEW:
        line = line.replace(a, b)
    line = re.sub(r"\bEXP (OK|FAIL|START_FAIL|NO_FRONTIER)", r"explore \1", line)
    line = re.sub(r" t=(\d+\.?\d*)ms", r" plan_time=\1ms", line)
    line = re.sub(r"tour=(\d+)n/", r"tour=\1_nodes/", line)
    return line


def fnum(pattern, s, default=None):
    m = re.search(pattern, s)
    return float(m.group(1)) if m else default


def parse_events(path):
    """events.log -> dict of series (시간축 = epoch 초)"""
    ev = {
        "state": [],      # (t, from, to, reason)
        "global": [],     # (t, result, dict)
        "local": [],      # (t, mode, result, plan_ms, goal_dist, why)
        "rth": [],        # (t, dist)
        "batt": [],       # (t, voltage, percent)
        "px4": [],        # (t, mode, armed)
        "avoid": [],      # (t, 1/0)
        "stuck": [],      # (t, x, y, z)
        "mission": [],    # (t, text)
        "first_state": None,
        "t0": None, "t_end": None, "trigger_t": None,
    }
    if not os.path.exists(path):
        return ev
    with open(path, errors="replace") as f:
        for raw in f:
            line = normalize(raw.rstrip("\n"))
            m = LINE_RE.match(line)
            if not m:
                continue
            t = float(m.group(1))
            state, cat, rest = m.group(3), m.group(4), m.group(5)
            if ev["t0"] is None:
                ev["t0"] = t
                ev["first_state"] = state
            ev["t_end"] = t

            if cat == "STATE":
                sm = re.match(r"(\w+) -> (\w+)\s*\[(.*?)\]", rest)
                if sm:
                    ev["state"].append((t, sm.group(1), sm.group(2), sm.group(3)))
            elif cat == "GLOBAL":
                d = {
                    "result": re.search(r"result=(\S+)", rest).group(1) if "result=" in rest else "?",
                    "clusters": fnum(r"clusters=(\d+)", rest),
                    "clusters_reach": fnum(r"clusters=\d+\(reachable (\d+)\)", rest),
                    "vps": fnum(r"viewpoints=(\d+)", rest),
                    "vps_reach": fnum(r"viewpoints=\d+\(path_reachable (\d+)\)", rest),
                    "total": fnum(r"total=(\d+)", rest),
                    "dormant": fnum(r"dormant=(\d+)", rest),
                    "prev_unreachable": fnum(r"prev_unreachable=(\d+)", rest),
                    "evaluated": fnum(r"evaluated=(\d+)", rest),
                    "no_candidate": fnum(r"no_candidate=(\d+)", rest),
                    "topo_unreachable": fnum(r"topo_unreachable=(\d+)", rest),
                    "no_visibility": fnum(r"no_visibility=(\d+)", rest),
                    "survived": fnum(r"survived=(\d+)", rest),
                    "tour_len": fnum(r"tour=\d+_nodes/([\d.]+)m", rest),
                    "plan_ms": fnum(r"plan_time=([\d.]+)ms", rest),
                }
                ev["global"].append((t, d["result"], d))
            elif cat == "LOCAL":
                lm = re.match(r"(explore|RTH)\s+(\S+)", rest)
                why = ""
                wm = re.search(r"why:\s*(.*?)(?:\s*\||$)", rest)
                if wm:
                    why = wm.group(1)
                if lm:
                    ev["local"].append((t, lm.group(1), lm.group(2),
                                        fnum(r"plan_time=([\d.]+)ms", rest),
                                        fnum(r"goal_dist=([\d.]+)m", rest), why))
            elif cat == "RTH":
                d = fnum(r"dist=([\d.]+)m", rest)
                if d is not None:
                    ev["rth"].append((t, d))
            elif cat == "BATT":
                ev["batt"].append((t, fnum(r"voltage=([\d.]+)V", rest),
                                   fnum(r"percent=([\-\d.]+)", rest)))
            elif cat == "PX4":
                mm = re.search(r"mode=(\S+?)\s+armed=(\d)", rest)
                if mm:
                    ev["px4"].append((t, mm.group(1), int(mm.group(2))))
            elif cat in ("Avoidance:", "AVOID"):
                if rest.startswith(("Activated", "ON")):
                    ev["avoid"].append((t, 1))
                elif rest.startswith(("Deactivated", "OFF")):
                    ev["avoid"].append((t, 0))
            elif cat == "STUCK":
                pm = re.search(r"pos=\(([\-\d.]+), ([\-\d.]+), ([\-\d.]+)\)", rest)
                if pm:
                    ev["stuck"].append((t, float(pm.group(1)), float(pm.group(2)),
                                        float(pm.group(3))))
            elif cat == "MISSION":
                ev["mission"].append((t, rest))
            elif cat == "EVENT" and "trigger" in rest:
                ev["trigger_t"] = t
    return ev


# 로컬 계획 연산시간 토픽 (존재하는 것만 자동 수집; Float32/Float64, 단위 ms)
PLAN_TIME_TOPICS = [
    # 탐색(그래프/A*) 계열
    "/local_planning/topo_graph_search_cost",
    "/local_planning/bubble_astar_search_cost",
    "/planning/timing/fast_searcher_search_cost",
    "/planning/timing/bubble_astar_search_cost",
    # MINCO 파이프라인 단계별 (성공한 로컬 플랜 1회당 1개씩)
    "/visualizer/pointCloudProcess_timecost",
    "/visualizer/PolysGenerate_timecost",
    "/visualizer/trajOptimize_timecost",
    "/visualizer/totoalOptimize_timecost",
    "/visualizer/trajectory_generation_cost",
    "/visualizer/lbfgs_optimization_cost",
    "/visualizer/yaw_trajectory_optimization_cost",
    # 체크 계열
    "/visualizer/collision_check_cost",
    "/visualizer/velocity_check_cost",
]


def read_bag(bag_path):
    """bag -> odom/명령/회피/계획시간 시계열 (필요 토픽만, 가벼움)"""
    try:
        import rosbag
    except ImportError:
        return None
    if not os.path.exists(bag_path):
        return None
    data = {"odom": [], "cmd": [], "avoid": [], "timing": {}}
    topics = ["/Odometry", "/position_cmd", "/FSM_flag_avoidance"] + PLAN_TIME_TOPICS
    try:
        with rosbag.Bag(bag_path) as bag:
            for topic, msg, t in bag.read_messages(topics=topics):
                ts = t.to_sec()
                if topic == "/Odometry":
                    p = msg.pose.pose.position
                    data["odom"].append((ts, p.x, p.y, p.z))
                elif topic == "/position_cmd":
                    data["cmd"].append((ts, msg.position.x, msg.position.y,
                                        msg.position.z))
                elif topic == "/FSM_flag_avoidance":
                    data["avoid"].append((ts, int(msg.data)))
                else:  # timing topics (std_msgs Float32/Float64 -> .data)
                    data["timing"].setdefault(topic, []).append((ts, float(msg.data)))
    except Exception as e:
        print("[plot_session] bag read failed:", e)
        return None
    return data


# ---------------------------------------------------------------- 플롯들

def _rel(ts, t0):
    return [t - t0 for t in ts]


def _state_bands(ev):
    """STATE 이벤트 -> [(t시작, t끝, 상태)] 밴드"""
    bands = []
    if not ev["state"]:
        return bands
    t0 = ev["t0"]
    cur = ev["state"][0][1] or ev["first_state"]
    start = t0
    for (t, s_from, s_to, _r) in ev["state"]:
        if s_to != cur:
            bands.append((start, t, cur))
            cur, start = s_to, t
    bands.append((start, ev["t_end"], cur))
    return bands


def plot_timeline(ev, out):
    fig, ax = plt.subplots(figsize=(12, 3.6))
    t0 = ev["t0"]
    # FSM 상태 밴드
    for (a, b, s) in _state_bands(ev):
        ax.barh(2, b - a, left=a - t0, height=0.8,
                color=STATE_COLORS.get(s, "#ccc"), edgecolor="none")
    # PX4 모드 밴드
    px = ev["px4"]
    for i, (t, mode, armed) in enumerate(px):
        t_next = px[i + 1][0] if i + 1 < len(px) else ev["t_end"]
        ax.barh(1, t_next - t, left=t - t0, height=0.8,
                color=PX4_COLORS.get(mode, "#999"),
                alpha=1.0 if armed else 0.35, edgecolor="none")
    # 회피 밴드
    av = ev["avoid"]
    for i, (t, on) in enumerate(av):
        if on:
            t_next = next((tt for (tt, o) in av[i + 1:] if o == 0), ev["t_end"])
            ax.barh(0, t_next - t, left=t - t0, height=0.8, color="#e53935")
    # 마커
    for (t, txt) in ev["mission"]:
        ax.axvline(t - t0, color="k", ls="--", lw=0.8)
        ax.text(t - t0, 2.55, txt.split("|")[0].strip()[:28], rotation=0,
                fontsize=7, ha="left", va="bottom")
    for (t, *_p) in ev["stuck"]:
        ax.plot(t - t0, 2, marker="x", color="red", ms=8, mew=2)

    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["Avoidance", "PX4 mode", "FSM state"])
    ax.set_xlabel("time since session start [s]")
    ax.set_title("Session timeline")
    ax.set_ylim(-0.6, 3.2)
    handles = [Patch(color=c, label=s) for s, c in STATE_COLORS.items()
               if any(b[2] == s for b in _state_bands(ev))]
    handles += [Patch(color=c, label=m) for m, c in PX4_COLORS.items()
                if any(p[1] == m for p in ev["px4"])]
    handles.append(Patch(color="#e53935", label="Avoidance ON"))
    ax.legend(handles=handles, loc="upper right", fontsize=7, ncol=4)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def plot_trajectory(bagdata, ev, out):
    od = bagdata["odom"]
    if len(od) < 2:
        return False
    t0 = od[0][0]
    xs = [o[1] for o in od]
    ys = [o[2] for o in od]
    ts = [o[0] - t0 for o in od]
    fig, ax = plt.subplots(figsize=(7.5, 7))
    sc = ax.scatter(xs, ys, c=ts, cmap="viridis", s=4)
    fig.colorbar(sc, ax=ax, label="time [s]")
    ax.plot(xs[0], ys[0], "g^", ms=12, label="start")
    ax.plot(xs[-1], ys[-1], "rX", ms=12, label="end")
    ax.plot(0, 0, "k*", ms=14, label="home(0,0)")
    for (t, x, y, z) in ev["stuck"]:
        ax.plot(x, y, "rx", ms=10, mew=2)
    if ev["stuck"]:
        ax.plot([], [], "rx", label="STUCK")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Flight path (XY)")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return True


def plot_altitude_speed(bagdata, ev, out):
    od = bagdata["odom"]
    if len(od) < 3:
        return False
    t0 = od[0][0]
    ts = [o[0] - t0 for o in od]
    zs = [o[3] for o in od]
    spd = [0.0]
    for i in range(1, len(od)):
        dt = od[i][0] - od[i - 1][0]
        d = math.dist(od[i][1:4], od[i - 1][1:4]) if dt > 0 else 0
        spd.append(d / dt if dt > 0 else 0)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 5.5), sharex=True)
    ax1.plot(ts, zs, lw=1.2)
    ax1.set_ylabel("altitude z [m]")
    ax1.grid(alpha=0.3)
    ax2.plot(ts, spd, lw=0.8, color="#ef6c00")
    ax2.set_ylabel("speed [m/s]")
    ax2.set_xlabel("time [s]")
    ax2.grid(alpha=0.3)
    for (t, _f, s_to, _r) in ev["state"]:
        if s_to in ("TAKEOFF_HOVER", "PLAN_TRAJ_EXP", "FINISH", "PLAN_TRAJ_RTH",
                    "LAND", "LANDED", "CAUTION"):
            for ax in (ax1, ax2):
                ax.axvline(t - t0, color=STATE_COLORS.get(s_to, "k"), ls="--", lw=0.9)
            ax1.text(t - t0, ax1.get_ylim()[1], s_to, rotation=90, fontsize=6,
                     va="top", ha="right")
    ax1.set_title("Altitude / speed with FSM transitions")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return True


def plot_frontier_pipeline(ev, out):
    g = ev["global"]
    if not g:
        return False
    t0 = ev["t0"]
    ts = [x[0] - t0 for x in g]

    def series(key):
        return [(x[2].get(key) if x[2].get(key) is not None else float("nan"))
                for x in g]

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    ax1.plot(ts, series("clusters"), label="clusters total", lw=1.4)
    ax1.plot(ts, series("clusters_reach"), label="clusters reachable", lw=1.4)
    ax1.plot(ts, series("vps"), label="viewpoints", ls="--")
    ax1.plot(ts, series("vps_reach"), label="viewpoints path_reachable", ls="--")
    ax1.set_ylabel("count")
    ax1.legend(fontsize=8, ncol=2)
    ax1.grid(alpha=0.3)
    ax1.set_title("Frontier clusters / viewpoints")

    for key, c in [("dormant", "#9e9e9e"), ("prev_unreachable", "#e53935"),
                   ("no_candidate", "#fb8c00"), ("topo_unreachable", "#8e24aa"),
                   ("no_visibility", "#3949ab"), ("survived", "#2e7d32")]:
        ax2.plot(ts, series(key), label=key, color=c, lw=1.3)
    ax2.set_ylabel("clusters at stage")
    ax2.legend(fontsize=8, ncol=3)
    ax2.grid(alpha=0.3)
    ax2.set_title("Pipeline stage breakdown (where clusters die)")

    ratio_reach, ratio_surv = [], []
    for (_t, _r, d) in g:
        c, cr = d.get("clusters"), d.get("clusters_reach")
        e, s = d.get("evaluated"), d.get("survived")
        ratio_reach.append(100.0 * cr / c if c else float("nan"))
        ratio_surv.append(100.0 * s / e if e else float("nan"))
    ax3.plot(ts, ratio_reach, label="reachable / total clusters [%]", lw=1.4)
    ax3.plot(ts, ratio_surv, label="survived / evaluated [%]", lw=1.4)
    ax3.set_ylim(-5, 105)
    ax3.set_ylabel("%")
    ax3.set_xlabel("time [s]")
    ax3.legend(fontsize=8)
    ax3.grid(alpha=0.3)
    ax3.set_title("Detection→reachable rates")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return True


def plot_plan_health(ev, out):
    g, l = ev["global"], ev["local"]
    if not g and not l:
        return False
    t0 = ev["t0"]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 5.5), sharex=True)

    results = sorted({r for (_t, r, _d) in g})
    rmap = {r: i for i, r in enumerate(results)}
    ax1.scatter([x[0] - t0 for x in g], [rmap[x[1]] for x in g], s=14,
                c=["#2e7d32" if ("OK" in x[1]) else "#c62828" for x in g])
    ax1.set_yticks(range(len(results)))
    ax1.set_yticklabels(results, fontsize=7)
    ax1.set_title("GLOBAL plan result")
    ax1.grid(alpha=0.3)

    okx = [x[0] - t0 for x in l if x[2] == "OK"]
    oky = [x[3] or 0 for x in l if x[2] == "OK"]
    fx = [x[0] - t0 for x in l if x[2] != "OK"]
    fy = [x[3] or 0 for x in l if x[2] != "OK"]
    ax2.scatter(okx, oky, s=10, color="#2e7d32", label="LOCAL OK")
    ax2.scatter(fx, fy, s=16, color="#c62828", marker="x", label="LOCAL FAIL")
    ax2.set_ylabel("local plan_time [ms]")
    ax2.set_xlabel("time [s]")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return True


def plot_rth(ev, out):
    if not ev["rth"]:
        return False
    t0 = ev["t0"]
    fig, ax = plt.subplots(figsize=(9, 3.2))
    ax.plot([x[0] - t0 for x in ev["rth"]], [x[1] for x in ev["rth"]],
            marker=".", lw=1.2)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("dist to home [m]")
    ax.set_title("RTH progress")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return True


def plot_battery(ev, out):
    b = [x for x in ev["batt"] if x[1] is not None]
    if not b:
        return False
    t0 = ev["t0"]
    fig, ax = plt.subplots(figsize=(9, 3.2))
    ax.plot([x[0] - t0 for x in b], [x[1] for x in b], lw=1.3, color="#ef6c00")
    ax.set_ylabel("voltage [V]", color="#ef6c00")
    ax.set_xlabel("time [s]")
    ax.grid(alpha=0.3)
    pc = [x for x in b if x[2] is not None and x[2] >= 0]
    if pc:
        ax2 = ax.twinx()
        ax2.plot([x[0] - t0 for x in pc], [x[2] for x in pc], lw=1.0,
                 color="#1565c0", ls="--")
        ax2.set_ylabel("percent [%]", color="#1565c0")
    ax.set_title("Battery")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return True


def plot_tracking(bagdata, out):
    od, cmd = bagdata["odom"], bagdata["cmd"]
    if len(od) < 3 or len(cmd) < 3:
        return False
    t0 = od[0][0]
    ci, errs, ets = 0, [], []
    for (t, x, y, z) in od:
        while ci + 1 < len(cmd) and cmd[ci + 1][0] <= t:
            ci += 1
        c = cmd[ci]
        if abs(c[0] - t) > 0.5:
            continue
        errs.append(math.dist((x, y, z), c[1:4]))
        ets.append(t - t0)
    if not errs:
        return False
    fig, ax = plt.subplots(figsize=(9, 3.2))
    ax.plot(ets, errs, lw=0.9)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|odom - cmd| [m]")
    ax.set_title("Tracking error (odom vs /position_cmd)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return True


# ------------------------------------------------- traj-error (추종 정확도)

def offboard_window(ev):
    """PX4 이벤트에서 (OFFBOARD+armed 시작, 이탈/disarm) epoch 구간. 없으면 None."""
    start = None
    for (t, mode, armed) in ev["px4"]:
        if start is None and mode == "OFFBOARD" and armed == 1:
            start = t
        elif start is not None and (mode != "OFFBOARD" or armed == 0):
            return (start, t)
    return (start, ev["t_end"]) if start is not None else None


def analyze_traj_error(session_dir, bagdata, ev):
    """추종 정확도: /position_cmd(명령) vs /Odometry(실제) -> traj-error/
    RMSE 는 evo_ape(trans_part, 정렬 없음)와 동일 정의. OFFBOARD 구간만 평가
    (지상 대기·AUTO.LAND 하강은 추종 대상이 아니므로 제외)."""
    od, cmd = bagdata["odom"], bagdata["cmd"]
    if len(od) < 10 or len(cmd) < 10:
        return None
    out = os.path.join(session_dir, "traj-error")
    os.makedirs(out, exist_ok=True)
    win = offboard_window(ev)
    t_lo, t_hi = win if win else (od[0][0], od[-1][0])

    # 명령을 odom 타임스탬프에 정렬(직전 명령 hold) 후 오차 계산
    rows, ci = [], 0
    for (t, x, y, z) in od:
        if t < t_lo or t > t_hi:
            continue
        while ci + 1 < len(cmd) and cmd[ci + 1][0] <= t:
            ci += 1
        c = cmd[ci]
        if abs(c[0] - t) > 0.5:
            continue
        ex, ey, ez = x - c[1], y - c[2], z - c[3]
        rows.append((t, c[1], c[2], c[3], x, y, z, ex, ey, ez,
                     math.sqrt(ex * ex + ey * ey + ez * ez)))
    if len(rows) < 10:
        return None

    with open(os.path.join(out, "tracking_error.csv"), "w") as f:
        f.write("t_epoch,t_rel,ref_x,ref_y,ref_z,act_x,act_y,act_z,"
                "err_x,err_y,err_z,err_norm\n")
        for r in rows:
            f.write("%.6f,%.3f," % (r[0], r[0] - t_lo) +
                    ",".join("%.4f" % v for v in r[1:]) + "\n")

    errs = [r[10] for r in rows]
    errs_xy = [math.hypot(r[7], r[8]) for r in rows]
    errs_z = [abs(r[9]) for r in rows]

    def rmse(v):
        return math.sqrt(sum(x * x for x in v) / len(v))

    st = {"n": len(rows), "dur": t_hi - t_lo, "win": bool(win),
          "rmse_3d": rmse(errs), "rmse_xy": rmse(errs_xy), "rmse_z": rmse(errs_z),
          "mean": sum(errs) / len(errs), "median": sorted(errs)[len(errs) // 2],
          "max": max(errs)}

    # evo_ape (표준 도구): TUM 내보내기 -> APE(translation, 무정렬)
    evo_bin = shutil.which("evo_ape") or os.path.expanduser("~/.local/bin/evo_ape")
    evo_txt = None
    if os.path.exists(evo_bin):
        ref_tum = os.path.join(out, "cmd_ref.tum")
        est_tum = os.path.join(out, "odom_est.tum")
        with open(ref_tum, "w") as f:
            for c in cmd:
                if t_lo <= c[0] <= t_hi:
                    f.write("%.6f %.4f %.4f %.4f 0 0 0 1\n" % c[:4])
        with open(est_tum, "w") as f:
            for o in od:
                if t_lo <= o[0] <= t_hi:
                    f.write("%.6f %.4f %.4f %.4f 0 0 0 1\n" % o[:4])
        try:
            zipf = os.path.join(out, "evo_ape_results.zip")
            if os.path.exists(zipf):
                os.remove(zipf)
            r = subprocess.run(
                [evo_bin, "tum", ref_tum, est_tum, "-r", "trans_part",
                 "--t_max_diff", "0.05", "--save_results", zipf],
                capture_output=True, text=True, timeout=120)
            evo_txt = r.stdout + (("\n[stderr]\n" + r.stderr) if r.returncode else "")
            with open(os.path.join(out, "evo_ape.txt"), "w") as f:
                f.write(evo_txt)
        except Exception as e:
            evo_txt = "evo_ape 실행 실패: %s" % e

    with open(os.path.join(out, "summary.txt"), "w") as f:
        f.write("Tracking accuracy: /position_cmd (ref) vs /Odometry (actual)\n")
        f.write("window: %s, %.1fs, n=%d\n" %
                ("OFFBOARD+armed" if st["win"] else "full session (no OFFBOARD event)",
                 st["dur"], st["n"]))
        f.write("RMSE 3D: %.4f m\nRMSE XY: %.4f m\nRMSE Z : %.4f m\n"
                % (st["rmse_3d"], st["rmse_xy"], st["rmse_z"]))
        f.write("mean: %.4f  median: %.4f  max: %.4f [m]\n"
                % (st["mean"], st["median"], st["max"]))
        if evo_txt:
            f.write("\n--- evo_ape tum (translation part, no alignment) ---\n")
            f.write(evo_txt)

    ts = [r[0] - t_lo for r in rows]
    fig, ax = plt.subplots(figsize=(11, 3.6))
    ax.plot(ts, errs, lw=0.9, label="3D")
    ax.plot(ts, errs_xy, lw=0.8, alpha=0.85, label="XY")
    ax.plot(ts, errs_z, lw=0.8, alpha=0.85, label="Z")
    ax.axhline(st["rmse_3d"], color="r", ls="--", lw=1.1,
               label="RMSE 3D=%.3f m" % st["rmse_3d"])
    ax.set_xlabel("time in OFFBOARD window [s]")
    ax.set_ylabel("tracking error [m]")
    ax.legend(fontsize=8, ncol=4)
    ax.grid(alpha=0.3)
    ax.set_title("Tracking error over time")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "error_over_time.png"), dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.5, 3.6))
    ax.hist(errs, bins=40, color="#1565c0", alpha=0.85)
    ax.axvline(st["rmse_3d"], color="r", ls="--",
               label="RMSE=%.3f m" % st["rmse_3d"])
    ax.set_xlabel("3D error [m]")
    ax.set_ylabel("samples")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_title("Error distribution")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "error_hist.png"), dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 6.5))
    ax.plot([r[1] for r in rows], [r[2] for r in rows], lw=1.6,
            color="#ef6c00", label="commanded (/position_cmd)")
    ax.plot([r[4] for r in rows], [r[5] for r in rows], lw=1.1,
            color="#1565c0", label="actual (/Odometry)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("Commanded vs actual path (XY)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "path_overlay_xy.png"), dpi=130)
    plt.close(fig)
    return st


# --------------------------------------------- local-plan-time (연산시간)

def _pct(vals, p):
    s = sorted(vals)
    return s[min(len(s) - 1, int(p / 100.0 * len(s)))]


def analyze_plan_time(session_dir, bagdata):
    """로컬 계획(MINCO 파이프라인 + 탐색/체크) 연산시간 -> local-plan-time/"""
    timing = {k: v for k, v in (bagdata.get("timing") or {}).items() if len(v) >= 2}
    if not timing:
        return None
    out = os.path.join(session_dir, "local-plan-time")
    os.makedirs(out, exist_ok=True)
    t0 = min(v[0][0] for v in timing.values())

    with open(os.path.join(out, "local_plan_times.csv"), "w") as f:
        f.write("t_epoch,t_rel,metric,ms\n")
        for k in sorted(timing):
            for (t, ms) in timing[k]:
                f.write("%.6f,%.3f,%s,%.4f\n" % (t, t - t0, k, ms))

    stats = {}
    with open(os.path.join(out, "summary.txt"), "w") as f:
        f.write("Local planning computation time [ms]\n")
        f.write("%-55s %6s %8s %8s %8s %8s\n" %
                ("metric", "n", "mean", "median", "p95", "max"))
        for k in sorted(timing):
            v = [x[1] for x in timing[k]]
            stats[k] = {"n": len(v), "mean": sum(v) / len(v),
                        "median": _pct(v, 50), "p95": _pct(v, 95), "max": max(v)}
            f.write("%-55s %6d %8.2f %8.2f %8.2f %8.2f\n" %
                    (k, len(v), stats[k]["mean"], stats[k]["median"],
                     stats[k]["p95"], stats[k]["max"]))

    minco = {k: v for k, v in timing.items() if "/visualizer/" in k}
    search = {k: v for k, v in timing.items() if "/visualizer/" not in k}
    fig, axes = plt.subplots(2, 1, figsize=(11.5, 7), sharex=True)
    for ax, group, title in [(axes[0], minco, "MINCO pipeline stages"),
                             (axes[1], search, "Search / check")]:
        for k in sorted(group):
            ts = [x[0] - t0 for x in group[k]]
            vs = [x[1] for x in group[k]]
            ax.plot(ts, vs, marker=".", ms=3, lw=0.8, label=k.split("/")[-1])
        ax.set_ylabel("time [ms]")
        ax.set_yscale("log")
        ax.grid(alpha=0.3, which="both")
        ax.legend(fontsize=7, ncol=3)
        ax.set_title(title)
    axes[1].set_xlabel("time since first sample [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "timeseries.png"), dpi=130)
    plt.close(fig)

    keys = sorted(timing, key=lambda k: -stats[k]["median"])
    fig, ax = plt.subplots(figsize=(11.5, 0.55 * len(keys) + 2))
    ax.boxplot([[x[1] for x in timing[k]] for k in keys], vert=False,
               labels=[k.split("/")[-1] for k in keys], showfliers=True,
               flierprops=dict(marker=".", ms=3, alpha=0.4))
    for i, k in enumerate(keys):
        ax.text(stats[k]["p95"], i + 1.28, "p95=%.1f" % stats[k]["p95"],
                fontsize=6.5, color="#c62828")
    ax.set_xscale("log")
    ax.set_xlabel("time [ms] (log)")
    ax.grid(alpha=0.3, axis="x", which="both")
    ax.set_title("Local planning time distribution")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "boxplot.png"), dpi=130)
    plt.close(fig)
    return stats


# ---------------------------------------------------------------- 리포트

def write_report(session_dir, ev, made, extra_rows=""):
    name = os.path.basename(os.path.normpath(session_dir))
    dur = (ev["t_end"] - ev["t0"]) if ev["t0"] else 0
    n_stuck = len(ev["stuck"])
    states = " → ".join(dict.fromkeys([s for (_a, _b, s) in _state_bands(ev)]))
    g_fail = sum(1 for x in ev["global"] if "OK" not in x[1])
    l_fail = sum(1 for x in ev["local"] if x[2] != "OK")
    missions = "<br>".join(html.escape(f"[+{t-ev['t0']:.1f}s] {txt}")
                           for (t, txt) in ev["mission"]) or "-"
    rows = f"""
    <tr><td>세션</td><td>{html.escape(name)}</td></tr>
    <tr><td>길이</td><td>{dur:.1f} s</td></tr>
    <tr><td>상태 흐름</td><td>{html.escape(states)}</td></tr>
    <tr><td>MISSION</td><td>{missions}</td></tr>
    <tr><td>GLOBAL 실패 횟수</td><td>{g_fail}</td></tr>
    <tr><td>LOCAL 실패 횟수</td><td>{l_fail}</td></tr>
    <tr><td>STUCK 이벤트</td><td>{n_stuck}</td></tr>
    {extra_rows}
    """
    imgs = "\n".join(
        f'<h2>{html.escape(title)}</h2><img src="{html.escape(rel)}" style="max-width:100%">'
        for (rel, title) in made)
    doc = f"""<!doctype html><html><head><meta charset="utf-8">
<title>{html.escape(name)}</title>
<style>body{{font-family:sans-serif;max-width:1100px;margin:24px auto;padding:0 12px}}
table{{border-collapse:collapse}}td{{border:1px solid #ccc;padding:4px 10px}}
h1{{font-size:20px}}h2{{font-size:15px;margin-top:28px}}</style></head><body>
<h1>EPIC session report — {html.escape(name)}</h1>
<table>{rows}</table>
{imgs}
</body></html>"""
    with open(os.path.join(session_dir, "report.html"), "w") as f:
        f.write(doc)


def process_session(session_dir):
    session_dir = os.path.abspath(session_dir)
    name = os.path.basename(os.path.normpath(session_dir))
    ev_path = glob.glob(os.path.join(session_dir, "*.events.log"))
    if not ev_path:
        print(f"[plot_session] {name}: events.log 없음 — 건너뜀")
        return False
    ev = parse_events(ev_path[0])
    if ev["t0"] is None:
        print(f"[plot_session] {name}: 이벤트가 비어있음 — 건너뜀")
        return False

    plots_dir = os.path.join(session_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)
    made = []

    def add(fn, title, fn_call):
        try:
            okv = fn_call(os.path.join(plots_dir, fn))
            if okv is not False:
                made.append(("plots/" + fn, title))
        except Exception as e:
            print(f"[plot_session] {fn} 실패: {e}")

    add("timeline.png", "타임라인 (FSM / PX4 / Avoidance)",
        lambda o: plot_timeline(ev, o))

    bags = glob.glob(os.path.join(session_dir, "*.bag"))
    bagdata = read_bag(bags[0]) if bags else None
    if bagdata and bagdata["odom"]:
        add("trajectory_xy.png", "비행 경로 (XY)",
            lambda o: plot_trajectory(bagdata, ev, o))
        add("altitude_speed.png", "고도 / 속도",
            lambda o: plot_altitude_speed(bagdata, ev, o))
    else:
        print(f"[plot_session] {name}: bag 없음/비어있음 — 경로·고도·추종오차 생략")

    add("frontier_pipeline.png", "프론티어 / 파이프라인 / 도달 비율",
        lambda o: plot_frontier_pipeline(ev, o))
    add("plan_health.png", "계획 결과 / 소요시간",
        lambda o: plot_plan_health(ev, o))
    add("rth_progress.png", "RTH 진행", lambda o: plot_rth(ev, o))
    add("battery.png", "배터리", lambda o: plot_battery(ev, o))

    # ---- traj-error/ : 추종 정확도 (RMSE, evo) ----
    extra_rows = ""
    if bagdata and bagdata["odom"]:
        try:
            st = analyze_traj_error(session_dir, bagdata, ev)
        except Exception as e:
            st = None
            print(f"[plot_session] traj-error 실패: {e}")
        if st:
            made.append(("traj-error/error_over_time.png", "추종 오차 (시간)"))
            made.append(("traj-error/error_hist.png", "추종 오차 분포"))
            made.append(("traj-error/path_overlay_xy.png", "명령 vs 실제 경로"))
            extra_rows += ("<tr><td>추종 RMSE (3D/XY/Z)</td>"
                           "<td>%.3f / %.3f / %.3f m (OFFBOARD %ds, n=%d)</td></tr>"
                           % (st["rmse_3d"], st["rmse_xy"], st["rmse_z"],
                              int(st["dur"]), st["n"]))

    # ---- local-plan-time/ : 로컬 계획 연산시간 ----
    if bagdata:
        try:
            pt = analyze_plan_time(session_dir, bagdata)
        except Exception as e:
            pt = None
            print(f"[plot_session] local-plan-time 실패: {e}")
        if pt:
            made.append(("local-plan-time/timeseries.png", "로컬 계획 연산시간 (시계열)"))
            made.append(("local-plan-time/boxplot.png", "로컬 계획 연산시간 (분포)"))
            key = "/visualizer/totoalOptimize_timecost"
            if key in pt:
                extra_rows += ("<tr><td>MINCO 최적화 시간 (median/p95)</td>"
                               "<td>%.1f / %.1f ms</td></tr>"
                               % (pt[key]["median"], pt[key]["p95"]))

    write_report(session_dir, ev, made, extra_rows)
    print(f"[plot_session] {name}: {len(made)}개 plot + report.html 생성")
    return True


def default_records_dir():
    try:
        import rospkg
        return os.path.join(rospkg.RosPack().get_path("epic_planner"), "records")
    except Exception:
        return os.path.expanduser("~/records")


def main():
    args = [a for a in sys.argv[1:] if a]
    rec = default_records_dir()
    if "--all" in args:
        for d in sorted(glob.glob(os.path.join(rec, "*_20*"))):
            if os.path.isdir(d):
                process_session(d)
        return
    if args:
        process_session(args[0])
        return
    sessions = sorted([d for d in glob.glob(os.path.join(rec, "*_20*"))
                       if os.path.isdir(d)], key=os.path.getmtime)
    if not sessions:
        print("[plot_session] 세션 폴더 없음:", rec)
        sys.exit(1)
    process_session(sessions[-1])


if __name__ == "__main__":
    main()
