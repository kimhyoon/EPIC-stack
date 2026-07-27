#!/usr/bin/env python3

"""Check odometry quality and topology retry evidence in a planner debug bag."""

import argparse
import csv
import math
from collections import Counter
from pathlib import Path

import rosbag


RESULT_NAMES = {
    1: "REACH_END",
    2: "NO_PATH",
    3: "START_FAIL",
    4: "END_FAIL",
    5: "TIME_OUT",
    6: "EDGE_COLLISION",
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--odom-topic", default="")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--max-odom-step", type=float, default=1.0)
    parser.add_argument("--max-derived-speed", type=float, default=10.0)
    parser.add_argument("--max-time-gap", type=float, default=0.5)
    parser.add_argument("--strict", action="store_true")
    return parser.parse_args()


def select_odom_topic(bag, requested):
    topic_info = bag.get_type_and_topic_info().topics
    if requested:
        if requested not in topic_info:
            raise RuntimeError("requested odometry topic is absent: %s" %
                               requested)
        return requested
    preferred = [
        "/quad_0/lidar_slam/odom",
        "/quad_0/lidar_slam/odom_throttled",
        "/Odometry",
        "/mavros/local_position/odom",
        "/mavros/odometry/in",
    ]
    for topic in preferred:
        if topic in topic_info and topic_info[topic].msg_type == "nav_msgs/Odometry":
            return topic
    for topic, info in topic_info.items():
        if info.msg_type == "nav_msgs/Odometry":
            return topic
    raise RuntimeError("no nav_msgs/Odometry topic found")


def message_time(msg, bag_time):
    if hasattr(msg, "header") and msg.header.stamp.to_sec() > 0.0:
        return msg.header.stamp.to_sec()
    return bag_time.to_sec()


def parse_diag(text):
    result = {}
    for field in text.split(";"):
        if "=" not in field:
            continue
        key, value = field.split("=", 1)
        result[key] = value
    return result


def pair_key(values):
    start = tuple(round(value, 2) for value in values[0:3])
    end = tuple(round(value, 2) for value in values[3:6])
    return tuple(sorted((start, end)))


def main():
    args = parse_args()
    bag_path = args.bag.resolve()
    if not bag_path.is_file():
        raise SystemExit("bag not found: %s" % bag_path)
    output_dir = (args.output_dir or
                  bag_path.parent / (bag_path.stem + "_topology_validation"))
    output_dir.mkdir(parents=True, exist_ok=True)

    odom_rows = []
    edge_rows = []
    edge_results = Counter()
    retry_pending_peak = 0
    retry_reactivated_peak = 0
    retry_pending_samples = 0
    node_samples = 0
    retained_node_samples = 0
    node_count_min = None
    node_count_max = 0
    failed_pairs = set()
    recovered_pairs = set()

    with rosbag.Bag(str(bag_path), "r") as bag:
        odom_topic = select_odom_topic(bag, args.odom_topic)
        topics = [
            odom_topic,
            "/planning/expl_diag_kv",
            "/debug/topo_edge_updates",
            "/debug/topology_stability_nodes",
        ]
        previous = None
        for topic, msg, bag_time in bag.read_messages(topics=topics):
            if topic == odom_topic:
                stamp = message_time(msg, bag_time)
                position = (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                )
                step = 0.0
                derived_speed = 0.0
                dt = 0.0
                if previous is not None:
                    dt = stamp - previous[0]
                    step = math.sqrt(sum(
                        (position[i] - previous[1][i]) ** 2
                        for i in range(3)))
                    if dt > 1.0e-6:
                        derived_speed = step / dt
                twist_speed = math.sqrt(
                    msg.twist.twist.linear.x ** 2 +
                    msg.twist.twist.linear.y ** 2 +
                    msg.twist.twist.linear.z ** 2)
                odom_rows.append({
                    "time": stamp,
                    "dt": dt,
                    "x": position[0],
                    "y": position[1],
                    "z": position[2],
                    "step": step,
                    "derived_speed": derived_speed,
                    "twist_speed": twist_speed,
                })
                previous = (stamp, position)
            elif topic == "/planning/expl_diag_kv":
                fields = parse_diag(msg.data)
                pending = int(fields.get("pipe_retry_pending", "0"))
                reactivated = int(fields.get("pipe_retry_reactivated", "0"))
                retry_pending_peak = max(retry_pending_peak, pending)
                retry_reactivated_peak = max(retry_reactivated_peak,
                                             reactivated)
                if pending > 0:
                    retry_pending_samples += 1
            elif topic == "/debug/topo_edge_updates":
                data = list(msg.data)
                if len(data) < 3 or int(round(data[0])) != 1:
                    continue
                pair_count = int(round(data[2]))
                offset = 3
                for _ in range(pair_count):
                    if offset + 10 > len(data):
                        break
                    success = int(round(data[offset]))
                    result = int(round(data[offset + 1]))
                    was_connected = int(round(data[offset + 2]))
                    coordinates = data[offset + 3:offset + 9]
                    failure_count = int(round(data[offset + 9]))
                    key = pair_key(coordinates)
                    if not success:
                        failed_pairs.add(key)
                    elif key in failed_pairs:
                        recovered_pairs.add(key)
                    edge_results[RESULT_NAMES.get(
                        result, "RESULT_%d" % result)] += 1
                    edge_rows.append([
                        bag_time.to_sec(), success,
                        RESULT_NAMES.get(result, str(result)),
                        was_connected, failure_count,
                    ] + coordinates)
                    offset += 10
            elif topic == "/debug/topology_stability_nodes":
                node_samples += 1
                count = len(msg.points)
                node_count_min = count if node_count_min is None else min(
                    node_count_min, count)
                node_count_max = max(node_count_max, count)
                retained_node_samples += sum(
                    1 for color in msg.colors
                    if color.r > 0.8 and color.g > 0.5)

    with (output_dir / "odom_quality.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "time", "dt", "x", "y", "z", "step", "derived_speed",
            "twist_speed",
        ])
        writer.writeheader()
        writer.writerows(odom_rows)
    with (output_dir / "topology_edge_events.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "time", "success", "result", "was_connected",
            "consecutive_failures", "start_x", "start_y", "start_z",
            "end_x", "end_y", "end_z",
        ])
        writer.writerows(edge_rows)

    backward = sum(1 for row in odom_rows[1:] if row["dt"] < -1.0e-6)
    duplicate = sum(1 for row in odom_rows[1:]
                    if abs(row["dt"]) <= 1.0e-6)
    jumps = sum(1 for row in odom_rows[1:]
                if row["step"] > args.max_odom_step)
    speed_spikes = sum(1 for row in odom_rows[1:]
                       if row["derived_speed"] > args.max_derived_speed)
    time_gaps = sum(1 for row in odom_rows[1:]
                    if row["dt"] > args.max_time_gap)
    max_step = max((row["step"] for row in odom_rows), default=0.0)
    max_speed = max((row["derived_speed"] for row in odom_rows), default=0.0)
    max_gap = max((row["dt"] for row in odom_rows), default=0.0)
    duration = (odom_rows[-1]["time"] - odom_rows[0]["time"]
                if len(odom_rows) >= 2 else 0.0)

    hard_failures = backward + jumps + speed_spikes
    if len(odom_rows) < 10 or hard_failures > 0:
        quality = "INVALID_FOR_ALGORITHM_COMPARISON"
    elif time_gaps > 0 or duplicate > max(1, len(odom_rows) // 100):
        quality = "DEGRADED"
    else:
        quality = "VALID"

    report = [
        "# Topology Stability Bag Validation",
        "",
        "- Bag: `%s`" % bag_path,
        "- Odometry topic: `%s`" % odom_topic,
        "- Quality gate: **%s**" % quality,
        "- Odometry samples/duration: %d / %.2f s" %
        (len(odom_rows), duration),
        "- Max step / derived speed / gap: %.3f m / %.3f m/s / %.3f s" %
        (max_step, max_speed, max_gap),
        "- Backward timestamps / duplicate timestamps / time gaps: %d / %d / %d" %
        (backward, duplicate, time_gaps),
        "- Position jumps / speed spikes: %d / %d" % (jumps, speed_spikes),
        "",
        "## Updated Algorithm Evidence",
        "",
        "- Viewpoint retry pending peak/samples: %d / %d" %
        (retry_pending_peak, retry_pending_samples),
        "- Viewpoint retry reactivated peak: %d" % retry_reactivated_peak,
        "- Edge update events: %d" % len(edge_rows),
        "- Failed edge pairs later recovered: %d" % len(recovered_pairs),
        "- Stability marker samples: %d" % node_samples,
        "- Node count range: %s..%d" %
        ("n/a" if node_count_min is None else node_count_min, node_count_max),
        "- Hysteresis-retained node observations: %d" %
        retained_node_samples,
        "",
        "## Edge Results",
        "",
    ]
    for name, count in sorted(edge_results.items()):
        report.append("- %s: %d" % (name, count))
    report.extend([
        "",
        "A bag marked `INVALID_FOR_ALGORITHM_COMPARISON` must not be used to "
        "claim topology performance. It may only be used as a robustness "
        "stress test for crashes, permanent retry lockout, and premature "
        "FINISH behavior.",
        "",
    ])
    (output_dir / "summary.md").write_text(
        "\n".join(report), encoding="utf-8")
    print("\n".join(report))
    if args.strict and quality != "VALID":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
