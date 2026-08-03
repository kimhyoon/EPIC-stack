#!/usr/bin/env python3
"""Replay an EPIC flight bag, playing ONLY the planner's inputs.

A flight bag holds both what EPIC consumed (cloud, odom, mavros, trigger) and
what EPIC produced (trajectories, markers, /position_cmd, diagnostics). Playing
it whole would put the recorded results back on the same topic names that the
live planner publishes on, so rviz would show two publishers per topic and the
recorded path would be indistinguishable from the freshly planned one.

So: classify every topic in the bag, play the inputs, drop EPIC's own outputs,
and let the live planner recompute. The same sensor stream then produces a
newly planned path that can be compared against the recorded run.

Classification is a DENY-list of EPIC's advertised topics (gathered from the
advertise() calls in the EPIC tree). Deny beats allow. Anything not recognised
is played by default and reported, so an unexpected topic shows up in the table
rather than silently vanishing.

Usage (normally via bag_replay.launch):
    rosrun epic_planner bag_replay.py --bag flight.bag [--rate 1.0] [--start 0]
    rosrun epic_planner bag_replay.py --bag flight.bag --dry-run
"""

import argparse
import fnmatch
import os
import subprocess
import sys

try:
    import rosbag
except ImportError:
    sys.stderr.write("[bag_replay] rosbag python module not found; source the ROS setup.\n")
    raise


# EPIC's own outputs. Never replay these — the live planner republishes them.
# Patterns are fnmatch globs matched against the full topic name.
DENY_PATTERNS = [
    # planner results, diagnostics and timing
    "/planning/*",
    "/planning_vis/*",
    "/global_planning/*",
    "/local_planning/*",
    "/epic/*",
    "/trajectory/*",
    "/traj_generator/*",
    "/debug/*",
    "/time_cost",
    "/mem_cost*",
    "/yaw_state",
    "/lio_interface/map_updated",
    "/frontier_manager/explored_cell_count",
    # control output (would fight the live planner's command)
    "/position_cmd",
    "/px4ctrl/takeoff_land",
    "/drone_commander/onboard_command",
    # local planner (MINCO/GCOPTER) visualiser + topo graph visualiser
    "/visualizer/*",
    "/global_tour",
    "/viz_graph_topic",
    # node-relative visualisation topics; the node name prefix varies, so match
    # both the bare and namespaced forms
    "*sf_cluster_marker",
    "*viewpoint_candidates",
    "*norm_directions",
    "*viewpoint_centers",
    "*/good_obs", "*/bad_obs",
    "/occ", "*/occ",
    "/pocc", "*/pocc",
    "/frt", "*/frt",
    # odom_visualization / traj visual markers are regenerated live
    "*_odom_visualization/*",
    "/quad0_odom_visualization*",
    # waypoint_generator is relaunched by bag_replay.launch, so its own echo
    # topics would double up
    "/waypoint_generator/waypoints_vis",
    # setpoints the control chain sent TO the vehicle (downstream of
    # /position_cmd). Not an EPIC input, and replaying them alongside a live
    # plan is misleading. /mavros/state|battery|odometry/in ARE inputs and stay.
    "/mavros/setpoint_raw/*",
    "/mavros/setpoint_position/*",
    "/mavros/setpoint_velocity/*",
    "/mavros/setpoint_attitude/*",
    # log echo of the recorded run — would interleave with the live node's logs
    "/rosout", "/rosout_agg",
]

# Inputs EPIC needs. Anything matching these is always played even if a deny
# pattern would otherwise catch it is NOT true — deny wins — but listing them
# makes the report readable and documents the contract.
ALLOW_HINTS = [
    "/tf", "/tf_static",
    "/clock",
    "/Odometry", "/odometry*",
    "/cloud_registered*",
    "/lio_sam/*",
    "/livox/*",
    "/ml_/pointcloud",
    "/mavros/state", "/mavros/battery", "/mavros/odometry/in",
    "/FSM_flag_avoidance",
    "/waypoint_generator/waypoints",
    "/move_base_simple/goal",
    "*/cloud", "*/cloud_cropped",
    "*lidar_slam/odom",
]


def matches(topic, patterns):
    return any(fnmatch.fnmatch(topic, p) for p in patterns)


def classify(topics, extra_deny, extra_allow, minimal=False):
    """Return (play, skip) lists of (topic, reason).

    Deny wins over the built-in allow hints so an EPIC output can never leak
    back in. extra_allow is applied first and CAN override a deny — the escape
    hatch for a bag whose input happens to sit under a denied prefix.

    minimal=True plays only topics positively recognised as planner inputs;
    everything unrecognised (camera images, telemetry, ...) is dropped. Useful
    for the big real-flight bags where the ML-X image streams dominate the size
    but EPIC never reads them.
    """
    play, skip = [], []
    for t in sorted(topics):
        if matches(t, extra_allow):
            play.append((t, "forced by --allow"))
            continue
        if matches(t, DENY_PATTERNS) or matches(t, extra_deny):
            skip.append((t, "EPIC output"))
            continue
        if matches(t, ALLOW_HINTS):
            play.append((t, "planner input"))
        elif minimal:
            skip.append((t, "not a known input (--minimal)"))
        else:
            play.append((t, "unrecognised -> played"))
    return play, skip


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="path to the flight bag")
    ap.add_argument("--rate", default="1.0", help="rosbag play rate (default 1.0)")
    ap.add_argument("--start", default="0.0", help="start offset [s]")
    ap.add_argument("--duration", default="", help="play only this many seconds")
    # roslaunch cannot mix $(eval ...) with other substitutions in one attribute,
    # so the boolean flags accept an explicit true/false value as well as the
    # bare form (--loop == --loop true).
    ap.add_argument("--loop", nargs="?", const="true", default="false",
                    help="loop the bag")
    ap.add_argument("--clock", default="true", help="publish /clock (default true)")
    ap.add_argument("--delay", default="3.0",
                    help="wait this long before playing so the planner is up")
    ap.add_argument("--allow", default="",
                    help="comma-separated extra topics/globs to force-play")
    ap.add_argument("--deny", default="",
                    help="comma-separated extra topics/globs to exclude")
    ap.add_argument("--dry-run", dest="dry_run", nargs="?", const="true",
                    default="false",
                    help="print the classification and exit without playing")
    ap.add_argument("--minimal", nargs="?", const="true", default="false",
                    help="play only recognised planner inputs (drop camera/telemetry)")
    # roslaunch appends __name:= / __log:= remap args to every node it starts,
    # so parse_known_args (not parse_args) — otherwise the node dies at startup.
    args, unknown = ap.parse_known_args()
    stray = [a for a in unknown if not a.startswith("__")]
    if stray:
        sys.stderr.write("[bag_replay] ignoring unexpected args: %s\n" % " ".join(stray))

    def truthy(v):
        return str(v).strip().lower() in ("1", "true", "yes", "on")

    do_loop = truthy(args.loop)
    do_minimal = truthy(args.minimal)
    do_dry_run = truthy(args.dry_run)

    bag_path = os.path.expanduser(args.bag)
    if not os.path.isfile(bag_path):
        sys.stderr.write("[bag_replay] bag not found: %s\n" % bag_path)
        return 2

    extra_deny = [s.strip() for s in args.deny.split(",") if s.strip()]
    extra_allow = [s.strip() for s in args.allow.split(",") if s.strip()]

    with rosbag.Bag(bag_path, "r") as bag:
        info = bag.get_type_and_topic_info()
        topic_info = info.topics

    play, skip = classify(list(topic_info.keys()), extra_deny, extra_allow,
                          minimal=do_minimal)

    def row(t, why):
        ti = topic_info[t]
        return "  %-52s %-34s %7d msgs   %s" % (
            t, ti.msg_type.split("/")[-1], ti.message_count, why)

    print("=" * 110)
    print("[bag_replay] %s" % bag_path)
    print("=" * 110)
    print("PLAY  (%d topics) -> fed to the live planner" % len(play))
    for t, why in play:
        print(row(t, why))
    print("")
    print("SKIP  (%d topics) -> EPIC's recorded output, replaced by live results" % len(skip))
    for t, why in skip:
        print(row(t, why))
    print("=" * 110)

    if not play:
        sys.stderr.write("[bag_replay] nothing left to play; check --deny patterns.\n")
        return 3

    unrecognised = [t for t, why in play if why.startswith("unrecognised")]
    if unrecognised:
        print("[bag_replay] NOTE: %d topic(s) were not recognised and are played by "
              "default. If any is an EPIC output, exclude it with "
              "--deny (or bag_replay.launch deny:=...):" % len(unrecognised))
        for t in unrecognised:
            print("    %s" % t)
        print("")

    if do_dry_run:
        return 0

    cmd = ["rosbag", "play", bag_path, "--rate", str(args.rate),
           "--start", str(args.start), "--delay", str(args.delay)]
    if str(args.clock).lower() in ("1", "true", "yes"):
        cmd.append("--clock")
    if do_loop:
        cmd.append("--loop")
    if args.duration:
        cmd += ["--duration", str(args.duration)]
    cmd.append("--topics")
    cmd += [t for t, _ in play]

    print("[bag_replay] %s" % " ".join(cmd[:8] + ["...", "(%d topics)" % len(play)]))
    sys.stdout.flush()
    try:
        return subprocess.call(cmd)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
