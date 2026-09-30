#!/usr/bin/env python3
"""Isolated SO101 folded-arm hover and optional real SUPER handover probe.

Runs the production yhang550 model without a bench anchor or payload.  Only
processes started by this script are stopped.  Results are written to a unique
build/empty_arm_hover/<name> directory.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess as sp
import threading
import time
import xml.etree.ElementTree as ET

import rosgraph
import rospkg
import rospy
import yaml
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import JointState


ROOT = Path(os.environ.get("PX4_DIR", str(Path.home() / "PX4-RoboMaster"))).resolve()
ROS_WS = Path(os.environ.get("ROS_WS", str(Path.home() / "super_ws"))).resolve()
BUILD = ROOT / "build/px4_sitl_default"
MODELS = ROOT / "Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
EMPTY_WORLD = ROOT / "Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world"
BRIDGE_PACKAGE = Path(rospkg.RosPack().get_path("super_px4_bridge"))
NAVIGATION_LAUNCH = BRIDGE_PACKAGE / "launch/super_navigation.launch"
SUPER_CONFIG = BRIDGE_PACKAGE / "config/px4_sim.yaml"
FOLDED = {
    "shoulder_pan": -0.070,
    "shoulder_lift": -1.175,
    "elbow_flex": 1.450,
    "wrist_flex": 1.300,
    "wrist_roll": -1.570,
    "gripper": 0.300,
}
FIELDS = (
    "sim_s", "armed", "mode", "manager_state", "odom_x", "odom_y", "odom_z", "speed_m_s",
    "target_z", "truth_z", "roll_deg", "pitch_deg", "max_fold_error_rad",
    "max_fold_speed_rad_s", *(f"joint_{name}" for name in FOLDED),
)


def port_busy(port):
    with socket.socket() as sock:
        sock.settimeout(0.3)
        return sock.connect_ex(("127.0.0.1", port)) == 0


def process_running(name):
    return sp.run(["pgrep", "-x", name], stdout=sp.DEVNULL,
                  stderr=sp.DEVNULL, check=False).returncode == 0


def stop_group(proc):
    if proc is None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=6)
        return
    except sp.TimeoutExpired:
        pass
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            proc.wait(timeout=4)
            break
        except sp.TimeoutExpired:
            pass


def make_world(path):
    tree = ET.parse(EMPTY_WORLD)
    world = tree.find("world")
    if world is None:
        raise RuntimeError("empty.world has no world element")
    vehicle = ET.SubElement(world, "include")
    ET.SubElement(vehicle, "uri").text = "model://yhang550"
    ET.SubElement(vehicle, "name").text = "yhang550"
    ET.SubElement(vehicle, "pose").text = "0 0 0 0 0 0"
    tree.write(path, encoding="unicode", xml_declaration=True)


def attitude_degrees(q):
    sin_roll = 2 * (q.w * q.x + q.y * q.z)
    cos_roll = 1 - 2 * (q.x * q.x + q.y * q.y)
    sin_pitch = 2 * (q.w * q.y - q.z * q.x)
    return (math.degrees(math.atan2(sin_roll, cos_roll)),
            math.degrees(math.asin(max(-1.0, min(1.0, sin_pitch)))))


def make_navigation_probe(folder, hold_yaw):
    """Save a launch/config test artifact, leaving ROS source files untouched."""
    if not hold_yaw:
        return NAVIGATION_LAUNCH
    config = yaml.safe_load(SUPER_CONFIG.read_text())
    config["fsm"]["click_yaw_en"] = True
    config["super_planner"]["goal_yaw_en"] = True
    config_path = folder / "super_probe.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    launch = ET.parse(NAVIGATION_LAUNCH)
    planner = launch.find("node[@pkg='super_planner']")
    if planner is None:
        raise RuntimeError("navigation launch has no SUPER planner node")
    ET.SubElement(planner, "param", name="config_path", value=str(config_path))
    path = folder / "navigation_probe.launch"
    launch.write(path, encoding="unicode", xml_declaration=True)
    return path


class SetpointTrace:
    """Capture each stage, using simulator timestamps rather than wall time."""

    def __init__(self, folder):
        self.lock = threading.Lock()
        self.closed = False
        self.manager_state = "WAIT_FCU"
        self.samples = []
        self.events = []
        self.file = (folder / "setpoints.csv").open("w", newline="")
        fields = ("stream", "receive_s", "stamp_s", "x", "y", "z",
                  "vx", "vy", "vz", "ax", "ay", "az", "yaw", "type_mask")
        self.writer = csv.DictWriter(self.file, fieldnames=fields)
        self.writer.writeheader()
        self.event_file = (folder / "events.jsonl").open("w")
        self.subscribers = [
            rospy.Subscriber("/planning/pos_cmd", PositionCommand,
                             self.on_planner, queue_size=100),
            rospy.Subscriber("/super/setpoint_raw/local", PositionTarget,
                             lambda msg: self.on_raw("bridge", msg), queue_size=100),
            rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget,
                             lambda msg: self.on_raw("manager", msg), queue_size=100),
            rospy.Subscriber("/rosout_agg", Log, self.on_log, queue_size=100),
            rospy.Subscriber("/planning/resolved_goal", PoseStamped,
                             self.on_resolved_goal, queue_size=10),
        ]

    def record(self, stream, msg, acceleration, mask):
        row = dict(stream=stream, receive_s=rospy.Time.now().to_sec(),
                   stamp_s=msg.header.stamp.to_sec(), x=msg.position.x,
                   y=msg.position.y, z=msg.position.z, vx=msg.velocity.x,
                   vy=msg.velocity.y, vz=msg.velocity.z, ax=acceleration.x,
                   ay=acceleration.y, az=acceleration.z, yaw=msg.yaw,
                   type_mask=mask)
        with self.lock:
            if self.closed:
                return
            self.samples.append(row)
            self.writer.writerow(row)
            self.file.flush()

    def on_planner(self, msg):
        self.record("planner", msg, msg.acceleration, -1)

    def on_raw(self, stream, msg):
        self.record(stream, msg, msg.acceleration_or_force, msg.type_mask)

    def event(self, kind, **details):
        with self.lock:
            if self.closed:
                return
            item = dict(kind=kind, receive_s=rospy.Time.now().to_sec(), **details)
            self.events.append(item)
            self.event_file.write(json.dumps(item, allow_nan=False) + "\n")
            self.event_file.flush()

    def on_log(self, msg):
        match = re.search(r"\[FlightManager\] (\w+) -> (\w+)", msg.msg)
        if match:
            self.manager_state = match.group(2)
            self.event("transition", old=match.group(1), new=match.group(2),
                       stamp_s=msg.header.stamp.to_sec())

    def on_resolved_goal(self, msg):
        self.event("resolved_goal", stamp_s=msg.header.stamp.to_sec(),
                   x=msg.pose.position.x, y=msg.pose.position.y, z=msg.pose.position.z)

    def close(self):
        for subscriber in self.subscribers:
            subscriber.unregister()
        with self.lock:
            self.closed = True
            self.file.close()
            self.event_file.close()


def handover_summary(trace, goal_time):
    with trace.lock:
        events, samples = list(trace.events), list(trace.samples)
    final = next((e for e in events if e.get("new") == "FINAL_HOVER" and
                  e["receive_s"] >= goal_time), None)
    if final is None:
        return dict(observed=False)
    tracking = [s for s in samples if s["stream"] == "manager" and
                goal_time <= s["receive_s"] <= final["receive_s"] + 0.05 and
                not (s["type_mask"] & PositionTarget.IGNORE_VZ)]
    holds = [s for s in samples if s["stream"] == "manager" and
             s["receive_s"] >= final["stamp_s"] - 0.05 and
             s["type_mask"] & PositionTarget.IGNORE_VZ]
    planner = [s for s in samples if s["stream"] == "planner" and
               goal_time <= s["receive_s"] <= final["receive_s"]]
    if not tracking or not holds:
        return dict(observed=False, transition_sim_s=final["stamp_s"])
    before, after = tracking[-1], holds[0]
    delta = after["z"] - before["z"]
    position_step = math.sqrt(sum((after[k] - before[k]) ** 2 for k in ("x", "y", "z")))
    resolved = next((e for e in events if e["kind"] == "resolved_goal" and
                     goal_time <= e["stamp_s"] <= final["stamp_s"] + 0.05), None)
    return dict(observed=True, transition_sim_s=final["stamp_s"],
                last_tracking_z_m=before["z"], first_hold_z_m=after["z"],
                delta_z_m=delta, last_tracking_vz_m_s=before["vz"],
                last_planner_z_m=planner[-1]["z"] if planner else None,
                position_step_m=position_step, resolved_goal=resolved,
                continuous=abs(delta) <= 0.01 and position_step <= 0.02)


def yaw_trace_summary(trace):
    with trace.lock:
        samples = [s for s in trace.samples if s["stream"] == "manager"]
    rates = []
    for a, b in zip(samples, samples[1:]):
        dt = b["stamp_s"] - a["stamp_s"]
        if dt > 0:
            diff = math.atan2(math.sin(b["yaw"] - a["yaw"]), math.cos(b["yaw"] - a["yaw"]))
            rates.append(abs(diff / dt))
    return dict(max_command_yaw_rate_rad_s=max(rates) if rates else None,
                bounded=bool(rates) and max(rates) <= 0.351)


class Telemetry:
    def __init__(self, trace):
        self.trace = trace
        self.state = None
        self.odom = None
        self.target = None
        self.joints = None
        self.truth_z = math.nan
        rospy.Subscriber("/mavros/state", State, self.on_state, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.on_odom,
                         queue_size=20)
        rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget,
                         self.on_target, queue_size=20)
        rospy.Subscriber("/so101/joint_states", JointState, self.on_joints,
                         queue_size=20)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_models,
                         queue_size=10)

    def on_state(self, msg):
        self.state = msg

    def on_odom(self, msg):
        self.odom = msg

    def on_target(self, msg):
        self.target = msg

    def on_joints(self, msg):
        self.joints = msg

    def on_models(self, msg):
        if "yhang550" in msg.name:
            self.truth_z = msg.pose[msg.name.index("yhang550")].position.z

    def row(self):
        if self.state is None or self.odom is None or self.joints is None:
            return None
        names = dict(zip(self.joints.name, self.joints.position))
        speeds = dict(zip(self.joints.name, self.joints.velocity))
        if not all(name in names for name in FOLDED):
            return None
        if not all(name in speeds for name in FOLDED):
            return None
        pose = self.odom.pose.pose
        velocity = self.odom.twist.twist.linear
        roll, pitch = attitude_degrees(pose.orientation)
        row = dict(
            sim_s=rospy.Time.now().to_sec(),
            armed=self.state.armed,
            mode=self.state.mode,
            manager_state=self.trace.manager_state,
            odom_x=pose.position.x,
            odom_y=pose.position.y,
            odom_z=pose.position.z,
            speed_m_s=math.sqrt(velocity.x ** 2 + velocity.y ** 2 +
                                velocity.z ** 2),
            target_z=self.target.position.z if self.target else math.nan,
            truth_z=self.truth_z,
            roll_deg=roll,
            pitch_deg=pitch,
            max_fold_error_rad=max(abs(names[name] - target)
                                   for name, target in FOLDED.items()),
            max_fold_speed_rad_s=max(abs(speeds[name]) for name in FOLDED),
        )
        row.update({f"joint_{name}": names[name] for name in FOLDED})
        required = ("sim_s", "odom_x", "odom_y", "odom_z", "speed_m_s",
                    "roll_deg", "pitch_deg", "max_fold_error_rad",
                    "max_fold_speed_rad_s", *(f"joint_{name}" for name in FOLDED))
        if any(not math.isfinite(row[key]) for key in required):
            raise RuntimeError("non-finite flight or joint telemetry")
        return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="unique case name, letters/digits/underscore")
    parser.add_argument("--hover-seconds", type=float, default=20.0)
    parser.add_argument("--height", type=float, default=1.5)
    parser.add_argument("--gazebo-port", type=int, default=11358)
    parser.add_argument("--super-goal", type=float, nargs=3, metavar=("DX", "DY", "Z"),
                        help="real SUPER goal: relative local XY, absolute local Z (m)")
    parser.add_argument("--goal-body-frame", action="store_true",
                        help="rotate the goal XY offset by measured yaw (reduces initial turns)")
    parser.add_argument("--super-hold-yaw", action="store_true",
                        help="test-only config enables the requested terminal yaw constraint")
    parser.add_argument("--next-super-goal", type=float, nargs=3, metavar=("DX", "DY", "Z"),
                        help="second goal after ten seconds of first final hover")
    parser.add_argument("--goal-yaw-offset", type=float, default=0.0,
                        help="requested goal yaw offset relative to current measured yaw (rad)")
    parser.add_argument("--drop-super-after", type=float,
                        help="fault test: stop this case's bridge after N sim seconds of first goal")
    args = parser.parse_args()
    if not args.name.replace("_", "").isalnum():
        parser.error("case name must contain only letters, digits and underscores")
    if not 10 <= args.hover_seconds <= 120 or not 1.0 <= args.height <= 3.0:
        parser.error("hover-seconds must be 10..120 and height 1..3 m")
    if not 1024 <= args.gazebo_port <= 65535:
        parser.error("invalid Gazebo port")
    if args.super_goal and (not all(math.isfinite(v) for v in args.super_goal) or
                           math.hypot(*args.super_goal[:2]) > 1.0 or
                           not 1.0 <= args.super_goal[2] <= 2.5 or
                           args.height != 1.5):
        parser.error("SUPER probe requires XY offset <=1 m, Z 1..2.5 m, takeoff 1.5 m")
    if args.goal_body_frame and not args.super_goal:
        parser.error("goal-body-frame requires super-goal")
    if args.super_hold_yaw and not args.super_goal:
        parser.error("super-hold-yaw requires super-goal")
    if args.next_super_goal and (not args.super_goal or
            not all(math.isfinite(v) for v in args.next_super_goal) or
            math.hypot(*args.next_super_goal[:2]) > 1.0 or
            not 1.0 <= args.next_super_goal[2] <= 2.5):
        parser.error("next-super-goal requires super-goal, XY <=1 m and Z 1..2.5 m")
    if not math.isfinite(args.goal_yaw_offset) or abs(args.goal_yaw_offset) > 1.5:
        parser.error("goal-yaw-offset must be within +/-1.5 rad")
    if args.drop_super_after is not None and (not args.super_goal or args.next_super_goal or
            not math.isfinite(args.drop_super_after) or not 0.1 <= args.drop_super_after <= 1.0):
        parser.error("drop-super-after requires one SUPER goal and a time 0.1..1 sim seconds")
    if any(port_busy(p) for p in (11311, args.gazebo_port, 4560)):
        parser.error("ROS, Gazebo or PX4 port occupied; existing session untouched")
    if any(process_running(name) for name in
           ("px4", "gzserver", "roscore", "rosmaster", "mavros_node")):
        parser.error("an existing simulation is running; existing session untouched")
    if not (BUILD / "bin/px4").is_file():
        parser.error("PX4 SITL is not built")
    folder = ROOT / "build/empty_arm_hover" / args.name
    if folder.exists():
        parser.error("case directory exists; choose a new name")
    folder.mkdir(parents=True)
    (folder / "rootfs").mkdir()
    make_world(folder / "scene.world")
    env = os.environ.copy()
    env.update(
        ROS_MASTER_URI="http://127.0.0.1:11311",
        ROS_HOSTNAME="localhost",
        GAZEBO_MASTER_URI=f"http://127.0.0.1:{args.gazebo_port}",
        GAZEBO_MODEL_PATH=f"{MODELS}:{env.get('GAZEBO_MODEL_PATH', '')}",
        GAZEBO_PLUGIN_PATH=(
            f"{ROS_WS}/devel/lib:{BUILD}/build_gazebo-classic:"
            f"/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:"
            f"{env.get('GAZEBO_PLUGIN_PATH', '')}"
        ),
        LD_LIBRARY_PATH=(
            f"{ROS_WS}/devel/lib:{BUILD}/build_gazebo-classic:"
            f"/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:"
            f"{env.get('LD_LIBRARY_PATH', '')}"
        ),
        PX4_SIM_MODEL="gazebo-classic_yhang550",
        PX4_SYS_AUTOSTART="1020",
    )
    env.pop("ROS_IP", None)
    procs = []
    logs = []
    result = dict(success=False, vehicle_fixed=False, payload=False,
                  super_goal=bool(args.super_goal), target_height_m=args.height,
                  super_hold_yaw=args.super_hold_yaw)
    rows = []
    trace = None
    goal_time = None
    final_hold_start = None
    goal_runs = []
    fault_injected = False
    failsafe_start = None
    wall_start = time.monotonic()

    def start(name, command, cwd=None):
        logfile = (folder / f"{name}.log").open("w")
        logs.append(logfile)
        proc = sp.Popen(command, cwd=cwd, env=env, stdout=logfile,
                        stderr=sp.STDOUT, start_new_session=True)
        procs.append(proc)
        return proc

    def wait_for(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if any(proc.poll() is not None for proc in procs):
                raise RuntimeError(f"process exited while waiting for {label}")
            if predicate():
                return
            time.sleep(0.2)
        raise RuntimeError(f"timeout waiting for {label}")

    try:
        start("roscore", ["roscore"])
        wait_for(rosgraph.is_master_online, 20, "roscore")
        sp.run(["rosparam", "set", "/use_sim_time", "true"], env=env,
               check=True, timeout=10)
        rospy.init_node("empty_arm_hover_probe", anonymous=True,
                        disable_signals=True)
        trace = SetpointTrace(folder)
        telemetry = Telemetry(trace)
        goal_pub = rospy.Publisher("/planning/user_goal", PoseStamped, queue_size=1)

        def send_goal(row, spec):
            nonlocal goal_time, final_hold_start
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "world"
            dx, dy = spec[:2]
            q = telemetry.odom.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            if args.goal_body_frame:
                dx, dy = (math.cos(yaw) * dx - math.sin(yaw) * dy,
                          math.sin(yaw) * dx + math.cos(yaw) * dy)
            goal.pose.position.x = row["odom_x"] + dx
            goal.pose.position.y = row["odom_y"] + dy
            goal.pose.position.z = spec[2]
            goal.pose.orientation.z = math.sin((yaw + args.goal_yaw_offset) / 2)
            goal.pose.orientation.w = math.cos((yaw + args.goal_yaw_offset) / 2)
            result["requested_goal"] = dict(x=goal.pose.position.x,
                                            y=goal.pose.position.y, z=goal.pose.position.z)
            goal_time = row["sim_s"]
            final_hold_start = None
            goal_runs.append(dict(requested_goal=dict(result["requested_goal"]), sent_sim_s=goal_time))
            trace.event("user_goal_sent", **result["requested_goal"])
            goal_pub.publish(goal)
            print(f"SUPER goal {len(goal_runs)} at sim {goal_time:.2f}s: {result['requested_goal']}", flush=True)
        start("gazebo", ["gzserver", "--verbose", "--seed", "123",
                         "-s", "libgazebo_ros_api_plugin.so",
                         str(folder / "scene.world")])
        start("px4", [str(BUILD / "bin/px4"), "-d", str(BUILD / "etc")],
              cwd=folder / "rootfs")
        start("mavros", ["roslaunch", "mavros", "px4.launch",
                         "fcu_url:=udp://:14540@127.0.0.1:14580"])
        wait_for(lambda: telemetry.state is not None and
                 telemetry.state.connected and telemetry.odom is not None and
                 telemetry.joints is not None, 100, "MAVROS/odom/joints")
        wait_for(lambda: math.isfinite(telemetry.truth_z), 20,
                 "Gazebo vehicle truth pose")
        folded_since = None

        def folded_stable():
            nonlocal folded_since
            row = telemetry.row()
            stable = (row is not None and row["sim_s"] > 2.0 and
                      row["max_fold_error_rad"] < 0.05 and
                      row["max_fold_speed_rad_s"] < 0.05 and
                      row["speed_m_s"] < 0.5)
            if not stable:
                folded_since = None
            elif folded_since is None:
                folded_since = row["sim_s"]
            return stable and row["sim_s"] - folded_since >= 2.0

        wait_for(folded_stable, 45, "two seconds of folded, stationary pre-arm state")
        prearm = telemetry.row()
        result["prearm"] = {
            key: None if isinstance(prearm[key], float) and
            not math.isfinite(prearm[key]) else prearm[key]
            for key in FIELDS
        }
        if args.super_goal:
            (folder / "launch_snapshot.xml").write_text(NAVIGATION_LAUNCH.read_text())
            (folder / "config_snapshot.yaml").write_text(SUPER_CONFIG.read_text())
            navigation_launch = make_navigation_probe(folder, args.super_hold_yaw)
            start("navigation", ["roslaunch", str(navigation_launch)])
        else:
            start("flight_manager", [
                "rosrun", "super_px4_bridge", "super_flight_manager_node",
                f"_takeoff_height:={args.height}", "_auto_arm:=true",
                "_hover_stable_time:=2.0",
            ])
        hold_start = None
        settled_since = None
        armed_since = None
        last_sim = -1.0
        last_progress = time.monotonic()
        with (folder / "telemetry.csv").open("w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=FIELDS)
            writer.writeheader()
            while True:
                if time.monotonic() - wall_start > 360:
                    raise RuntimeError("wall-time watchdog exceeded")
                if any(proc.poll() is not None for proc in procs):
                    raise RuntimeError("simulation process exited")
                row = telemetry.row()
                if row and row["sim_s"] > last_sim:
                    last_sim = row["sim_s"]
                    last_progress = time.monotonic()
                    writer.writerow(row)
                    csvfile.flush()
                    rows.append(row)
                    if row["sim_s"] > 140:
                        raise RuntimeError("takeoff/hover sim-time limit exceeded")
                    if not math.isfinite(row["truth_z"]):
                        raise RuntimeError("Gazebo vehicle truth pose missing/non-finite")
                    if goal_time is not None and row["odom_z"] < 0.5:
                        raise RuntimeError("unsafe descent below 0.5 m during SUPER probe")
                    if row["max_fold_error_rad"] > 0.10:
                        raise RuntimeError("folded arm departed target by over 0.10 rad")
                    if row["armed"] and armed_since is None:
                        armed_since = row["sim_s"]
                    if armed_since is not None and row["sim_s"] - armed_since > 3:
                        if max(abs(row["roll_deg"]), abs(row["pitch_deg"])) > 25:
                            raise RuntimeError("unsafe tilt above 25 degrees")
                        if math.hypot(row["odom_x"] - prearm["odom_x"],
                                      row["odom_y"] - prearm["odom_y"]) > 1.5:
                            raise RuntimeError("unsafe horizontal drift above 1.5 m")
                        if row["sim_s"] - armed_since > 12 and row["odom_z"] < 0.2:
                            raise RuntimeError("failed to climb after 12 sim seconds")
                    if row["armed"] and row["mode"] == "OFFBOARD" and hold_start is None:
                        stable = (math.isfinite(row["target_z"]) and
                                  abs(row["target_z"] - args.height) < 0.01 and
                                  abs(row["odom_z"] - args.height) < 0.15 and
                                  row["speed_m_s"] < 0.25)
                        if not stable:
                            settled_since = None
                        elif settled_since is None:
                            settled_since = row["sim_s"]
                        elif row["sim_s"] - settled_since >= 2.0:
                            hold_start = row["sim_s"]
                            print(f"Stable hover began at sim {hold_start:.2f}s",
                                  flush=True)
                    if hold_start is not None:
                        if not row["armed"] or row["mode"] != "OFFBOARD":
                            raise RuntimeError("lost armed/OFFBOARD during hover")
                        if not args.super_goal:
                            if row["sim_s"] - hold_start >= args.hover_seconds:
                                break
                        elif goal_time is None and row["sim_s"] - hold_start >= 3 and \
                                trace.manager_state == "HOVER_READY" and \
                                goal_pub.get_num_connections() > 0:
                            send_goal(row, args.super_goal)
                        if goal_time is not None:
                            if args.drop_super_after is not None and not fault_injected and \
                                    trace.manager_state == "TRACK_SUPER" and \
                                    row["sim_s"] - goal_time >= args.drop_super_after:
                                sp.run(["rosnode", "kill", "/super_mavros_bridge_node"],
                                       env=env, check=True, timeout=10,
                                       stdout=sp.DEVNULL, stderr=sp.DEVNULL)
                                fault_injected = True
                                trace.event("bridge_stopped")
                            if trace.manager_state == "FAILSAFE_HOVER":
                                if not fault_injected:
                                    raise RuntimeError("SUPER trajectory entered FAILSAFE_HOVER")
                                if failsafe_start is None:
                                    failsafe_start = row["sim_s"]
                                if row["sim_s"] - failsafe_start >= 5:
                                    break
                            with trace.lock:
                                current_final = any(e.get("new") == "FINAL_HOVER" and
                                                    e["stamp_s"] >= goal_time for e in trace.events)
                            if trace.manager_state == "FINAL_HOVER" and current_final:
                                if final_hold_start is None:
                                    final_hold_start = row["sim_s"]
                                    print(f"FINAL_HOVER at sim {final_hold_start:.2f}s", flush=True)
                                if args.next_super_goal and len(goal_runs) == 1 and \
                                        row["sim_s"] - final_hold_start >= 10:
                                    goal_runs[-1].update(hold_start=final_hold_start,
                                                         hold_end=row["sim_s"])
                                    send_goal(row, args.next_super_goal)
                                elif row["sim_s"] - final_hold_start >= args.hover_seconds:
                                    goal_runs[-1].update(hold_start=final_hold_start,
                                                         hold_end=row["sim_s"])
                                    break
                            elif row["sim_s"] - goal_time > 65:
                                raise RuntimeError("SUPER handover not completed within 65 sim seconds")
                if time.monotonic() - last_progress > 30:
                    raise RuntimeError("simulation clock stalled")
                time.sleep(0.1)
        if args.drop_super_after is not None:
            result.update(expected_failsafe=True, fault_injected=fault_injected,
                          failsafe_observed_sim_s=failsafe_start,
                          yaw_trace=yaw_trace_summary(trace))
            with trace.lock:
                false_success = any(e.get("new") == "FINAL_HOVER" and e["stamp_s"] >= goal_time
                                    for e in trace.events)
            result["success"] = (fault_injected and failsafe_start is not None and not false_success and
                                 result["yaw_trace"]["bounded"] and rows[-1]["speed_m_s"] <= 0.3 and
                                 abs(rows[-1]["odom_z"] - rows[-1]["target_z"]) <= 0.25)
            if not result["success"]:
                raise RuntimeError("fault test failed: missing failsafe or unstable hold")
            return # finally still records the result and cleans only our processes.
        measured_start = final_hold_start if args.super_goal else hold_start
        final_handover = handover_summary(trace, goal_time) if args.super_goal else None
        measured_z = final_handover["first_hold_z_m"] if args.super_goal else args.height
        hover = [r for r in rows if r["sim_s"] >= measured_start + 5.0]
        errors = [abs(r["odom_z"] - measured_z) for r in hover]
        max_tilt = max(max(abs(r["roll_deg"]), abs(r["pitch_deg"])) for r in hover)
        result.update(
            flight_completed=True,
            measured_target_height_m=measured_z,
            hover_metrics_exclude_first_s=5.0,
            hover_sim_seconds=rows[-1]["sim_s"] - measured_start,
            mean_abs_z_error_m=sum(errors) / len(errors),
            max_abs_z_error_m=max(errors),
            max_tilt_deg=max_tilt,
            max_fold_error_rad=max(r["max_fold_error_rad"] for r in hover),
            sample_count=len(rows),
        )
        result["success"] = (
            result["mean_abs_z_error_m"] <= 0.10 and
            result["max_abs_z_error_m"] <= 0.25 and
            max_tilt <= 15.0 and
            result["max_fold_error_rad"] <= 0.05
        )
        if args.super_goal:
            result["handover"] = final_handover
            for case in goal_runs:
                case["handover"] = handover_summary(trace, case["sent_sim_s"])
                target_z = case["handover"].get("first_hold_z_m", case["requested_goal"]["z"])
                window = [r for r in rows if case["hold_start"] + 5 <= r["sim_s"] <= case["hold_end"]]
                case["mean_abs_z_error_m"] = sum(abs(r["odom_z"] - target_z) for r in window) / len(window)
                case["max_abs_z_error_m"] = max(abs(r["odom_z"] - target_z) for r in window)
            result["goal_runs"] = goal_runs
            result["yaw_trace"] = yaw_trace_summary(trace)
            if not all(case["handover"].get("continuous", False) for case in goal_runs):
                result["success"] = False
                result["reason"] = "position setpoint discontinuity or missing handover"
            if not result["yaw_trace"]["bounded"]:
                result["success"] = False
                result["reason"] = "final command yaw rate exceeded 0.35 rad/s"
            if any(case["mean_abs_z_error_m"] > 0.10 or case["max_abs_z_error_m"] > 0.25
                   for case in goal_runs):
                result["success"] = False
                result["reason"] = "a final hover window exceeded height error limits"
    except (RuntimeError, KeyboardInterrupt, OSError, sp.SubprocessError) as exc:
        result["reason"] = str(exc) or "operator interrupted"
    finally:
        if trace is not None:
            if goal_time is not None:
                result["handover"] = handover_summary(trace, goal_time)
                result["yaw_trace"] = yaw_trace_summary(trace)
                result["goal_runs"] = goal_runs
            trace.close()
            rospy.signal_shutdown("probe complete")
        for proc in reversed(procs):
            stop_group(proc)
        for logfile in logs:
            logfile.close()
        if rows:
            result["last_sample"] = {
                key: None if isinstance(value, float) and
                not math.isfinite(value) else value
                for key, value in rows[-1].items()
            }
            result["max_recorded_tilt_deg"] = max(
                max(abs(r["roll_deg"]), abs(r["pitch_deg"])) for r in rows)
            result["max_recorded_z_m"] = max(r["odom_z"] for r in rows)
        result["elapsed_wall_s"] = time.monotonic() - wall_start
        result["ulog_files"] = [str(p) for p in (folder / "rootfs/log").rglob("*.ulg")]
        (folder / "summary.json").write_text(json.dumps(result, indent=2,
                                                       allow_nan=False))
        print(json.dumps(result, indent=2, allow_nan=False), flush=True)
    if not result["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
