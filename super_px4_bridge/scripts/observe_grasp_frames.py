#!/usr/bin/env python3
"""Read-only sim-time coordinate audit. No Publisher, ServiceProxy or actuation."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import threading
import time
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import rospkg
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from super_px4_bridge.msg import CoordinateTruth
from grasp_audit.geometry import (FrozenOrigin, IdentityGuard, audit_truth, current_goal,
                                  finite, interpolate_pose, inverse, pose_transform, rpy_transform)


def pose_values(p):
    return [p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]


class Observer:
    def __init__(self, config, output):
        self.config, self.output = config, output
        self.lock = threading.RLock()
        self.origin = FrozenOrigin(config)
        self.guard = IdentityGuard()
        self.pending = deque(maxlen=200)
        self.odom = deque(maxlen=400)
        self.joints = deque(maxlen=400)
        self.last_clock = None
        self.session_start = None
        self.latest_request = 0
        self.resolved_stamp = 0
        self.resolved_position = None
        self.last_progress_wall = time.monotonic()
        self.counts = dict(samples=0, valid_truth=0, valid_chain=0, valid_local=0, target_present=0)
        self.subscribers = [
            rospy.Subscriber('/grasp_audit/truth', CoordinateTruth, self.truth, queue_size=100),
            rospy.Subscriber('/mavros/local_position/odom', Odometry, self.on_odom, queue_size=100),
            rospy.Subscriber('/so101/joint_states', JointState, self.on_joints, queue_size=100),
            rospy.Subscriber('/planning/resolved_goal', PoseStamped, self.on_resolved, queue_size=10),
            rospy.Subscriber('/planning/user_goal', PoseStamped, self.on_request, queue_size=10),
            rospy.Subscriber('/clock', Clock, self.clock, queue_size=100),
        ]

    def emit(self, report):
        self.output.write(json.dumps(report, allow_nan=False, sort_keys=True)+'\n')
        self.output.flush()

    def reset(self):
        self.origin.reset(); self.odom.clear(); self.joints.clear()
        self.resolved_stamp = 0; self.resolved_position = None
        self.latest_request = 0

    def clock(self, msg):
        stamp = msg.clock.to_sec()
        with self.lock:
            if self.session_start is None and stamp > 0:
                self.session_start = stamp
            if self.last_clock is not None and stamp < self.last_clock:
                self.reset(); self.pending.clear(); self.guard = IdentityGuard()
                self.session_start = stamp
                self.emit(dict(kind='reset', reason='CLOCK_RESET', stamp_s=stamp, close_permitted=False))
            if self.last_clock != stamp:
                self.last_progress_wall = time.monotonic()
            self.last_clock = stamp

    def truth(self, msg):
        with self.lock:
            stamp = msg.header.stamp.to_sec()
            if not math.isfinite(stamp) or stamp <= 0 or (self.last_clock is not None and stamp > self.last_clock+.02):
                self.emit(dict(kind='invalid', reason='INVALID_OR_FUTURE_TRUTH_TIME', close_permitted=False))
                self.origin.reset(); return
            if not (len(msg.link_names) == len(msg.link_ids) == len(msg.link_poses)) or len(set(msg.link_names)) != len(msg.link_names):
                self.emit(dict(kind='invalid', reason='INVALID_LINK_ARRAY', close_permitted=False))
                self.reset(); return
            packet = dict(stamp=stamp, frame=msg.header.frame_id,
                          instance=msg.source_instance, epoch=msg.world_epoch,
                          vehicle_name=msg.vehicle_name, vehicle_id=msg.vehicle_id,
                          joint_measurement_valid=msg.joint_measurement_valid,
                          links={n:dict(id=i, pose=pose_values(p)) for n, i, p in zip(msg.link_names,msg.link_ids,msg.link_poses)},
                          joint_names=list(msg.joint_names), position=list(msg.joint_position),
                          velocity=list(msg.joint_velocity), target_name=msg.target_name,
                          target_id=msg.target_id, target_present=msg.target_present,
                          target_pose=pose_values(msg.target_pose))
            if len(self.pending) == self.pending.maxlen:
                self.emit(dict(kind='invalid', reason='QUEUE_OVERFLOW', close_permitted=False))
                self.reset(); self.pending.clear()
            self.pending.append(packet)

    def on_odom(self, msg):
        with self.lock:
            try:
                if msg.header.frame_id != self.config['odom_frame'] or msg.child_frame_id != self.config['body_frame']:
                    raise ValueError('ODOM_FRAME_MISMATCH')
                values = pose_values(msg.pose.pose); pose_transform(values)
                stamp = msg.header.stamp.to_sec()
                if stamp <= 0 or not math.isfinite(stamp):
                    raise ValueError('INVALID_ODOM_TIME')
                if self.odom and stamp <= self.odom[-1][0]:
                    raise ValueError('ODOM_NONMONOTONIC')
                self.odom.append((stamp, values))
            except ValueError as error:
                self.origin.reset(); self.odom.clear()
                self.emit(dict(kind='invalid', reason=str(error), close_permitted=False))

    def on_joints(self, msg):
        with self.lock:
            try:
                names = list(msg.name)
                if len(names) != 6 or len(set(names)) != 6 or set(names) != set(self.config['joint_names']):
                    raise ValueError('ROS_JOINT_NAMES')
                q = finite(msg.position, (6,)); finite(msg.velocity, (6,))
                stamp = msg.header.stamp.to_sec()
                if stamp <= 0 or not math.isfinite(stamp) or (self.joints and stamp <= self.joints[-1][0]):
                    raise ValueError('ROS_JOINT_NONMONOTONIC')
                self.joints.append((stamp, [float(q[names.index(n)]) for n in self.config['joint_names']]))
            except ValueError as error:
                self.joints.clear()
                self.emit(dict(kind='invalid', reason=str(error), close_permitted=False))

    def on_request(self, msg):
        with self.lock:
            self.latest_request = rospy.Time.now().to_sec()
            self.resolved_stamp = 0; self.resolved_position = None

    def on_resolved(self, msg):
        with self.lock:
            try:
                if msg.header.frame_id != 'world':
                    raise ValueError('RESOLVED_FRAME_MISMATCH')
                self.resolved_position = finite(pose_values(msg.pose)[:3], (3,)).tolist()
                self.resolved_stamp = msg.header.stamp.to_sec()
            except ValueError:
                self.resolved_stamp = 0; self.resolved_position = None

    def process(self, packet, now):
        report = dict(kind='sample', stamp_s=packet['stamp'], epoch=packet['epoch'],
                      source_instance=packet['instance'], target_id=packet['target_id'],
                      close_permitted=False, local_valid=False)
        self.counts['samples'] += 1
        try:
            identity = (packet['instance'], packet['epoch'], packet['vehicle_id'], packet['target_id'],
                        tuple(sorted((n, v['id']) for n,v in packet['links'].items())))
            if not packet['instance'] or packet['epoch'] <= 0:
                raise ValueError('INVALID_SOURCE_IDENTITY')
            if self.guard.check(identity, packet['stamp']):
                self.reset(); self.session_start = packet['stamp']
                raise ValueError('IDENTITY_OR_CLOCK_CHANGED')
            age = now-packet['stamp']
            if age < -.02 or age > self.config['source_age_limit_s']:
                raise ValueError('TRUTH_STALE_OR_FUTURE')
            result, body = audit_truth(packet, self.config)
            report.update(result, source_age_s=age)
            self.counts['valid_truth'] += 1
            self.counts['valid_chain'] += int(result['chain_valid'])
            self.counts['target_present'] += int(result['target_present'])
            # Ancillary comparison only: old ROS publisher stamps ROS clock at UpdateBegin.
            # Do not represent nearest received angles as exact physical-step synchronization.
            if self.joints:
                closest = min(self.joints, key=lambda row:abs(row[0]-packet['stamp']))
                skew = abs(closest[0]-packet['stamp'])
                if now-closest[0] < self.config['joint_age_limit_s'] and skew <= self.config['source_skew_limit_s']:
                    q = [packet['position'][packet['joint_names'].index(n)] for n in self.config['joint_names']]
                    report.update(ros_joint_comparison='APPROXIMATE_ROS_CLOCK',
                                  ros_joint_skew_s=skew, ros_joint_max_difference_rad=float(np.max(np.abs(np.asarray(q)-closest[1]))))
            try:
                local_body, skew = interpolate_pose(list(self.odom), packet['stamp'], self.config['source_skew_limit_s'])
                if not result['chain_valid']:
                    raise ValueError('CHAIN_INVALID')
                errors = self.origin.update(local_body, body)
                report.update(odom_source_skew_s=skew, local_origin_diagnostic_only=True,
                              origin_samples=len(self.origin.candidates))
                if errors is None:
                    report['local_reason'] = 'ORIGIN_CALIBRATING'
                else:
                    transform = self.origin.transform
                    report.update(local_valid=True, origin_G_to_L=transform.tolist(),
                                  origin_position_residual_m=errors[0], origin_rotation_residual_rad=errors[1])
                    self.counts['valid_local'] += 1
                    if packet['target_present']:
                        obj = transform@pose_transform(packet['target_pose'])
                        # Estimator-chain result is compared with independent truth-chain result.
                        arm_local = local_body@rpy_transform(self.config['mount_pose'])
                        object_arm = (inverse(arm_local)@obj)[:3,3]
                        delta = float(np.linalg.norm(object_arm-result['object_arm']))
                        report.update(object_local=obj[:3,3].tolist(),
                                      object_arm_from_local=object_arm.tolist(),
                                      object_arm_local_truth_difference_m=delta,
                                      local_precision_consistent=bool(delta <= self.config['fk_position_limit_m']
                                          and errors[0] <= self.config['fk_position_limit_m']
                                          and errors[1] <= self.config['rotation_limit_rad']))
            except ValueError as error:
                report['local_reason'] = str(error)
                if str(error) != 'ODOM_NOT_BRACKETED':
                    self.origin.reset()
            report['resolved_goal_current'] = current_goal(self.resolved_stamp, self.session_start or now, self.latest_request, now)
            if report['resolved_goal_current']:
                report['resolved_goal_local'] = self.resolved_position
            # Neither coordinate consistency nor a current goal is a grasp permission.
        except (ValueError, KeyError, IndexError) as error:
            report.update(truth_valid=False, chain_valid=False, aligned_geometry_only=False, reason=str(error))
            self.origin.reset()
        self.emit(report)

    def drain(self):
        with self.lock:
            if self.last_clock is None: return
            # Leave 40 ms for source-stamped odometry to bracket the truth sample.
            while self.pending and self.pending[0]['stamp'] <= self.last_clock-.04:
                self.process(self.pending.popleft(), self.last_clock)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='New JSONL file; never overwrite')
    parser.add_argument('--duration', type=float, default=20, help='Simulation seconds to observe')
    parser.add_argument('--wall-timeout', type=float, default=120)
    args = parser.parse_args(rospy.myargv()[1:])
    if not all(math.isfinite(x) and x > 0 for x in (args.duration,args.wall_timeout)):
        parser.error('durations must be finite and positive')
    rospy.init_node('grasp_frame_observer', anonymous=True)
    if not rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Observation requires use_sim_time=true; observer will not change it')
    config_path = args.config or Path(rospkg.RosPack().get_path('super_px4_bridge'))/'config/grasp_frame_audit.yaml'
    config = yaml.safe_load(config_path.read_text())
    if not isinstance(config['origin_samples'], int) or config['origin_samples'] < 2:
        raise ValueError('origin_samples must be >=2')
    for key in ('fk_position_limit_m', 'rotation_limit_rad', 'source_age_limit_s',
                'source_skew_limit_s', 'joint_age_limit_s', 'origin_position_scatter_m',
                'origin_rotation_scatter_rad', 'alignment_diagnostic_limit_m'):
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError('Invalid diagnostic parameter: '+key)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as output:
        observer = Observer(config, output)
        observer.emit(dict(kind='manifest', config=config, observation_only=True, close_permitted=False))
        started = time.monotonic()
        outcome = 'WALL_TIMEOUT'
        try:
            while not rospy.is_shutdown() and time.monotonic()-started < args.wall_timeout:
                observer.drain()
                with observer.lock:
                    if observer.session_start is not None and observer.last_clock-observer.session_start >= args.duration:
                        outcome='OBSERVATION_COMPLETE'; break
                    if time.monotonic()-observer.last_progress_wall > 10:
                        outcome='STALLED'; break
                time.sleep(.005)
        finally:
            with observer.lock:
                observer.emit(dict(kind='summary', outcome=outcome, counts=observer.counts,
                                   close_permitted=False, grasp_tested=False))
            for subscriber in observer.subscribers: subscriber.unregister()
    return 0 if outcome == 'OBSERVATION_COMPLETE' and observer.counts['valid_chain'] > 0 else 2


if __name__ == '__main__':
    raise SystemExit(main())
