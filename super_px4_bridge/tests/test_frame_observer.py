#!/usr/bin/env python3
"""Synthetic callback regression; no master, simulator or command publication."""
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from super_px4_bridge.msg import CoordinateTruth

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from observe_grasp_frames import Observer
from grasp_audit.geometry import rpy_transform
import test_grasp_frames as fixtures


class ObserverChecks(unittest.TestCase):
    def setUp(self):
        fixture = fixtures.FrameChecks(); fixture.setUp()
        self.config,self.packet = fixture.c,fixture.packet
        self.output = io.StringIO()
        with patch('observe_grasp_frames.rospy.Subscriber'):
            self.observer = Observer(self.config,self.output)
        self.observer.session_start = 1

    def message(self, stamp, pose, frame='map'):
        msg = Odometry(); msg.header.stamp=rospy.Time.from_sec(stamp)
        msg.header.frame_id=frame; msg.child_frame_id='base_link'
        p = msg.pose.pose
        p.position.x,p.position.y,p.position.z=pose[:3]
        p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z=pose[3:]
        return msg

    def sample(self, stamp=1, epoch=1, origin=None):
        packet = self.packet(); packet.update(stamp=stamp,instance='synthetic',epoch=epoch)
        if origin is not None:
            body = rpy_transform([.7,-.3,1.25,.07,-.05,1.1])
            values = fixtures.as_pose(origin@body)
            self.observer.on_odom(self.message(stamp-.01,values))
            self.observer.on_odom(self.message(stamp+.01,values))
        self.observer.process(packet,stamp+.04)
        return json.loads(self.output.getvalue().splitlines()[-1])

    def test_local_chain_nonzero_origin_with_no_publication_or_service(self):
        origin = rpy_transform([2,-1,.3,0,0,.6])
        with patch('observe_grasp_frames.rospy.Publisher') as pub, patch('observe_grasp_frames.rospy.ServiceProxy') as service:
            for i in range(10): report = self.sample(1+i*.03,origin=origin)
            pub.assert_not_called(); service.assert_not_called()
        self.assertTrue(report['local_valid'])
        self.assertTrue(report['local_precision_consistent'])
        self.assertLess(report['object_arm_local_truth_difference_m'],1e-12)
        self.assertFalse(report['close_permitted'])

    def test_local_frame_missing_or_unsynchronized_not_claimed_valid(self):
        report = self.sample()
        self.assertFalse(report['local_valid']); self.assertEqual(report['local_reason'],'ODOM_NOT_BRACKETED')
        pose = [0,0,0,1,0,0,0]
        self.observer.on_odom(self.message(1.10,pose))
        self.observer.on_odom(self.message(1.20,pose))
        report = self.sample(1.15)
        self.assertFalse(report['local_valid']); self.assertEqual(report['local_reason'],'SOURCE_SKEW')

    def test_wrong_odom_frame_invalidates_registration(self):
        self.observer.origin.transform=np.eye(4)
        self.observer.on_odom(self.message(1,[0,0,0,1,0,0,0],frame='gazebo_world'))
        self.assertIsNone(self.observer.origin.transform)
        self.assertFalse(self.observer.odom)
        self.assertIn('ODOM_FRAME_MISMATCH',self.output.getvalue())

    def test_entity_change_clears_calibration_and_goals(self):
        self.sample(1)
        self.observer.origin.transform=np.eye(4)
        self.observer.resolved_stamp=1
        report = self.sample(1.02,epoch=2)
        self.assertFalse(report['truth_valid']); self.assertIsNone(self.observer.origin.transform)
        self.assertEqual(self.observer.resolved_stamp,0)

    def test_clock_rollback_clears_pending_and_origin(self):
        self.observer.last_clock=10
        self.observer.pending.append(dict(stamp=10))
        self.observer.origin.transform=np.eye(4)
        clock = Clock(); clock.clock=rospy.Time.from_sec(.5)
        self.observer.clock(clock)
        self.assertFalse(self.observer.pending); self.assertIsNone(self.observer.origin.transform)
        self.assertIn('CLOCK_RESET',self.output.getvalue())

    def test_future_truth_cannot_block_pending_queue(self):
        self.observer.last_clock=1
        msg = CoordinateTruth(); msg.header.stamp=rospy.Time.from_sec(100)
        self.observer.truth(msg)
        self.assertFalse(self.observer.pending)
        self.assertIn('INVALID_OR_FUTURE_TRUTH_TIME',self.output.getvalue())

    def test_duplicate_and_stale_truth_rejected(self):
        self.sample(1)
        report = self.sample(1)
        self.assertFalse(report['truth_valid']); self.assertEqual(report['reason'],'DUPLICATE_SOURCE')
        packet = self.packet(); packet.update(stamp=2,instance='synthetic',epoch=1)
        self.observer.process(packet,3)
        report = json.loads(self.output.getvalue().splitlines()[-1])
        self.assertFalse(report['truth_valid']); self.assertEqual(report['reason'],'TRUTH_STALE_OR_FUTURE')


if __name__ == '__main__': unittest.main()
