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
from mavros_msgs.msg import State
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

    def test_32ms_odom_21ms_truth_accumulates_without_weakening_gate(self):
        origin = rpy_transform([2,-1,.3,0,0,.6])
        body = rpy_transform([.7,-.3,1.25,.07,-.05,1.1])
        values = fixtures.as_pose(origin@body)
        for k in range(85): self.observer.on_odom(self.message(1+k*.032,values))
        reports = []
        for k in range(120): reports.append(self.sample(1.001+k*.021))
        valid = [r for r in reports if r['local_valid']]
        rejected = [r for r in reports if r.get('local_reason') == 'SOURCE_SKEW']
        self.assertGreater(len(valid),15); self.assertGreater(len(rejected),15)
        self.assertEqual(self.observer.origin_freezes,1)
        self.assertTrue(all(r['odom_source_skew_s'] <= .020000001 for r in valid))
        np.testing.assert_allclose(self.observer.origin.transform,origin,atol=1e-12)
        self.assertTrue(all(r['local_precision_consistent'] for r in valid))

    def test_rejected_brackets_preserve_candidates_and_frozen_origin(self):
        origin = rpy_transform([2,-1,.3,0,0,.6])
        body = rpy_transform([.7,-.3,1.25,.07,-.05,1.1])
        values = fixtures.as_pose(origin@body)
        for i in range(10):
            stamp=1+i*.08
            self.sample(stamp,origin=origin)
            self.observer.on_odom(self.message(stamp+.06,values))
            report=self.sample(stamp+.035)
            self.assertEqual(report['local_reason'],'SOURCE_SKEW')
        self.assertEqual(self.observer.origin_freezes,1)
        np.testing.assert_allclose(self.observer.origin.transform,origin,atol=1e-12)
        shifted=origin.copy(); shifted[2,3]+=.1
        report=self.sample(2,origin=shifted)
        self.assertTrue(report['local_valid']); self.assertFalse(report['local_precision_consistent'])
        self.assertGreater(report['origin_position_residual_m'],.09)
        self.assertGreater(report['tcp_position_residual_m'],.09)
        np.testing.assert_allclose(self.observer.origin.transform,origin,atol=1e-12)
        self.assertEqual(self.observer.summary('OBSERVATION_COMPLETE')['local_precision_verdict'],'FAIL')

    def freeze(self):
        origin=rpy_transform([2,-1,.3,0,0,.6])
        for i in range(10): self.sample(1+i*.03,origin=origin)
        return origin, fixtures.as_pose(origin@rpy_transform([.7,-.3,1.25,.07,-.05,1.1]))

    def test_duplicate_odom_drops_both_and_does_not_refit_origin(self):
        origin,values=self.freeze()
        self.observer.on_odom(self.message(1.5,values))
        changed=values.copy(); changed[0]+=.1
        self.observer.on_odom(self.message(1.5,changed))
        self.assertFalse(any(t==1.5 for t,_ in self.observer.odom))
        self.assertIn('ODOM_DUPLICATE_SOURCE',self.output.getvalue())
        np.testing.assert_allclose(self.observer.origin.transform,origin,atol=1e-12)
        self.observer.on_odom(self.message(1.5,values))
        self.assertFalse(any(t==1.5 for t,_ in self.observer.odom))
        self.assertTrue(self.sample(1.6,origin=origin)['local_valid'])

    def test_duplicate_calibration_support_latches_invalid_until_real_reset(self):
        origin,values=self.freeze()
        self.observer.on_odom(self.message(1.01,values))
        self.assertEqual(self.observer.origin_blocked_reason,'CALIBRATION_SOURCE_CONFLICT')
        report=self.sample(1.6,origin=origin)
        self.assertFalse(report['local_valid'])
        self.assertEqual(report['local_reason'],'CALIBRATION_SOURCE_CONFLICT')
        self.assertEqual(self.observer.summary('OBSERVATION_COMPLETE')['local_precision_verdict'],'INVALIDATED')
        self.observer.last_clock=2
        clock=Clock(); clock.clock=rospy.Time.from_sec(.5); self.observer.clock(clock)
        self.assertIsNone(self.observer.origin_blocked_reason)

    def test_late_future_and_bad_odom_drop_without_recalibration(self):
        origin,values=self.freeze()
        self.observer.on_odom(self.message(1.123,values))
        self.assertIn('ODOM_OUT_OF_ORDER',self.output.getvalue())
        self.observer.last_clock=2
        self.observer.on_odom(self.message(100,values))
        bad=values.copy(); bad[0]=float('nan')
        self.observer.on_odom(self.message(2,bad))
        self.assertIn('ODOM_STALE_OR_FUTURE',self.output.getvalue())
        np.testing.assert_allclose(self.observer.origin.transform,origin,atol=1e-12)

    def test_fcu_disconnect_invalidates_origin_and_pending(self):
        self.freeze()
        self.observer.pending.append(dict(stamp=2))
        self.observer.on_state(State(connected=True))
        self.observer.on_state(State(connected=False))
        self.assertIsNone(self.observer.origin.transform); self.assertFalse(self.observer.pending)
        self.assertEqual(self.observer.epoch_local_pairs,0)
        self.assertFalse(self.observer.summary('OBSERVATION_COMPLETE')['local_registration_current'])

    def test_delayed_future_endpoint_wait_is_bounded(self):
        origin=rpy_transform([2,-1,.3,0,0,.6])
        body=rpy_transform([.7,-.3,1.25,.07,-.05,1.1])
        values=fixtures.as_pose(origin@body)
        self.observer.on_odom(self.message(.99,values))
        packet=self.packet(); packet.update(stamp=1,instance='synthetic',epoch=1)
        self.observer.pending.append(packet)
        self.observer.last_clock=1.041; self.observer.drain()
        self.assertEqual(len(self.observer.pending),1)
        self.observer.on_odom(self.message(1.01,values))
        self.observer.last_clock=1.05; self.observer.drain()
        self.assertFalse(self.observer.pending)
        self.assertIn('ORIGIN_CALIBRATING',self.output.getvalue())
        packet=self.packet(); packet.update(stamp=1.1,instance='synthetic',epoch=1)
        self.observer.pending.append(packet)
        self.observer.last_clock=1.181; self.observer.drain()
        self.assertFalse(self.observer.pending)
        self.assertIn('ODOM_NOT_BRACKETED',self.output.getvalue())
        self.assertEqual(len(self.observer.origin.candidates),1)

    def test_missing_target_still_reports_body_tcp_precision(self):
        origin,_=self.freeze()
        packet=self.packet(); packet.update(stamp=1.4,instance='synthetic',epoch=1,target_present=False,target_id=50)
        values=fixtures.as_pose(origin@rpy_transform([.7,-.3,1.25,.07,-.05,1.1]))
        self.observer.on_odom(self.message(1.39,values)); self.observer.on_odom(self.message(1.41,values))
        self.observer.process(packet,1.44)
        report=json.loads(self.output.getvalue().splitlines()[-1])
        self.assertTrue(report['local_valid']); self.assertTrue(report['local_precision_consistent'])
        self.assertEqual(report['local_precision_scope'],'BODY_AND_TCP')
        self.assertFalse(report['aligned_geometry_only']); self.assertFalse(report['close_permitted'])

    def test_no_local_samples_summary_is_not_a_precision_pass(self):
        self.sample()
        report=self.observer.summary('OBSERVATION_COMPLETE')
        self.assertTrue(report['observation_complete']); self.assertTrue(report['chain_observed'])
        self.assertFalse(report['local_pairing_observed']); self.assertFalse(report['local_registration_current'])
        self.assertEqual(report['local_precision_verdict'],'NOT_EVALUATED')
        self.assertIsNone(report['max_local_position_residual_m'])

    def test_duplicate_stale_truth_does_not_rewind_identity_or_refit(self):
        origin,_=self.freeze()
        last_stamp=self.observer.guard.stamp
        report=self.sample(last_stamp)
        self.assertFalse(report['truth_valid']); self.assertEqual(report['reason'],'DUPLICATE_SOURCE')
        packet=self.packet(); packet.update(stamp=100,instance='synthetic',epoch=1)
        self.observer.process(packet,101)
        self.assertEqual(self.observer.guard.stamp,last_stamp)
        np.testing.assert_allclose(self.observer.origin.transform,origin,atol=1e-12)
        self.assertTrue(self.sample(1.4,origin=origin)['local_valid'])

    def test_source_identity_reset_requires_new_epoch_local_samples(self):
        origin,_=self.freeze()
        for i in range(10): self.sample(1.4+i*.03,origin=origin)
        self.assertTrue(self.observer.summary('OBSERVATION_COMPLETE')['local_registration_current'])
        self.sample(2,epoch=2)
        self.assertFalse(self.observer.summary('OBSERVATION_COMPLETE')['local_registration_current'])
        self.assertEqual(self.observer.epoch_local_pairs,0)
        self.assertIsNone(self.observer.origin.transform)

    def test_received_odom_delay_and_missing_samples_do_not_refit(self):
        origin=rpy_transform([2,-1,.3,0,0,.6])
        values=fixtures.as_pose(origin@rpy_transform([.7,-.3,1.25,.07,-.05,1.1]))
        # Physical/source stamps: odom 32ms, truth 21ms. Odom arrives 35ms
        # later, and two consecutive messages are absent. No re-stamping.
        deliveries=[(k*32+35,1+k*.032) for k in range(53) if k not in (20,21)]
        for tick in range(1850):
            stamp=1+tick*.001
            clock=Clock(); clock.clock=rospy.Time.from_sec(stamp); self.observer.clock(clock)
            for receive,source in deliveries:
                if tick==receive: self.observer.on_odom(self.message(source,values))
            if tick%21==0 and tick<=1659:
                packet=self.packet(); packet.update(stamp=stamp,instance='synthetic',epoch=1)
                self.observer.pending.append(packet)
            self.observer.drain()
        self.assertFalse(self.observer.pending)
        summary=self.observer.summary('OBSERVATION_COMPLETE')
        self.assertGreater(summary['counts']['valid_local'],10)
        self.assertTrue(summary['local_registration_current'])
        self.assertGreater(summary['local_reason_counts'].get('SOURCE_SKEW',0),0)
        self.assertEqual(summary['origin_freezes'],1); self.assertEqual(summary['origin_generation'],0)
        self.assertEqual(summary['local_precision_verdict'],'WITHIN_LIMITS_FOR_QUALIFIED_PAIRS')
        np.testing.assert_allclose(self.observer.origin.transform,origin,atol=1e-12)


if __name__ == '__main__': unittest.main()
