#!/usr/bin/env python3
"""Synthetic math/failure regression only; not physical grasp evidence."""
import copy
import math
from pathlib import Path
import sys
import unittest
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from grasp_audit.geometry import (FrozenOrigin, IdentityGuard, audit_truth, current_goal,
                                  fk_gripper, interpolate_pose, inverse, pose_transform,
                                  quaternion_rotation, residual, rpy_transform)


def as_pose(t):
    # Test conversion via eigenvector for arbitrary positive/negative quaternion branch.
    r = t[:3,:3]
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]])/3
    _, vectors = np.linalg.eigh(k)
    x,y,z,w = vectors[:,-1]
    return [*t[:3,3], w,x,y,z]


class FrameChecks(unittest.TestCase):
    def setUp(self):
        self.c = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/grasp_frame_audit.yaml').read_text())

    def packet(self, body=None, angles=None):
        body = rpy_transform([.7,-.3,1.25,.07,-.05,1.1]) if body is None else body
        angles = np.array([-.07,-1.175,1.45,1.3,-1.57,.3]) if angles is None else angles
        arm = body@rpy_transform(self.c['mount_pose'])
        gripper = arm@fk_gripper(self.c, angles)
        obj = gripper.copy()
        obj[:3,3] += gripper[:3,:3]@self.c['reference_gripper']
        links = dict(zip([self.c[k] for k in ('body_link','arm_link','gripper_link','lidar_link')],
                         [dict(id=i+10,pose=as_pose(t)) for i,t in enumerate(
                          [body,arm,gripper,body@rpy_transform(self.c['lidar_mount_pose'])])]))
        return dict(frame='gazebo_world', vehicle_name='yhang550', vehicle_id=2,
                    joint_measurement_valid=True,
                    target_name='test_cube', target_id=50, target_present=True,
                    target_pose=as_pose(obj), links=links, joint_names=self.c['joint_names'],
                    position=list(angles), velocity=[0]*6)

    def test_full_chain_random_rotations_and_object_reference(self):
        rng = np.random.default_rng(101)
        for _ in range(100):
            body = rpy_transform([*rng.uniform(-5,5,3), *rng.uniform(-1,1,3)])
            q = rng.uniform(-1,1,6)
            out, _ = audit_truth(self.packet(body,q),self.c)
            self.assertTrue(out['chain_valid'])
            self.assertLess(out['reference_error_m'],1e-12)
            self.assertFalse(out['close_permitted'])

    def test_inverse_roundtrip_full_rpy_and_nonzero_origin(self):
        t = rpy_transform([3,-2,.5,.2,-.3,2.4])
        p = np.array([.2,.7,.1,1])
        np.testing.assert_allclose(inverse(t)@(t@p),p,atol=1e-12)

    def test_wrong_axis_or_mount_does_not_pass(self):
        packet = self.packet()
        config = copy.deepcopy(self.c); config['mount_pose'][3] = 0
        self.assertFalse(audit_truth(packet,config)[0]['chain_valid'])
        config = copy.deepcopy(self.c); config['joint_origins'][2][0] *= -1
        self.assertFalse(audit_truth(packet,config)[0]['chain_valid'])

    def test_wrong_frame_missing_link_names_and_nan(self):
        mutations = [lambda p:p.update(frame='world'),
                     lambda p:p['links'].pop(self.c['arm_link']),
                     lambda p:p['position'].__setitem__(0,float('nan')),
                     lambda p:p.update(joint_names=['gripper']*6),
                     lambda p:p.update(target_name='other_cube'),
                     lambda p:p.update(joint_measurement_valid=False),
                     lambda p:p['links'][self.c['body_link']].update(id=0)]
        for mutate in mutations:
            p = self.packet(); mutate(p)
            with self.assertRaises(ValueError): audit_truth(p,self.c)

    def test_zero_nan_or_misnormalized_quaternion_rejected(self):
        for q in ([0,0,0,0], [math.nan,0,0,1], [2,0,0,0]):
            with self.assertRaises(ValueError): quaternion_rotation(q)

    def test_missing_target_is_not_alignment_or_permission(self):
        p = self.packet(); p['target_present'] = False
        out,_ = audit_truth(p,self.c)
        self.assertTrue(out['chain_valid']); self.assertFalse(out['aligned_geometry_only'])
        self.assertFalse(out['close_permitted']); self.assertEqual(out['reason'],'TARGET_MISSING')

    def test_identity_restart_reset_and_duplicate(self):
        guard = IdentityGuard()
        self.assertFalse(guard.check(('source',1,2,50),1))
        with self.assertRaises(ValueError): guard.check(('source',1,2,50),1)
        self.assertTrue(guard.check(('source',1,2,50),.5))
        self.assertTrue(guard.check(('source',2,2,51),.6))
        self.assertTrue(guard.check(('new_source',1,2,51),.7))

    def test_frozen_origin_does_not_refit_away_later_error(self):
        origin = FrozenOrigin(self.c)
        transform = rpy_transform([1,2,.3,0,0,.4])
        for i in range(self.c['origin_samples']):
            body = rpy_transform([i*.02,-.2,1,0,0,i*.1])
            origin.update(transform@body,body)
        np.testing.assert_allclose(origin.transform,transform,atol=1e-12)
        shifted = transform.copy(); shifted[2,3] += .1
        self.assertGreater(origin.update(shifted@body,body)[0],.09)
        np.testing.assert_allclose(origin.transform,transform,atol=1e-12)
        origin.reset(); self.assertIsNone(origin.transform)

    def test_scattering_origin_does_not_calibrate(self):
        origin = FrozenOrigin(self.c)
        for i in range(30): origin.update(rpy_transform([i*.1,0,0,0,0,0]),np.eye(4))
        self.assertIsNone(origin.transform)

    def test_source_time_interpolation_and_antipodal_quaternions(self):
        a = [0,0,1,1,0,0,0]
        b = [.02,0,1,-1,0,0,0]
        t,skew = interpolate_pose([(1,a),(1.02,b)],1.01,.02)
        np.testing.assert_allclose(t[:3,3],[.01,0,1],atol=1e-12)
        self.assertAlmostEqual(skew,.01)
        for stamp,limit in [(1.03,.02),(1.01,.005)]:
            with self.assertRaises(ValueError): interpolate_pose([(1,a),(1.02,b)],stamp,limit)

    def test_old_latched_goal_is_not_current(self):
        self.assertFalse(current_goal(5,10,0,15))
        self.assertFalse(current_goal(12,10,13,15))
        self.assertFalse(current_goal(20,10,13,15))
        self.assertTrue(current_goal(14,10,13,15))

    def test_interpolation_rejects_ambiguous_or_invalid_source_times(self):
        pose = [0,0,0,1,0,0,0]
        for samples in ([(1,pose),(1,pose)],[(1.02,pose),(1,pose)],
                        [(float('nan'),pose)],[(0,pose)]):
            with self.assertRaises(ValueError): interpolate_pose(samples,1,.02)
        for stamp,limit in ((float('nan'),.02),(0,.02),(1,0),(1,float('nan'))):
            with self.assertRaises(ValueError): interpolate_pose([(1,pose)],stamp,limit)


if __name__ == '__main__': unittest.main()
