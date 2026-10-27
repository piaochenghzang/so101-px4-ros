#!/usr/bin/env python3
"""S3 input isolation, geometry recovery and ambiguity/sparsity counterexamples."""
import copy
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS
import numpy as np
import yaml

BRIDGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BRIDGE/'scripts/perception'))
from static_target import (shape_from_geometry,estimate,large_plane_patch_mask,model_samples,
    rotation_deg,rotation_angle,validate_config)


class StaticTargetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry=yaml.safe_load((BRIDGE/'config/perception/mid360s_s1.yaml').read_text())
        cls.cfg=yaml.safe_load((BRIDGE/'config/perception/static_target.yaml').read_text())
        cls.parts=shape_from_geometry(cls.geometry)

    def test_world_pose_and_installation_cannot_enter_local_model(self):
        g=copy.deepcopy(self.geometry)
        g['target']['position_world_m']=[float('nan')]*3
        g['target']['quaternion_world_wxyz']=[0]*4
        g['vehicle']='not_a_transform'; g['observations']='not_a_goal'
        changed=shape_from_geometry(g)
        for a,b in zip(self.parts,changed):
            self.assertEqual(a['name'],b['name'])
            for key in ('half','center','R'): np.testing.assert_array_equal(a[key],b[key])

    def test_distant_coplanar_target_fragment_survives_large_plane(self):
        rng=np.random.default_rng(3)
        y,z=np.meshgrid(np.linspace(-.5,.5,41),np.linspace(-.5,.5,41))
        wall=np.column_stack([np.zeros(y.size),y.ravel(),z.ravel()])
        small=np.column_stack([np.zeros(80),rng.uniform(1.95,2.05,80),rng.uniform(-.04,.04,80)])
        mask=large_plane_patch_mask(np.r_[wall,small],self.cfg)
        self.assertTrue(mask[:len(wall)].all())
        self.assertFalse(mask[len(wall):].any())

    def test_recover_independent_dense_shape_pose(self):
        local,_,_=model_samples(self.parts,.012)
        # Pose is supplied only to generate synthetic measured XYZ, never as an initial estimate.
        R=rotation_deg([10,-12,32]); t=np.array([1.1,.12,-.08]); points=local@R.T+t
        r,_,_=estimate(points,self.parts,self.cfg)
        self.assertTrue(r['valid'],r['reasons'])
        self.assertLess(np.linalg.norm(np.array(r['best']['position_lidar_m'])-t),.003)
        self.assertLess(rotation_angle(np.array(r['best']['R']).T@R),3.)
        self.assertFalse(r['control_authorized']); self.assertFalse(r['grasp_ready'])

    def test_body_without_asymmetric_features_is_rejected(self):
        body=[p for p in self.parts if p['name']=='body']
        local,_,_=model_samples(body,.01); R=rotation_deg([0,0,20]); t=np.array([1.1,0,0])
        r,_,_=estimate(local@R.T+t,self.parts,self.cfg)
        self.assertFalse(r['valid'])

    def test_sparse_empty_and_nonsensical_points_reject(self):
        for points in (np.empty((0,3)),np.array([[1.,0,0],[1.01,0,0]])):
            r,_,_=estimate(points,self.parts,self.cfg)
            self.assertFalse(r['valid']); self.assertIsNone(r['best'])
        for p in (np.array([[1,0,np.nan]]),np.zeros(3)):
            with self.assertRaises(ValueError): estimate(p,self.parts,self.cfg)

    def test_gravity_and_invalid_settings_reject(self):
        for up in ([0,0,0],[0,0,float('nan')],[0,0,9.81]):
            with self.assertRaises(ValueError): estimate(np.zeros((0,3)),self.parts,self.cfg,up)
        for key,value in [('cluster_radius_m',0),('roi_min_m',[5,0,0]),('upright_tilt_limit_deg',180),
                          ('yaw_seed_step_deg',17),('min_inlier_fraction',float('nan'))]:
            cfg=copy.deepcopy(self.cfg); cfg[key]=value
            with self.assertRaises(ValueError): validate_config(cfg)

    def test_raw_reader_accesses_only_measurements_and_rejects_gaps(self):
        sys.path.insert(0,str(BRIDGE/'tests'))
        from test_mid360s_static import fixture
        clouds,_,imus,*_=fixture()
        rows=[('/mid360s_sim/points',m,None) for m in clouds]+[('/mid360s_sim/imu',m,None) for m in imus]
        reads=[]
        class Bag:
            def __init__(self,*args): pass
            def __enter__(self): return self
            def __exit__(self,*args): pass
            def read_messages(self,topics):
                reads.append(topics)
                return iter(rows)
        with patch.dict(sys.modules,{'rosbag':NS(Bag=Bag)}):
            spec=importlib.util.spec_from_file_location('raw_reader_under_test',BRIDGE/'scripts/perception/run_static_target.py')
            module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
            points,source=module.read_raw(Path('/not/a/scene'),.1,1.)
            self.assertEqual(source['observation_end_ns'],1_200_000_000)
            self.assertFalse(source['restamped'])
            self.assertEqual(len(points),30)
            self.assertEqual(reads[-1],['/mid360s_sim/points','/mid360s_sim/imu'])
            rows.reverse()
            reordered,reordered_source=module.read_raw(Path('/not/a/scene'),.1,1.)
            np.testing.assert_array_equal(reordered,points)
            self.assertGreater(reordered_source['bag_imu_header_order_reversals'],0)
            rows.reverse()
            rows.pop(4)
            with self.assertRaisesRegex(ValueError,'raw frames'): module.read_raw(Path('/not/a/scene'),.1,1.)


if __name__=='__main__': unittest.main()
