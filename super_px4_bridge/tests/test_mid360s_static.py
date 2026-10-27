#!/usr/bin/env python3
"""Static cloud source-time, geometry and no-target counterexamples; no ROS master."""
import copy
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import yaml

BRIDGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BRIDGE/'scripts/perception'))
from scan_metrics import cloud_array, point_times, analyse
from run_static_scan import scene_file, cases
from check_installation import load_profile, transform


def stamp(ns):
    return NS(to_nsec=lambda:ns,to_sec=lambda:ns*1e-9)


def cloud(ns, truth=False, only_background=False):
    schema=[('x',0,7),('y',4,7),('z',8,7)]
    schema+=([('label',12,6),('sample_index',16,6),('timestamp',20,8),('offset_time',28,6)] if truth else
             [('intensity',12,7),('tag',16,2),('line',17,2),('timestamp',18,8),('offset_time',26,6)])
    points=[([1.94,0,0],1),([2.085,.02,.1],2),([3,2,0],5)]
    if only_background: points=points[-1:]
    data=bytearray(len(points)*32)
    for i,(p,label) in enumerate(points):
        offset=(i+1)*1_000_000; base=i*32
        struct.pack_into('<fff',data,base,*p)
        struct.pack_into('<d',data,base+(20 if truth else 18),float(ns+offset))
        struct.pack_into('<I',data,base+(28 if truth else 26),offset)
        if truth: struct.pack_into('<I',data,base+12,label)
    return NS(header=NS(stamp=stamp(ns),frame_id='mid360s_lidar'),height=1,width=len(points),point_step=32,
        row_step=len(points)*32,is_bigendian=False,data=bytes(data),
        fields=[NS(name=n,offset=o,datatype=t,count=1) for n,o,t in schema])


def fixture():
    parts=[dict(name='body',label=1,pose=transform([2,0,0,0,0,0]).tolist(),size_m=[.12,.08,.06]),
           dict(name='key',label=2,pose=transform([2.1,.02,.1,0,0,0]).tolist(),size_m=[.03,.025,.02]),
           dict(name='handle',label=3,pose=transform([2,0,.15,0,0,0]).tolist(),size_m=[.02,.02,.07])]
    scene=dict(parts=parts,scoring_boxes=parts,lidar_world=np.eye(4).tolist(),fov_deg=[-7,52])
    cfg=dict(frame_rate_hz=10,imu_rate_hz=200,warmup_s=.1,duration_s=1.,integration_windows_s=[.2,.5,1.],coverage_cell_m=.005)
    clouds=[cloud(i*100_000_000) for i in range(1,14)]
    truths=[cloud(i*100_000_000,True) for i in range(1,14)]
    imus=[NS(header=NS(stamp=stamp(i*5_000_000),frame_id='mid360s_lidar'),orientation_covariance=[-1]*9,
        angular_velocity=NS(x=0,y=0,z=0),linear_acceleration=NS(x=0,y=0,z=9.81)) for i in range(300)]
    return clouds,truths,imus,scene,cfg,dict(name='primary')


class StaticTests(unittest.TestCase):
    def test_unaligned_driver2_fields_and_point_nanoseconds(self):
        m=cloud(1_000_000_000); a=cloud_array(m)
        np.testing.assert_array_equal(point_times(m,a,100_000_000),[1001000000,1002000000,1003000000])
        self.assertNotIn('label',a.dtype.names)
        self.assertEqual(a['timestamp'][0],1001000000.)

    def test_wrong_time_units_and_duplicate_offsets_reject(self):
        for offset,value,fmt in [(18,1.001,'<d'),(26,100_000_000,'<I'),(58,1_000_000,'<I')]:
            m=cloud(1_000_000_000); b=bytearray(m.data); struct.pack_into(fmt,b,offset,value); m.data=bytes(b)
            with self.assertRaises(ValueError): point_times(m,cloud_array(m),100_000_000)

    def test_incomplete_layout_reject(self):
        m=cloud(1_000_000_000); m.data=m.data[:-1]
        with self.assertRaises(ValueError): cloud_array(m)
        m=cloud(1_000_000_000); m.fields.append(m.fields[-1])
        with self.assertRaises(ValueError): cloud_array(m)

    def test_source_time_windows_and_static_force(self):
        r=analyse(*fixture())
        self.assertTrue(r['static_case_passed']); self.assertEqual(r['frames'],10)
        self.assertEqual(r['target_return_count'],20)
        for row in r['integration']:
            self.assertAlmostEqual(row['summary']['body']['returns']['mean'],10*row['window_s'])
        self.assertLessEqual(r['max_imu_bracket_s'],.005)

    def test_missing_frame_and_stale_imu_reject(self):
        args=list(fixture()); args[0].pop(4)
        with self.assertRaisesRegex(ValueError,'cloud frame'): analyse(*args)
        args=list(fixture()); args[2]=args[2][:120]
        with self.assertRaisesRegex(ValueError,'bracketed'): analyse(*args)
        args=list(fixture()); args[2][20].linear_acceleration.z=-9.81
        with self.assertRaisesRegex(ValueError,'specific-force'): analyse(*args)

    def test_surface_pose_error_does_not_pass_as_cloud_success(self):
        args=list(fixture()); args[3]['parts'][0]['pose'][0][3]+=.1
        with self.assertRaises(ValueError): analyse(*args)

    def test_expected_occlusion_and_unexpected_target_absence(self):
        args=list(fixture())
        args[0]=[cloud(i*100_000_000,only_background=True) for i in range(1,14)]
        args[1]=[cloud(i*100_000_000,True,True) for i in range(1,14)]
        self.assertFalse(analyse(*args)['static_case_passed'])
        args[-1]['name']='occluded'
        self.assertTrue(analyse(*args)['static_case_passed'])

    def test_generated_scene_is_static_and_has_unique_link_names(self):
        c=load_profile(BRIDGE/'config/perception/mid360s_s1.yaml')
        cfg=yaml.safe_load((BRIDGE/'config/perception/mid360s_static.yaml').read_text())
        report=dict(selected=dict(primary=dict(body_world_xyz_m=[3.5,0,1.25])),arm_geometry=dict(hover_body_world_xyz_m=[4.8,0,1.95]))
        arm=(NS(arm=np.eye(4),boxes=lambda q,j:[]),np.zeros(5))
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'scene.world'
            scene_file(c,cfg,report,arm,cases(cfg)['primary'],path,Path('/unused/trace.csv.gz'))
            tree=ET.parse(path)
            self.assertTrue(all(m.findtext('static')=='true' for m in tree.findall('world/model')))
            for link in tree.findall('.//link'):
                names=[e.get('name') for e in link if e.tag in ('collision','visual')]
                self.assertEqual(len(names),len(set(names)))
            self.assertEqual(len(tree.findall('world/plugin')),1)
            self.assertNotIn('mavlink',path.read_text().lower())
            self.assertNotIn('controller',path.read_text().lower())


if __name__=='__main__': unittest.main()
