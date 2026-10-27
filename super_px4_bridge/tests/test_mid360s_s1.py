#!/usr/bin/env python3
"""S1 transform/occlusion and measured-parameter counterexamples, offline only."""
import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import yaml

BRIDGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('s1', BRIDGE/'scripts/perception/check_installation.py')
s1 = importlib.util.module_from_spec(spec); spec.loader.exec_module(s1)
CONFIG = BRIDGE/'config/perception/mid360s_s1.yaml'
ROOT = Path(sys.argv.pop(1)).expanduser() if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else Path.home()/'PX4-RoboMaster'


class S1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = s1.load_profile(CONFIG)
        cls.arm = s1.load_arm(ROOT)

    def report(self, c=None):
        # Share the existing expensive IK route, not its geometric verdict.
        with patch.object(s1, 'load_arm', return_value=self.arm):
            return s1.check(c or self.config, ROOT)

    def test_mount_origin_and_direct_calibration_override(self):
        np.testing.assert_allclose(s1.sensor_transform(self.config)[:3, 3], [0, 0, .160])
        c = copy.deepcopy(self.config)
        c['lidar']['body_from_lidar_override_xyz_rpy_deg'] = [.02, -.01, .175, 2, -4, 90]
        np.testing.assert_allclose(s1.sensor_transform(c), s1.transform([.02, -.01, .175, 2, -4, 90]))
        # Full rigid inverse, including nonzero tilt and lever arm.
        p = np.array([1., .3, -.2, 1.])
        np.testing.assert_allclose(s1.inverse(s1.sensor_transform(c))@s1.sensor_transform(c)@p, p, atol=1e-12)

    def test_tilt_changes_fov_and_not_only_height(self):
        c = copy.deepcopy(self.config)
        c['target']['observation_parts'] = ['body']
        T = s1.transform([2, 0, 0, 0, 0, 0]); parts = dict(body=(T, np.ones(3)*.01))
        level = s1.visibility(c, np.eye(4), parts, [])['body']
        pitched = s1.visibility(c, s1.transform([0, 0, 0, 0, -20, 0]), parts, [])['body']
        self.assertTrue(level['all_surface_samples_in_fov'])
        self.assertFalse(pitched['all_surface_samples_in_fov'])
        self.assertLess(pitched['elevation_max_deg'], -19)

    def test_segment_box_occlusion_does_not_hide_endpoint_surface(self):
        T = s1.transform([1, 0, 0, 0, 0, 45]); h = np.array([.1, .1, .1])
        self.assertTrue(s1.blocked(np.zeros(3), np.array([[2., 0, 0]]), T, h)[0])
        self.assertFalse(s1.blocked(np.zeros(3), np.array([[.5, 0, 0]]), T, h)[0])
        self.assertFalse(s1.blocked(np.zeros(3), np.array([[2., 0, 1.]]), T, h)[0])
        self.assertFalse(s1.blocked(np.zeros(3), np.array([[.9, 0, 0]]), s1.transform([1, 0, 0, 0, 0, 0]), h)[0])

    def test_occluded_object_is_not_a_candidate_merely_due_to_fov(self):
        c = copy.deepcopy(self.config); c['target']['observation_parts'] = ['body']
        parts = dict(body=(s1.transform([2, 0, 0, 0, 0, 0]), np.ones(3)*.05))
        free = s1.visibility(c, np.eye(4), parts, [])['body']
        hidden = s1.visibility(c, np.eye(4), parts, [(s1.transform([1, 0, 0, 0, 0, 0]), np.ones(3)*.2)])['body']
        self.assertGreater(free['sampled_visible_fraction'], 0)
        self.assertEqual(hidden['sampled_visible_fraction'], 0)
        self.assertTrue(hidden['all_surface_samples_in_fov'])

    def test_baseline_and_grasp_blind_zone_are_both_reported(self):
        r = self.report()
        self.assertTrue(r['s1_geometric_precheck_passed'])
        self.assertAlmostEqual(r['mounting']['exposed_plate_top_area_m2'], .010175)
        self.assertGreater(r['arm_geometry']['min_arm_to_body_or_key_m'], .035)
        for case in r['selected'].values():
            self.assertTrue(case['geometric_candidate'])
            self.assertGreaterEqual(case['robust_min_fov_margin_deg'], 1.)
        for v in r['grasp_pose_view'].values():
            self.assertEqual(v['sampled_visible_fraction'], 0.)
        for key in ('motion_permitted', 'scan_simulated', 'hardware_calibrated', 'runtime_profile_updated'):
            self.assertFalse(r[key])

    def test_measured_arm_mount_translation_changes_required_hover(self):
        r = self.report(); c = copy.deepcopy(self.config)
        c['vehicle']['body_from_arm_xyz_rpy_deg'][0] += .02
        new = self.report(c)
        np.testing.assert_allclose(np.array(new['arm_geometry']['hover_body_world_xyz_m'])-
            r['arm_geometry']['hover_body_world_xyz_m'], [-.02, 0, 0], atol=1e-12)
        np.testing.assert_allclose(new['arm_geometry']['tcp_arm_m'], r['arm_geometry']['tcp_arm_m'])

    def test_large_body_and_incorrect_target_rotation_reject(self):
        c = copy.deepcopy(self.config); c['target']['parts'][0]['size_m'] = [.4, .4, .4]
        r = self.report(c)
        self.assertFalse(r['s1_geometric_precheck_passed'])
        self.assertLess(r['arm_geometry']['worst_pair']['slack_m'], 0.)
        c = copy.deepcopy(self.config); c['target']['quaternion_world_wxyz'] = [1., 0, 0, 0]
        r = self.report(c)
        self.assertFalse(r['s1_geometric_precheck_passed'])
        self.assertGreater(r['arm_geometry']['orientation_error_deg'], 5.)

    def test_undersized_plate_and_mount_overlap_reject(self):
        c = copy.deepcopy(self.config); c['lidar']['plate_size_m'] = [.08, .08, .003]
        self.assertFalse(self.report(c)['mounting']['dimensional_plate_check_passed'])
        c = copy.deepcopy(self.config); c['lidar']['body_from_base_xyz_rpy_deg'][2] = .03
        self.assertFalse(self.report(c)['mounting']['dimensional_plate_check_passed'])

    def test_unvalidated_handle_width_is_not_accepted_as_cube20(self):
        c = copy.deepcopy(self.config); c['target']['parts'][-1]['size_m'][0] = .03
        self.assertFalse(self.report(c)['arm_geometry']['handle_section_matches_tested_20mm'])

    def test_grasp_point_outside_handle_is_not_a_reachable_grasp(self):
        c = copy.deepcopy(self.config)
        c['target']['object_from_grasp_xyz_rpy_deg'][0] = .3
        r = self.report(c)
        self.assertFalse(r['arm_geometry']['handle_section_matches_tested_20mm'])
        self.assertFalse(r['s1_geometric_precheck_passed'])

    def test_bad_measured_values_fail_before_geometry(self):
        for group, key, value in [('lidar', 'body_from_lidar_override_xyz_rpy_deg', [0, 0, float('nan'), 0, 0, 0]),
                ('lidar', 'housing_size_m', [.065, -.065, .06]),
                ('lidar', 'vertical_fov_deg', [52., -7.]),
                ('target', 'quaternion_world_wxyz', [0., 0, 0, 0]),
                ('observations', 'surface_grid_per_edge', 1)]:
            c = copy.deepcopy(self.config); c[group][key] = value
            with tempfile.TemporaryDirectory() as d:
                path = Path(d)/'measured.yaml'; path.write_text(yaml.safe_dump(c))
                with self.assertRaises(ValueError, msg=key): s1.load_profile(path)


if __name__ == '__main__':
    unittest.main()
