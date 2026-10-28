#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import numpy as np
path=Path(__file__).resolve().parents[1]/'scripts/perception/fastlio_sim_adapter.py'
spec=importlib.util.spec_from_file_location('adapter',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class TimedCloud(unittest.TestCase):
    def points(self):
        a=np.zeros(3,dtype=[(k,'f4') for k in ('x','y','z','intensity')]+[('line','u1'),('offset_time','u4')])
        a['x']=[1,2,3];a['offset_time']=[0,50000000,99995000];a['line']=[0,1,3];return a
    def test_preserve_points_lines_and_nanosecond_offsets(self):
        a=self.points();b=m.timed_points(a)
        np.testing.assert_allclose(b['x'],a['x']);np.testing.assert_array_equal(b['ring'],a['line'])
        np.testing.assert_allclose(b['time'],[0,.05,.099995],atol=1e-8)
        self.assertEqual(b.dtype.itemsize,24)
    def test_reject_bad_source_time_and_fields(self):
        for field,bad in [('offset_time',[0,100000000,100000001]),('offset_time',[0,5,1]),('line',[0,4,0]),('x',[1,float('nan'),2])]:
            a=self.points();a[field]=bad
            with self.assertRaises(ValueError):m.timed_points(a)
        with self.assertRaises(ValueError):m.timed_points(self.points()[:0])
if __name__=='__main__':unittest.main()
