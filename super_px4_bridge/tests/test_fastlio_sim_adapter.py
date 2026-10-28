#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest import mock
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


def pose(position,roll=0.,pitch=0.,yaw=0.):
    # Construct test poses in the documented Z-Y-X Euler convention.
    sr,cr=np.sin(roll),np.cos(roll)
    sp,cp=np.sin(pitch),np.cos(pitch)
    sy,cy=np.sin(yaw),np.cos(yaw)
    rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    T=np.eye(4);T[:3,:3]=rz@ry@rx;T[:3,3]=position
    return T


class LioOriginAlignment(unittest.TestCase):
    def test_initial_translation_and_heading_are_zeroed(self):
        for position,yaw in [([12.,-7.,2.5],.7),([-4.,8.,-1.],-2.4),([.1,-.2,.3],np.pi)]:
            with self.subTest(position=position,yaw=yaw):
                first=pose(position,yaw=yaw);original=first.copy()
                aligned=m.lio_origin_alignment(first)@first
                np.testing.assert_allclose(aligned,np.eye(4),atol=1e-12)
                np.testing.assert_array_equal(first,original)

    def test_preserves_roll_pitch_and_gravity_direction(self):
        gravity=np.array([0.,0.,-1.])
        for roll,pitch,yaw in [(.23,-.31,1.2),(-.4,.27,-2.8),(.08,.04,.6)]:
            with self.subTest(roll=roll,pitch=pitch,yaw=yaw):
                first=pose([3.,-5.,7.],roll,pitch,yaw)
                alignment=m.lio_origin_alignment(first)
                np.testing.assert_allclose(alignment@first,pose([0.,0.,0.],roll,pitch),atol=1e-12)
                np.testing.assert_allclose(alignment[:3,:3]@gravity,gravity,atol=1e-12)
                # A yaw-only change of map frame cannot change gravity seen by the body.
                np.testing.assert_allclose((alignment@first)[:3,:3].T@gravity,first[:3,:3].T@gravity,atol=1e-12)

    def test_later_displacement_and_rotation_use_same_fixed_map(self):
        first=pose([10.,-4.,2.],yaw=np.pi/2)
        later=pose([8.,-1.,3.],yaw=np.pi)
        alignment=m.lio_origin_alignment(first)
        # In the initial heading frame, world displacement (-2,+3,+1)
        # becomes (+3,+2,+1), and a further quarter turn stays a quarter turn.
        np.testing.assert_allclose(alignment@later,pose([3.,2.,1.],yaw=np.pi/2),atol=1e-12)
        np.testing.assert_allclose(np.linalg.inv(alignment@first)@(alignment@later),
                                   np.linalg.inv(first)@later,atol=1e-12)

    def test_rejects_nonfinite_and_nonrigid_poses(self):
        invalid={'shape':np.eye(3),'zero_matrix':np.zeros((4,4))}
        for key,value in [('nan',float('nan')),('inf',float('inf'))]:
            T=np.eye(4);T[0,3]=value;invalid[key+'_translation']=T
            T=np.eye(4);T[1,1]=value;invalid[key+'_rotation']=T
        T=np.eye(4);T[3,2]=.1;invalid['homogeneous_row']=T
        T=np.eye(4);T[0,0]=1.01;invalid['scale']=T
        T=np.eye(4);T[0,1]=.01;invalid['shear']=T
        T=np.eye(4);T[0,0]=-1.;invalid['reflection']=T
        for reason,T in invalid.items():
            with self.subTest(reason=reason):
                with self.assertRaises(ValueError):m.lio_origin_alignment(T)


class AlignmentPolicy(unittest.TestCase):
    def ros_modules(self,params):
        rospy=types.ModuleType('rospy')
        rospy.get_param=mock.Mock(side_effect=lambda key,default:params.get(key,default))
        rospy.Publisher=mock.Mock();rospy.Subscriber=mock.Mock()
        modules={'rospy':rospy}
        for package,names in [('nav_msgs',['Odometry']),('geometry_msgs',['PoseWithCovarianceStamped']),
                              ('sensor_msgs',['PointCloud2']),('std_msgs',['String'])]:
            parent=types.ModuleType(package);messages=types.ModuleType(package+'.msg')
            for name in names:setattr(messages,name,type(name,(),{}))
            parent.msg=messages;modules[package]=parent;modules[package+'.msg']=messages
        installation=types.ModuleType('check_installation')
        installation.load_profile=mock.Mock(return_value={})
        installation.sensor_transform=mock.Mock(return_value=np.eye(4))
        modules['check_installation']=installation
        return rospy,modules

    def test_lio_origin_never_subscribes_to_mavros_alignment(self):
        for publish in [False,True]:
            with self.subTest(publish_to_fcu=publish):
                rospy,modules=self.ros_modules({'~alignment_source':'lio_origin','~publish_to_fcu':publish})
                with mock.patch.dict(sys.modules,modules):node=m.Adapter()
                topics=[call.args[0] for call in rospy.Subscriber.call_args_list]
                self.assertCountEqual(topics,['/mid360s_sim/points','/Odometry','/fastlio/cloud_registered'])
                self.assertEqual(node.map_frame,'map')
                self.assertIsNone(node.local)
                publishers=[call.args[0] for call in rospy.Publisher.call_args_list]
                self.assertEqual('/mavros/vision_pose/pose_cov' in publishers,publish)

    def test_default_mode_keeps_existing_mavros_alignment(self):
        rospy,modules=self.ros_modules({})
        with mock.patch.dict(sys.modules,modules):node=m.Adapter()
        self.assertIn('/mavros/local_position/odom',[call.args[0] for call in rospy.Subscriber.call_args_list])
        self.assertIsNone(node.fcu_pub)
        self.assertEqual(node.map_frame,'world')

    def test_fcu_publish_rejects_mavros_alignment_before_creating_ros_io(self):
        rospy,modules=self.ros_modules({'~alignment_source':'stationary_mavros','~publish_to_fcu':True})
        with mock.patch.dict(sys.modules,modules):
            with self.assertRaisesRegex(ValueError,'cannot align against its own MAVROS output'):m.Adapter()
        rospy.Publisher.assert_not_called();rospy.Subscriber.assert_not_called()

    def test_unknown_alignment_source_is_rejected(self):
        rospy,modules=self.ros_modules({'~alignment_source':'unknown'})
        with mock.patch.dict(sys.modules,modules):
            with self.assertRaisesRegex(ValueError,'Unknown alignment source'):m.Adapter()
        rospy.Publisher.assert_not_called();rospy.Subscriber.assert_not_called()


if __name__=='__main__':unittest.main()
