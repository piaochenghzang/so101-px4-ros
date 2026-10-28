#!/usr/bin/env python3
"""Simulation bridge: timed MID360 proxy -> unmodified FAST-LIO -> SUPER.
The one-time map alignment uses stationary MAVROS odometry, never object truth.
PX4 EKF localization remains explicitly selected by the experiment launcher.
"""
import copy
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np

OUTPUT_DTYPE=np.dtype(dict(names=['x','y','z','intensity','time','ring'],
    formats=['<f4']*5+['<u2'],offsets=[0,4,8,12,16,20],itemsize=24))

def timed_points(array):
    required={'x','y','z','intensity','line','offset_time'}
    if not required.issubset(array.dtype.names or ()) or len(array)==0:raise ValueError('Missing timed LiDAR fields')
    out=np.zeros(len(array),dtype=OUTPUT_DTYPE)
    for key in ('x','y','z','intensity'):out[key]=array[key]
    if not np.isfinite(np.column_stack([out[k] for k in ('x','y','z')])).all():raise ValueError('Nonfinite cloud')
    offsets=np.asarray(array['offset_time'],dtype=np.uint64)
    if offsets.max()>=100000000 or np.any(np.diff(offsets.astype(np.int64))<0):raise ValueError('Invalid 100ms point times')
    if np.any(array['line']>=4):raise ValueError('Invalid synthetic line')
    out['time']=offsets.astype(np.float64)*1e-9;out['ring']=array['line']
    return out

class Adapter:
    def __init__(self):
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from check_installation import load_profile,sensor_transform
        self.rospy=rospy;self.Odometry=Odometry;self.String=String
        self.mount=sensor_transform(load_profile(Path(__file__).resolve().parents[2]/"config/perception/mid360s_s1.yaml"));self.alignment=None;self.local=None;self.last=None
        self.raw_pub=rospy.Publisher('/fastlio_sim/points',PointCloud2,queue_size=2)
        self.odom_pub=rospy.Publisher('/lidar_slam/odom',Odometry,queue_size=20)
        self.cloud_pub=rospy.Publisher('/cloud_registered',PointCloud2,queue_size=2)
        self.alignment_pub=rospy.Publisher('/fastlio_sim/alignment',String,queue_size=1,latch=True)
        self.subs=[rospy.Subscriber(t,k,f,queue_size=n,tcp_nodelay=True) for t,k,f,n in
            [('/mid360s_sim/points',PointCloud2,self.raw,2),('/Odometry',Odometry,self.odom,20),
             ('/fastlio/cloud_registered',PointCloud2,self.cloud,2),
             ('/mavros/local_position/odom',Odometry,self.local_odom,20)]]

    @staticmethod
    def matrix(p):
        from tf.transformations import quaternion_matrix
        q=p.orientation;v=np.array([q.x,q.y,q.z,q.w])
        if not np.isfinite(v).all() or np.linalg.norm(v)<1e-8:raise ValueError('Invalid quaternion')
        T=quaternion_matrix(v)
        T[:3,3]=[p.position.x,p.position.y,p.position.z]
        if not np.isfinite(T).all():raise ValueError('Invalid pose')
        return T

    @staticmethod
    def set_pose(p,T):
        from tf.transformations import quaternion_from_matrix
        p.position.x,p.position.y,p.position.z=map(float,T[:3,3])
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quaternion_from_matrix(T))

    def local_odom(self,m):self.local=m

    def raw(self,m):
        from scan_metrics import cloud_array
        from sensor_msgs.msg import PointCloud2,PointField
        try:
            if m.header.frame_id!='mid360s_lidar':raise ValueError('Unexpected raw frame')
            data=timed_points(cloud_array(m))
            out=PointCloud2();out.header=m.header;out.height=1;out.width=len(data)
            out.fields=[PointField(k,OUTPUT_DTYPE.fields[k][1],PointField.UINT16 if k=='ring' else PointField.FLOAT32,1)
                for k in OUTPUT_DTYPE.names]
            out.point_step=24;out.row_step=24*len(data);out.is_dense=True;out.is_bigendian=False;out.data=data.tobytes()
            self.raw_pub.publish(out)
        except ValueError as e:self.rospy.logerr_throttle(2.,str(e))

    def odom(self,m):
        if m.header.frame_id!='camera_init' or m.child_frame_id!='body':return
        T=self.matrix(m.pose.pose);now=self.rospy.Time.now().to_sec();stamp=m.header.stamp.to_sec()
        if not -.02<=now-stamp<=.3:return
        if self.alignment is None:
            a=self.local
            if a is None or abs(a.header.stamp.to_sec()-stamp)>.10:return
            v=a.twist.twist.linear;w=a.twist.twist.angular
            if np.linalg.norm([v.x,v.y,v.z])>.03 or np.linalg.norm([w.x,w.y,w.z])>.05:return
            self.alignment=self.matrix(a.pose.pose)@self.mount@np.linalg.inv(T)
            self.alignment_pub.publish(self.String(json.dumps(dict(world_from_camera_init=self.alignment.tolist(),
                calibration_stamp_s=stamp,source='ONE_STATIONARY_MAVROS_POSE',continuous_truth_correction=False))))
        body=self.alignment@T@np.linalg.inv(self.mount)
        out=self.Odometry();out.header=copy.deepcopy(m.header);out.header.frame_id='world';out.child_frame_id='base_link'
        self.set_pose(out.pose.pose,body)
        if self.last and stamp>self.last[0]:
            v=body[:3,:3].T@(body[:3,3]-self.last[1])/(stamp-self.last[0])
            out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,v)
        self.last=(stamp,body[:3,3].copy());self.odom_pub.publish(out)

    def cloud(self,m):
        if self.alignment is None or m.header.frame_id!='camera_init':return
        from scan_metrics import cloud_array
        out=copy.deepcopy(m);array=cloud_array(m).copy()
        xyz=np.column_stack([array[k] for k in ('x','y','z')])
        xyz=xyz@self.alignment[:3,:3].T+self.alignment[:3,3]
        for i,k in enumerate(('x','y','z')):array[k]=xyz[:,i]
        out.header.frame_id='world';out.data=array.tobytes();self.cloud_pub.publish(out)

if __name__=='__main__':
    import rospy
    rospy.init_node('fastlio_sim_adapter');node=Adapter();rospy.spin()
