#!/usr/bin/env python3
"""Generate and record isolated STATIC S2 scenes. Never starts PX4 or commands an arm."""
import argparse
import csv
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess as sp
import threading
import time
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from check_installation import BRIDGE, check, load_arm, load_profile, sensor_transform, transform, inverse, pose_transform
from scan_metrics import analyse, cloud_array


def save(path, value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def pose_text(T):
    R=T[:3,:3]
    pitch=math.asin(np.clip(-R[2,0],-1,1))
    if abs(math.cos(pitch))<1e-8: raise ValueError('Preview Euler singularity')
    roll=math.atan2(R[2,1],R[2,2]); yaw=math.atan2(R[1,0],R[0,0])
    return ' '.join(map(str,[*T[:3,3],roll,pitch,yaw]))


def box(link, name, T, size, color=(.7,.7,.7,1)):
    for kind in ('collision','visual'):
        item=ET.SubElement(link,kind,name=name if kind=='collision' else name+'_visual')
        ET.SubElement(item,'pose').text=pose_text(T)
        ET.SubElement(ET.SubElement(ET.SubElement(item,'geometry'),'box'),'size').text=' '.join(map(str,size))
        if kind=='collision': ET.SubElement(item,'laser_retro').text='100'
        else:
            mat=ET.SubElement(item,'material')
            for tag in ('ambient','diffuse'): ET.SubElement(mat,tag).text=' '.join(map(str,color))


def model(world, name, T):
    m=ET.SubElement(world,'model',name=name); ET.SubElement(m,'static').text='true'
    ET.SubElement(m,'pose').text=pose_text(T)
    return m, ET.SubElement(m,'link',name='link')


def cases(cfg):
    sigma=cfg['range_noise_sigmas_m']
    if sigma != [0.,.01,.02]: raise ValueError('Use the declared 0/10/20 mm first sensitivity matrix')
    base=[dict(name=name,view=view,sigma=noise,seed=cfg['seed']) for name,view,noise in (
        ('primary','primary',sigma[0]),('primary_repeat','primary',sigma[0]),
        ('backup','backup',sigma[0]),('primary_noise','primary',sigma[1]),('backup_noise','backup',sigma[1]),
        ('primary_noise2','primary',sigma[2]),('backup_noise2','backup',sigma[2]),
        ('occluded','primary',sigma[1]),('blind','grasp',sigma[1]))]
    return {r['name']:r for r in base}


def scene_file(c, cfg, report, arm, case, path, trace):
    root=ET.Element('sdf',version='1.7'); world=ET.SubElement(root,'world',name='mid360s_s2')
    ET.SubElement(world,'gravity').text='0 0 -9.81'
    phy=ET.SubElement(world,'physics',name='ode',type='ode')
    ET.SubElement(phy,'max_step_size').text='.001'; ET.SubElement(phy,'real_time_update_rate').text='1000'
    ET.SubElement(ET.SubElement(world,'scene'),'ambient').text='.5 .5 .5 1'
    ET.SubElement(world,'include').append(ET.Element('uri')); world.find('include/uri').text='model://sun'
    # No external target, flight plugin, mesh network download, or dynamic body.
    ground=transform([0,0,0,0,0,0]); _, link=model(world,'s2_ground',ground)
    box(link,'ground',np.eye(4),[30,30,.1])
    target=c['target']; O=pose_transform([*target['position_world_m'],*target['quaternion_world_wxyz']])
    _, link=model(world,'s2_target',O); parts=[]; score=[]
    for i,p in enumerate(target['parts']):
        T=transform(p['xyz_rpy_deg']); box(link,p['name'],T,p['size_m'],(.85,.85,.85,1))
        part=dict(name=p['name'],label={'body':1,'key':2,'handle':3}[p['name']],pose=(O@T).tolist(),size_m=p['size_m'])
        parts.append(part); score.append(part)
    support=O@transform(target['object_from_support_xyz_rpy_deg']); _,link=model(world,'s2_support',support)
    box(link,'support',np.eye(4),target['support_size_m'],(.3,.3,.3,1))
    score.append(dict(name='support',pose=support.tolist(),size_m=target['support_size_m']))
    view=case['view']; B=np.eye(4)
    if view=='grasp': B[:3,3]=report['arm_geometry']['hover_body_world_xyz_m']
    else:
        B=transform([0,0,0,0,0,c['observations']['reference_to_lidar_azimuth_deg']+180])
        B[:3,3]=report['selected'][view]['body_world_xyz_m']
    L=B@sensor_transform(c)
    vehicle,link=model(world,'s2_vehicle',B)
    def body_box(name,T,size):
        box(link,name,T,size,(.15,.25,.35,1)); score.append(dict(name=name,pose=(B@T).tolist(),size_m=list(size)))
    body=np.array(c['vehicle']['body_box_xyz_size_m']); body_box('central_body',transform([*body[:3],0,0,0]),body[3:])
    l=c['lidar']; base=transform(l['body_from_base_xyz_rpy_deg']); plate=l['plate_size_m']
    body_box('mount_plate',base@transform([0,0,-plate[2]/2,0,0,0]),plate)
    h=cfg['lower_housing_collision_height_m']
    body_box('lower_housing',base@transform([0,0,h/2,0,0,0]),[*l['housing_size_m'][:2],h])
    plan,start,*_=arm; A=transform(c['vehicle']['body_from_arm_xyz_rpy_deg'])
    for name,T,half in plan.boxes(start,.3): body_box('folded_'+name,A@inverse(plan.arm)@T,half*2)
    sensor=ET.SubElement(vehicle,'link',name='lidar_frame'); ET.SubElement(sensor,'pose').text=pose_text(sensor_transform(c))
    joint=ET.SubElement(vehicle,'joint',name='lidar_mount',type='fixed')
    ET.SubElement(joint,'parent').text='link'; ET.SubElement(joint,'child').text='lidar_frame'
    obj=np.array(target['position_world_m'])
    for name,xyz,size in [('backdrop',[obj[0]+2,0,1.5],[.1,6,3]),
                          ('side_upper',[obj[0]-.5,.7,1],[.2,.2,2]),
                          ('side_lower',[obj[0]-.5,-.7,1],[.2,.2,2])]:
        T=transform([*xyz,0,0,0]); _,ln=model(world,'s2_'+name,T); box(ln,name,np.eye(4),size)
        score.append(dict(name=name,pose=T.tolist(),size_m=size))
    if case['name']=='occluded':
        body_ref=next(p for p in parts if p['name']=='body')
        center=(L[:3,3]+np.array(body_ref['pose'])[:3,3])/2
        T=transform([*center,0,0,0]); _,ln=model(world,'s2_occluder',T); box(ln,'occluder',np.eye(4),[.4,.4,.4])
        score.append(dict(name='occluder',pose=T.tolist(),size_m=[.4,.4,.4]))
    plugin=ET.SubElement(world,'plugin',name='mid360s_static_probe',filename='libso101_mid360s_static.so')
    params=dict(trace_file=trace,point_rate=cfg['point_rate_hz'],frame_rate=cfg['frame_rate_hz'],imu_rate=cfg['imu_rate_hz'],
        min_range=l['min_range_m'],max_range=l['max_range_m'],low_deg=l['vertical_fov_deg'][0],high_deg=l['vertical_fov_deg'][1],
        noise_sigma=case['sigma'],seed=case['seed'],points_topic=cfg['topics']['points'],
        imu_topic=cfg['topics']['imu'],truth_topic=cfg['topics']['truth'])
    for key,value in params.items(): ET.SubElement(plugin,key).text=str(value)
    ET.ElementTree(root).write(path,encoding='unicode',xml_declaration=True)
    scene=dict(lidar_world=L.tolist(),body_world=B.tolist(),parts=parts,scoring_boxes=score,
        fov_deg=l['vertical_fov_deg'],static_only=True,case=case)
    save(path.with_suffix('.json'),scene)
    return scene


def stop(proc):
    if proc is None or proc.poll() is not None: return
    for sig,seconds in ((signal.SIGINT,4),(signal.SIGTERM,3),(signal.SIGKILL,2)):
        try: os.killpg(proc.pid,sig)
        except ProcessLookupError: return
        try: proc.wait(timeout=seconds); return
        except sp.TimeoutExpired: pass


def audit_real_bag(path, folder):
    """Historical bag proves schema only unless device/target provenance is supplied."""
    import rosbag
    result=dict(path=str(path),sensor_model_verified=False,performance_comparison_valid=False,
        purpose='historical Livox driver2 schema/source-time comparison only')
    with rosbag.Bag(str(path)) as bag:
        result['topics']={k:dict(type=v.msg_type,count=v.message_count,frequency_hz=v.frequency)
            for k,v in bag.get_type_and_topic_info().topics.items() if k in ('/livox/lidar','/livox/imu')}
        frames=[]; imu=[]; first=None
        for topic,m,_ in bag.read_messages(topics=['/livox/lidar','/livox/imu']):
            stamp=m.header.stamp.to_nsec()
            if first is None: first=stamp
            if stamp-first>3_000_000_000: break
            if topic=='/livox/imu': imu.append(stamp); continue
            offsets=np.array([p.offset_time for p in m.points],dtype=np.int64)
            frames.append(dict(points=m.point_num,header_minus_timebase_ns=stamp-int(m.timebase),
                offset_min_ns=int(min(offsets)),offset_max_ns=int(max(offsets)),
                offsets_nondecreasing=bool(np.all(np.diff(offsets)>=0))))
        result['first_3s_frames']=frames
        result['imu_period_s']=stats_diff(imu)
        result['frames_with_nonmonotonic_offsets']=sum(not f['offsets_nondecreasing'] for f in frames)
        result['frames_extending_beyond_100ms']=sum(f['offset_max_ns']>=100_000_000 for f in frames)
        result['contract_note']='Real frame sizes/time offsets can vary or be out of order; strict ordered 100ms S2 checks are not a hardware adapter'
        result['timestamp_units']='CustomMsg timebase + offset_time, nanoseconds; Imu header stamp, ROS seconds/nanoseconds'
    save(folder/'historical_bag_schema.json',result)
    return result


def stats_diff(stamps):
    d=np.diff(np.array(stamps,dtype=np.int64))*1e-9
    return dict(median=float(np.median(d)),p95=float(np.quantile(d,.95))) if len(d) else None


def record_case(cfg, case, folder, env, gui=False):
    import rosbag
    import rospy
    from sensor_msgs.msg import PointCloud2, Imu
    from rosgraph_msgs.msg import Clock
    from std_srvs.srv import Empty
    lock=threading.RLock(); data=dict(points=[],truth=[],imu=[]); active=[True]
    bag=rosbag.Bag(str(folder/'raw.bag'),'w',compression='lz4')
    def receive(key,m):
        with lock:
            if not active[0]: return
            bag.write(cfg['topics'][key],m,rospy.Time.now())
            data[key].append(m)
    def clock(m):
        with lock:
            if active[0]: bag.write('/clock',m,rospy.Time.now())
    subs=[rospy.Subscriber(cfg['topics'][key],typ,lambda m,k=key:receive(k,m),queue_size=1000,tcp_nodelay=True)
        for key,typ in [('points',PointCloud2),('truth',PointCloud2),('imu',Imu)]]
    subs.append(rospy.Subscriber('/clock',Clock,clock,queue_size=2000))
    proc=None; client=None
    try:
        with (folder/'gazebo.log').open('w') as log:
            proc=sp.Popen(['gzserver','--verbose','-u','-s','libgazebo_ros_api_plugin.so',str(folder/'scene.world')],
                          env=env,stdout=log,stderr=sp.STDOUT,start_new_session=True)
            if gui: client=sp.Popen(['gzclient'],env=env,stdout=log,stderr=sp.STDOUT,start_new_session=True)
            rospy.wait_for_service('/gazebo/unpause_physics',timeout=45)
            deadline=time.monotonic()+15
            while not all(s.get_num_connections()>0 for s in subs[:3]):
                if proc.poll() is not None or time.monotonic()>deadline: raise RuntimeError('Scan publishers did not connect')
                time.sleep(.05)
            rospy.ServiceProxy('/gazebo/unpause_physics',Empty)()
            deadline=time.monotonic()+120
            span=round((cfg['warmup_s']+cfg['duration_s'])*1e9)
            while True:
                with lock:
                    if data['points'] and data['truth'] and data['imu']:
                        first=data['points'][0].header.stamp.to_nsec(); last=data['points'][-1].header.stamp.to_nsec()
                        if last-first>=span and data['truth'][-1].header.stamp.to_nsec()>=last and data['imu'][-1].header.stamp.to_nsec()>=last:
                            break
                if proc.poll() is not None or time.monotonic()>deadline: raise RuntimeError('Static source-time recording timeout')
                time.sleep(.01)  # Wall polling only; completion is determined by source simulation stamps.
            rospy.ServiceProxy('/gazebo/pause_physics',Empty)()
    finally:
        stop(client); stop(proc)
        for s in subs: s.unregister()
        with lock: active[0]=False; bag.close()
    return data


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config',type=Path,default=BRIDGE/'config/perception/mid360s_s1.yaml')
    ap.add_argument('--scan-config',type=Path,default=BRIDGE/'config/perception/mid360s_static.yaml')
    ap.add_argument('--px4-root',type=Path,default=Path.home()/'PX4-RoboMaster')
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--cases',nargs='+',default=None)
    ap.add_argument('--real-bag',type=Path)
    ap.add_argument('--gui',action='store_true')
    args=ap.parse_args(); c=load_profile(args.config); cfg=yaml.safe_load(args.scan_config.read_text())
    if (cfg['schema_version']!=1 or cfg['point_timestamp_unit']!='absolute_nanoseconds'
            or cfg['point_rate_hz']%cfg['frame_rate_hz'] or cfg['duration_s']<1 or cfg['warmup_s']<.1
            or 1000000000%cfg['point_rate_hz'] or 1000000000%cfg['imu_rate_hz']):
        ap.error('Invalid static scan configuration')
    trace=BRIDGE/cfg['trace_file']
    if hashlib.sha256(gzip.decompress(trace.read_bytes())).hexdigest()!=cfg['trace_uncompressed_sha256']:
        ap.error('Vendor trace hash mismatch')
    matrix=cases(cfg); selected=args.cases or list(matrix)
    if any(k not in matrix for k in selected): ap.error('Unknown case')
    if args.output.exists(): ap.error('Choose a new output directory; prior runs are preserved')
    report=check(c,args.px4_root)
    if not report['s1_geometric_precheck_passed']: ap.error('S1 geometry rejected')
    arm=load_arm(args.px4_root); args.output.mkdir(parents=True)
    save(args.output/'s1_report.json',report)
    for src,name in ((args.config,'geometry_used.yaml'),(args.scan_config,'scan_used.yaml')):
        (args.output/name).write_bytes(src.read_bytes())
    manifest=dict(scan_model='MID360 vendor trace, clipped to configured MID360S FOV; NOT calibrated MID360S',
        trace_commit=cfg['trace_commit'],trace_sha256=cfg['trace_uncompressed_sha256'],trace_samples=800000,
        driver2_contract_commit=cfg['driver2_contract_commit'],
        trace_column_1='ordinal sample index despite CSV Time/s header; ignored as physical time',
        trace_repeat_period_s=800000/cfg['point_rate_hz'],timing='simulation time for frames, point timestamp/offset and IMU',
        raw_cloud='driver2 PointXYZRTLT fields, absolute timestamp FLOAT64 ns + extra uint32 offset_time ns',
        truth_topic=cfg['topics']['truth'],truth_topic_for_scoring_only=True,
        point_rate_before_out_of_fov_clipping_hz=cfg['point_rate_hz'],reflectivity_model=False,
        real_device_scan_calibrated=False,moving_sensor_supported=False,flight_or_grasp_started=False,
        hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [args.config,args.scan_config,Path(__file__),
            Path(__file__).with_name('scan_metrics.py'),BRIDGE/'src/perception/mid360s_static_plugin.cpp',
            BRIDGE.parents[1]/'devel/lib/libso101_mid360s_static.so']})
    save(args.output/'manifest.json',manifest)
    env=os.environ.copy(); env.update(ROS_MASTER_URI='http://127.0.0.1:'+str(cfg['ros_port']),ROS_HOSTNAME='localhost',
        GAZEBO_MASTER_URI='http://127.0.0.1:'+str(cfg['gazebo_port']),GAZEBO_MODEL_DATABASE_URI='',
        GAZEBO_PLUGIN_PATH=str(BRIDGE.parents[1]/'devel/lib')+':/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins')
    env.pop('ROS_IP',None)
    for port in (cfg['ros_port'],cfg['gazebo_port']):
        with socket.socket() as s:
            if s.connect_ex(('127.0.0.1',port))==0: raise RuntimeError('Experiment port already occupied: '+str(port))
    os.environ.update(ROS_MASTER_URI=env['ROS_MASTER_URI'],ROS_HOSTNAME='localhost'); os.environ.pop('ROS_IP',None)
    import rosgraph
    import rospy
    master=None; results={}
    try:
        with (args.output/'roscore.log').open('w') as log:
            master=sp.Popen(['roscore','-p',str(cfg['ros_port'])],env=env,stdout=log,stderr=sp.STDOUT,start_new_session=True)
            deadline=time.monotonic()+30
            while not rosgraph.is_master_online():
                if master.poll() is not None or time.monotonic()>deadline: raise RuntimeError('Isolated ROS master failed')
                time.sleep(.1)
            rospy.set_param('/use_sim_time',True); rospy.init_node('mid360s_s2_recorder',anonymous=True,disable_signals=True)
            for name in selected:
                folder=args.output/name; folder.mkdir(); case=matrix[name]
                scene=scene_file(c,cfg,report,arm,case,folder/'scene.world',trace)
                data=record_case(cfg,case,folder,env,args.gui)
                r=analyse(data['points'],data['truth'],data['imu'],scene,cfg,case)
                save(folder/'result.json',r); results[name]=r
                print(json.dumps(dict(case=name,passed=r['static_case_passed'],target_returns=r['target_return_count'],
                    imu_bracket_s=r['max_imu_bracket_s'])),flush=True)
            repeat=None
            if 'primary' in results and 'primary_repeat' in results:
                import rosbag
                hashes=[]
                for name in ('primary','primary_repeat'):
                    h=hashlib.sha256()
                    with rosbag.Bag(str(args.output/name/'raw.bag')) as bag:
                        for _,m,_ in bag.read_messages(topics=[cfg['topics']['points']]):
                            if cfg['warmup_s'] <= m.header.stamp.to_sec() < cfg['warmup_s']+cfg['duration_s']:
                                h.update(m.data)
                    hashes.append(h.hexdigest())
                repeat=dict(primary_hash=hashes[0],repeat_hash=hashes[1],same_point_bytes=hashes[0]==hashes[1])
            real=audit_real_bag(args.real_bag,args.output) if args.real_bag else None
            summary=dict(static_simulation_matrix_passed=bool(all(r['static_case_passed'] for r in results.values()) and
                (repeat is None or repeat['same_point_bytes'])),cases={k:dict(passed=r['static_case_passed'],
                    target_returns=r['target_return_count'],frames=r['frames']) for k,r in results.items()},repeatability=repeat,
                historical_interface_comparison_performed=real is not None,mid360s_real_bag_calibration_passed=False,
                no_flight_or_grasp_commands=True,hardware_transfer_ready=False)
            save(args.output/'result.json',summary)
            with (args.output/'coverage.csv').open('w',newline='') as f:
                w=csv.writer(f); w.writerow(['case','window_s','part','returns_min','returns_mean','returns_max','coverage_mean'])
                for k,r in results.items():
                    for row in r['integration']:
                        for part,v in row['summary'].items():
                            s=v['returns']; cov=v['clean_visible_surface_coverage']
                            w.writerow([k,row['window_s'],part,s['min'],s['mean'],s['max'],cov['mean'] if cov else ''])
            print(json.dumps(summary,indent=2),flush=True)
            return 0 if summary['static_simulation_matrix_passed'] else 2
    finally:
        if rospy.core.is_initialized(): rospy.signal_shutdown('S2 finished')
        stop(master)


if __name__=='__main__':
    raise SystemExit(main())
