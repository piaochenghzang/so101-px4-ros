#!/usr/bin/env python3
"""Isolated passive Gazebo coordinate test. No PX4/MAVROS/servo/motor actuation.

Expanded production SDF is copied into a NEW zero-gravity test world, with all
model plugins, sensors and collisions removed. Only our read-only truth plugin is added. Never
uses spawn, set_model_state, set_joint_properties, reset or command publishers.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess as sp
import time
import xml.etree.ElementTree as ET

import rosgraph
import rospkg
import yaml


def port_busy(port):
    with socket.socket() as sock:
        sock.settimeout(.2)
        return sock.connect_ex(('127.0.0.1',port)) == 0


def stop_group(process):
    if process is None: return
    for sig, wait in ((signal.SIGINT,5),(signal.SIGTERM,3),(signal.SIGKILL,2)):
        try: os.killpg(process.pid,sig)
        except ProcessLookupError: return
        try: process.wait(timeout=wait); return
        except sp.TimeoutExpired: pass


def write_json(path, value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def build_world(models, folder, case, environment):
    vehicle_file = models/'yhang550/yhang550.sdf'
    expanded = sp.check_output(['gz','sdf','-p',str(vehicle_file)],env=environment,text=True,timeout=30)
    (folder/'production_expanded.sdf').write_text(expanded)
    vehicle = ET.fromstring(expanded).find('model')
    if vehicle is None: raise RuntimeError('Cannot expand production vehicle')
    removed = []
    for parent in vehicle.iter():
        for child in list(parent):
            if child.tag in ('plugin','sensor','collision'):
                removed.append(dict(type=child.tag,name=child.get('name'))); parent.remove(child)
    static = vehicle.find('static')
    if static is None: static = ET.SubElement(vehicle,'static')
    static.text = 'false'  # Real ODE joint measurements, not staticPosition cache.
    pose = vehicle.find('pose')
    if pose is None: pose = ET.SubElement(vehicle,'pose')
    poses = dict(baseline='0 0 1.3 0 0 0',yawed='0.8 -0.4 1.6 0 0 1.2',
                 tilted='-0.3 0.5 1.7 0.08 -0.05 -0.7',missing_target='0 0 1.3 0 0 0',
                 wrong_mount='0 0 1.3 0 0 0')
    pose.text = poses[case]
    root = ET.Element('sdf',version='1.7')
    world = ET.SubElement(root,'world',name='default')
    ET.SubElement(world,'gravity').text='0 0 0'
    physics = ET.SubElement(world,'physics',name='audit',type='ode')
    ET.SubElement(physics,'max_step_size').text = '.001'
    ET.SubElement(physics,'real_time_update_rate').text = '1000'
    world.append(vehicle)
    if case != 'missing_target':
        cube = ET.parse(models/'test_cube/model.sdf').find('model')
        cube.set('name','test_cube')
        tag = cube.find('static')
        if tag is None: tag = ET.SubElement(cube,'static')
        tag.text='true'
        tag = cube.find('pose')
        if tag is None: tag = ET.SubElement(cube,'pose')
        tag.text='0.3 0.1 1.1 0 0 .3'
        for parent in cube.iter():
            for child in list(parent):
                if child.tag in ('plugin','sensor','collision'): parent.remove(child)
        world.append(cube)
    plugin = ET.SubElement(world,'plugin',name='coordinate_truth',filename='libso101_coordinate_truth.so')
    ET.SubElement(plugin,'vehicleName').text='yhang550'
    ET.SubElement(plugin,'targetName').text='test_cube'
    if vehicle.findall('.//plugin') or vehicle.findall('.//sensor') or vehicle.findall('.//collision'):
        raise RuntimeError('Passive model still has actuation, sensor or contact geometry')
    ET.ElementTree(root).write(folder/'scene.world',encoding='unicode',xml_declaration=True)
    return removed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name',required=True)
    parser.add_argument('--case',choices=['baseline','yawed','tilted','missing_target','wrong_mount'],default='baseline')
    parser.add_argument('--duration',type=float,default=4)
    args = parser.parse_args()
    if not args.name or Path(args.name).name != args.name or args.name in ('.','..'):
        parser.error('name must be a single safe directory name')
    if not 1 <= args.duration <= 30: parser.error('duration must be within [1,30] sim seconds')
    root = Path(os.environ.get('PX4_DIR',str(Path.home()/'PX4-RoboMaster'))).resolve()
    workspace = Path(os.environ.get('ROS_WS',str(Path.home()/'super_ws'))).resolve()
    package = Path(rospkg.RosPack().get_path('super_px4_bridge'))
    models = root/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'
    for process in ('gzserver','gzclient','px4','rosmaster'):
        if sp.run(['pgrep','-x',process],stdout=sp.DEVNULL,stderr=sp.DEVNULL).returncode == 0:
            raise RuntimeError('Existing '+process+' detected; refusing to disturb it')
    if port_busy(11357) or port_busy(11457): raise RuntimeError('Audit ports are occupied')
    folder = root/'build/frame_audit'/args.name
    folder.mkdir(parents=True,exist_ok=False)
    library = workspace/'devel/lib/libso101_coordinate_truth.so'
    if not library.is_file(): raise RuntimeError('Build so101_coordinate_truth first')
    environment = os.environ.copy()
    environment.update(ROS_MASTER_URI='http://127.0.0.1:11357',ROS_HOSTNAME='localhost',
                       GAZEBO_MASTER_URI='http://127.0.0.1:11457',GAZEBO_MODEL_PATH=str(models),
                       GAZEBO_PLUGIN_PATH=str(library.parent)+':/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',
                       GAZEBO_MODEL_DATABASE_URI='')
    environment.pop('ROS_IP',None)
    removed = build_world(models,folder,args.case,environment)
    config = yaml.safe_load((package/'config/grasp_frame_audit.yaml').read_text())
    if args.case == 'wrong_mount': config['mount_pose'][3]=0  # Negative checker case only.
    config_path = folder/'audit_config.yaml'
    config_path.write_text(yaml.safe_dump(config,sort_keys=False))
    write_json(folder/'manifest.json',dict(case=args.case,vehicle_static=False,cube_static=True,gravity=[0,0,0],
               motion_commands_sent=False,px4_started=False,mavros_started=False,removed=removed,
               source_package=str(package),truth_library_sha256=hashlib.sha256(library.read_bytes()).hexdigest()))
    processes,logs = [],[]
    def start(command,name):
        log = (folder/(name+'.log')).open('w'); logs.append(log)
        process = sp.Popen(command,env=environment,stdout=log,stderr=sp.STDOUT,start_new_session=True)
        processes.append(process); return process
    result = dict(success=False,close_permitted=False)
    try:
        core = start(['roscore','-p','11357'],'roscore')
        deadline = time.monotonic()+20
        master = rosgraph.Master('/readonly_frame_audit_runner',environment['ROS_MASTER_URI'])
        while time.monotonic() < deadline:
            if core.poll() is not None: raise RuntimeError('Own roscore exited')
            try: master.getPid(); break
            except (socket.error,rosgraph.MasterError): time.sleep(.1)
        else: raise RuntimeError('Own roscore did not become ready')
        # Isolated master only. Never changes the user's running ROS parameters.
        sp.run(['rosparam','set','/use_sim_time','true'],env=environment,check=True,timeout=10)
        observer = start(['python3',str(package/'scripts/observe_grasp_frames.py'),
                          '--config',str(config_path),'--output',str(folder/'observations.jsonl'),
                          '--duration',str(args.duration),'--wall-timeout','45'],'observer')
        server = start(['gzserver','--verbose','-s','libgazebo_ros_api_plugin.so',str(folder/'scene.world')],'gazebo')
        graph = None
        deadline = time.monotonic()+50
        while observer.poll() is None and time.monotonic() < deadline:
            if server.poll() is not None: raise RuntimeError('Own passive Gazebo exited')
            graph = master.getSystemState()
            if any(topic.startswith('/so101/') and topic.endswith('/cmd_pos') or
                   topic in ('/mavros/setpoint_raw/local','/planning/click_goal') for topic,_ in graph[0]):
                raise RuntimeError('Unexpected command publisher in isolated observation master')
            time.sleep(.1)
        if observer.poll() is None: raise RuntimeError('Observer wall timeout')
        write_json(folder/'ros_graph.json',dict(publishers=graph[0],subscribers=graph[1],services=graph[2]))
        events = [json.loads(line) for line in (folder/'observations.jsonl').read_text().splitlines()]
        samples = [e for e in events if e.get('kind') == 'sample' and e.get('truth_valid')]
        if not samples: raise RuntimeError('No finite physical-step truth samples')
        expected_chain = args.case != 'wrong_mount'
        first = samples[0]
        body_drift = max(sum((s['body_pose_world'][k]-first['body_pose_world'][k])**2 for k in range(3))**.5 for s in samples)
        joint_drift = max(max(abs(a-b) for a,b in zip(s['joint_position_measured'],first['joint_position_measured'])) for s in samples)
        checks = dict(chain_expected=all(s['chain_valid'] == expected_chain for s in samples),
                      target_expected=all(s['target_present'] == (args.case != 'missing_target') for s in samples),
                      no_close_permission=all(e.get('close_permitted') is False for e in events),
                      no_fake_local_frame=all(s['local_valid'] is False for s in samples),
                      observation_completed=events[-1].get('outcome') == 'OBSERVATION_COMPLETE',
                      passive_motion_negligible=body_drift <= 1e-5 and joint_drift <= 1e-5)
        result.update(success=all(checks.values()),checks=checks,samples=len(samples),
                      observer_exit_code=observer.returncode,
                      max_fk_position_error_m=max(s['fk_position_error_m'] for s in samples),
                      max_fk_rotation_error_rad=max(s['fk_rotation_error_rad'] for s in samples),
                      max_mount_position_error_m=max(s['mount_position_error_m'] for s in samples),
                      max_lidar_position_error_m=max(s['lidar_position_error_m'] for s in samples),
                      max_body_drift_m=body_drift,max_joint_drift_rad=joint_drift,
                      local_frame_tested=False,grasp_tested=False)
    except Exception as error:
        result['error']=str(error)
    finally:
        for process in reversed(processes): stop_group(process)
        for log in logs: log.close()
        write_json(folder/'result.json',result)
    print(json.dumps(dict(folder=str(folder),**result),indent=2,allow_nan=False))
    return 0 if result['success'] else 1


if __name__ == '__main__': raise SystemExit(main())
