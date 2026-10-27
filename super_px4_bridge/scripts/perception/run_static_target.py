#!/usr/bin/env python3
"""S3 bag replay only. Estimate from raw XYZ; read scene/labels AFTER estimation for scoring."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import yaml
import rosbag
from scan_metrics import cloud_array, xyz, point_times
from check_installation import BRIDGE, inverse, pose_transform
from static_target import estimate, shape_from_geometry, rotation_angle, candidate_points


def save(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def read_raw(path,start_s,duration_s):
    """The only bag topics accessed by estimation: measured points and IMU."""
    arrays=[]; times=[]; imus=[]; clouds=[]
    with rosbag.Bag(str(path)) as bag:
        for topic,m,_ in bag.read_messages(topics=['/mid360s_sim/points','/mid360s_sim/imu']):
            (imus if topic=='/mid360s_sim/imu' else clouds).append(m)
    if not clouds: raise ValueError('Missing raw point stream')
    # Bag receipt-time ordering may differ from sensor source ordering across /clock startup.
    imu_reversals=int(np.sum(np.diff([m.header.stamp.to_nsec() for m in imus])<0))
    imus.sort(key=lambda m:m.header.stamp.to_nsec()); clouds.sort(key=lambda m:m.header.stamp.to_nsec())
    first=clouds[0].header.stamp.to_nsec(); frame_stamps=[m.header.stamp.to_nsec() for m in clouds]
    begin=first+round(start_s*1e9); end=begin+round(duration_s*1e9)
    for m in clouds:
        if m.header.frame_id!='mid360s_lidar': raise ValueError('Wrong raw cloud frame')
        a=cloud_array(m); t=point_times(m,a,100_000_000)
        keep=(t>=begin)&(t<end); arrays.append(xyz(a)[keep]); times.append(t[keep])
    if np.any(np.diff(frame_stamps)!=100_000_000) or frame_stamps[-1]+100_000_000<end:
        raise ValueError('Missing, duplicate or incomplete raw frames')
    if (not imus or imus[0].header.stamp.to_nsec()>begin or
            imus[-1].header.stamp.to_nsec()<end):
        raise ValueError('Window not bracketed by IMU')
    selected_imu=[m for m in imus if begin<=m.header.stamp.to_nsec()<=end]
    if np.any(np.diff([m.header.stamp.to_nsec() for m in imus])!=5_000_000):
        raise ValueError('Missing or out-of-order ideal static IMU')
    if not selected_imu or any(m.header.frame_id!='mid360s_lidar' for m in selected_imu):
        raise ValueError('Wrong IMU source or window')
    gyros=np.array([[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z] for m in selected_imu])
    if not np.isfinite(gyros).all() or np.max(np.linalg.norm(gyros,axis=1))>.001:
        raise ValueError('S3 input is not an admitted static observation')
    accelerations=np.array([[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z] for m in selected_imu])
    up=np.median(accelerations,axis=0)
    if not np.isfinite(up).all() or not 9.5<=np.linalg.norm(up)<=10.1:
        raise ValueError('Invalid static IMU specific force')
    up/=np.linalg.norm(up)
    return np.concatenate(arrays),dict(frame='mid360s_lidar',observation_start_ns=begin,
        observation_end_ns=end,observation_duration_s=duration_s,source_clock='simulation',
        input_topics=['/mid360s_sim/points','/mid360s_sim/imu'],restamped=False,imu_up_lidar=up.tolist(),
        bag_imu_header_order_reversals=imu_reversals,paired_by='sorted source header and point time')


def score_after_estimation(dataset,case,begin,end,report,remaining,groups):
    """Independent offline oracle. Nothing from here is passed into estimate()."""
    scene=json.loads((dataset/case/'scene.json').read_text())
    geometry=yaml.safe_load((dataset/'geometry_used.yaml').read_text())
    O=pose_transform([*geometry['target']['position_world_m'],*geometry['target']['quaternion_world_wxyz']])
    expected=inverse(np.array(scene['lidar_world']))@O
    by_stamp={}; raw_by_stamp={}
    with rosbag.Bag(str(dataset/case/'raw.bag')) as bag:
        for topic,m,_ in bag.read_messages(topics=['/mid360s_s2/ray_truth','/mid360s_sim/points']):
            a=cloud_array(m); t=m.header.stamp.to_nsec()+a['offset_time'].astype(np.int64)
            keep=(t>=begin)&(t<end)
            if topic=='/mid360s_s2/ray_truth': by_stamp[m.header.stamp.to_nsec()]=a['label'][keep]
            else: raw_by_stamp[m.header.stamp.to_nsec()]=xyz(a)[keep]
    labels=np.concatenate(list(by_stamp.values()))
    visible=bool(np.any((labels>=1)&(labels<=3)))
    evaluation=dict(target_has_returns=visible,reference='scene and ray truth used only after output exists',
        accepted=report['valid'],translation_error_m=None,rotation_error_deg=None,
        meets_3mm_5deg=False,segmentation_target_fraction=None)
    best=report['best']
    if best:
        R=np.array(best['R']); t=np.array(best['position_lidar_m'])
        error=float(np.linalg.norm(t-expected[:3,3])); angle=rotation_angle(R.T@expected[:3,:3])
        evaluation.update(translation_error_m=error,rotation_error_deg=angle,
            meets_3mm_5deg=bool(visible and report['valid'] and error<=.003 and angle<=5))
        lookup={tuple(point):int(label) for stamp,batch in raw_by_stamp.items() for point,label in zip(batch,by_stamp[stamp])}
        original=groups[best['cluster']]; chosen=candidate_points(report,groups)
        before=np.array([lookup[tuple(p)] for p in original]); after=np.array([lookup[tuple(p)] for p in chosen])
        count=lambda labels:{str(int(i)):int(np.sum(labels==i)) for i in np.unique(labels)}
        target_before=int(np.sum((before>=1)&(before<=3)))
        target_after=int(np.sum((after>=1)&(after<=3)))
        support_before=int(np.sum(before==4)); support_after=int(np.sum(after==4))
        evaluation.update(segmentation_before_target_fraction=target_before/len(before),
            segmentation_target_fraction=target_after/len(after),
            segmentation_label_counts_before=count(before),segmentation_label_counts_after=count(after),
            retained_target_fraction=target_after/target_before if target_before else None,
            removed_support_fraction=(support_before-support_after)/support_before if support_before else None)
    return evaluation


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dataset',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--config',type=Path,default=BRIDGE/'config/perception/static_target.yaml')
    ap.add_argument('--geometry',type=Path,default=BRIDGE/'config/perception/mid360s_s1.yaml')
    ap.add_argument('--cases',nargs='+',default=['primary','primary_noise','primary_noise2','backup','backup_noise','occluded','blind'])
    ap.add_argument('--window',type=float,default=1.)
    ap.add_argument('--starts',nargs='+',type=float,default=[.2,1.2])
    args=ap.parse_args()
    if args.output.exists(): ap.error('Choose a new output directory')
    if args.window<=0 or not math.isfinite(args.window) or any(s<0 or not math.isfinite(s) for s in args.starts):
        ap.error('Invalid observation window')
    cfg=yaml.safe_load(args.config.read_text()); geometry=yaml.safe_load(args.geometry.read_text())
    parts=shape_from_geometry(geometry)
    args.output.mkdir(parents=True)
    manifest=dict(estimator_inputs='raw XYZ + local box dimensions + measured static IMU up',
        no_world_target_pose_roi_or_initialization=True,oracle_only_after_estimate=True,
        no_publishers_or_control_commands=True,static_simulation_only=True,
        parent_dataset=str(args.dataset.resolve()),hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [args.config,args.geometry,Path(__file__),Path(__file__).with_name('static_target.py'),
                Path(__file__).with_name('check_installation.py'),Path(__file__).with_name('scan_metrics.py')]})
    save(args.output/'manifest.json',manifest)
    (args.output/'config_used.yaml').write_bytes(args.config.read_bytes())
    local_model={'target':{'parts':[{k:p[k] for k in ('name','size_m','xyz_rpy_deg')} for p in geometry['target']['parts']]}}
    (args.output/'model_used.yaml').write_text(yaml.safe_dump(local_model,sort_keys=False))
    records=[]
    for case in args.cases:
        for start in args.starts:
            name=case+'_start'+str(start).replace('.','_'); folder=args.output/name; folder.mkdir()
            points,source=read_raw(args.dataset/case/'raw.bag',start,args.window)
            report,remaining,groups=estimate(points,parts,cfg,source['imu_up_lidar']); report['source_observation']=source
            # Persist inference BEFORE reading any scene or semantic scoring topic.
            save(folder/'estimate.json',report)
            chosen=candidate_points(report,groups)
            original=groups[report['best']['cluster']] if report['best'] else np.empty((0,3))
            np.savez_compressed(folder/'segmentation.npz',remaining=remaining,candidate=chosen,raw_candidate=original)
            evaluation=score_after_estimation(args.dataset,case,source['observation_start_ns'],source['observation_end_ns'],report,remaining,groups)
            save(folder/'evaluation.json',evaluation)
            row=dict(case=case,start_s=start,window_s=args.window,valid=report['valid'],reasons=report['reasons'],
                best=report['best'],elapsed_s=report['elapsed_wall_s'],evaluation=evaluation)
            records.append(row)
            print(json.dumps(dict(case=case,start=start,valid=report['valid'],reasons=report['reasons'],
                error_m=evaluation['translation_error_m'],angle_deg=evaluation['rotation_error_deg'])),flush=True)
    # Byte hashes are recorded only after inference. No scoring geometry is an estimator input.
    manifest['input_bag_sha256']={case:hashlib.sha256((args.dataset/case/'raw.bag').read_bytes()).hexdigest() for case in args.cases}
    scoring=[args.dataset/'geometry_used.yaml',args.dataset/'manifest.json']+[args.dataset/case/'scene.json' for case in args.cases]
    manifest['scoring_only_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in scoring}
    save(args.output/'manifest.json',manifest)
    accepted=[r for r in records if r['valid'] and r['evaluation']['target_has_returns']]
    summary=dict(records=records,accepted_target_windows=len(accepted),
        accepted_without_target=sum(r['valid'] and not r['evaluation']['target_has_returns'] for r in records),
        accepted_translation_p95_m=float(np.quantile([r['evaluation']['translation_error_m'] for r in accepted],.95)) if accepted else None,
        accepted_rotation_p95_deg=float(np.quantile([r['evaluation']['rotation_error_deg'] for r in accepted],.95)) if accepted else None,
        demo_alignment_windows=sum(r['evaluation']['meets_3mm_5deg'] for r in records),
        flight_or_grasp_ready=False,real_device_validated=False)
    # Repeatability is a diagnostic, not calibrated uncertainty or control permission.
    repeat=[]
    for i,a in enumerate(records):
        for b in records[i+1:]:
            if a['case']!=b['case'] or not a['best'] or not b['best']: continue
            if min(a['start_s']+a['window_s'],b['start_s']+b['window_s'])>max(a['start_s'],b['start_s'])+1e-9: continue
            repeat.append(dict(case=a['case'],starts_s=[a['start_s'],b['start_s']],
                both_quality_accepted=a['valid'] and b['valid'],
                translation_delta_m=float(np.linalg.norm(np.array(a['best']['position_lidar_m'])-b['best']['position_lidar_m'])),
                rotation_delta_deg=rotation_angle(np.array(a['best']['R']).T@np.array(b['best']['R']))))
    summary['disjoint_window_repeatability']=repeat
    save(args.output/'result.json',summary)
    return 0


if __name__=='__main__': raise SystemExit(main())
