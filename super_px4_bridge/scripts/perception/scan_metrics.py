"""S2 source-time and static-surface scoring; truth is never a perception input."""
import itertools
import math
import numpy as np
from check_installation import blocked, inverse, transform

TYPE = {2:'u1', 4:'<u2', 6:'<u4', 7:'<f4', 8:'<f8'}


def cloud_array(m):
    if m.is_bigendian or m.height != 1 or m.row_step != m.width*m.point_step or len(m.data) != m.row_step:
        raise ValueError('Invalid point cloud layout')
    if len({f.name for f in m.fields}) != len(m.fields):
        raise ValueError('Duplicate cloud field')
    dtype = np.dtype(dict(names=[f.name for f in m.fields],
        formats=[TYPE[f.datatype] for f in m.fields], offsets=[f.offset for f in m.fields], itemsize=m.point_step))
    if any(f.count != 1 for f in m.fields):
        raise ValueError('Only scalar fields supported')
    return np.frombuffer(m.data, dtype=dtype, count=m.width)


def point_times(m, a, frame_ns):
    if not {'x','y','z','timestamp','offset_time'} <= set(a.dtype.names):
        raise ValueError('Missing coordinates or point time')
    offset = a['offset_time'].astype(np.int64)
    times = m.header.stamp.to_nsec()+offset
    if (np.any(offset >= frame_ns) or np.any(np.diff(offset) <= 0) or
            not np.isfinite(a['timestamp']).all() or
            np.any(abs(a['timestamp']-times) > 1.0)):
        raise ValueError('Bad point time, units or order')
    return times


def xyz(a):
    p = np.column_stack([a[k] for k in ('x','y','z')])
    if not np.isfinite(p).all():
        raise ValueError('Nonfinite cloud coordinates')
    return p


def cells(local, size, cell):
    """Nearest box face and two face-grid coordinates, for CLEAN scoring points."""
    half = size/2
    axes = np.argmin(abs(abs(local)-half), axis=1)
    result = []
    for p, axis in zip(local, axes):
        other = [i for i in range(3) if i != axis]
        n = np.ceil(size[other]/cell).astype(int)
        ij = np.clip(np.floor((p[other]+half[other])/size[other]*n).astype(int), 0, n-1)
        result.append((int(axis), 1 if p[axis] >= 0 else -1, int(ij[0]), int(ij[1])))
    return result


def visible_cells(part, scene, cell):
    T, size = np.array(part['pose']), np.array(part['size_m']); half = size/2
    sensor = np.array(scene['lidar_world'])[:3, 3]
    L = inverse(np.array(scene['lidar_world'])); ids, points, normals = [], [], []
    for axis, sign in itertools.product(range(3), (-1, 1)):
        other = [i for i in range(3) if i != axis]
        n = np.ceil(size[other]/cell).astype(int)
        for i, j in itertools.product(range(n[0]), range(n[1])):
            p = np.zeros(3); p[axis] = sign*half[axis]
            p[other] = -half[other]+(np.array([i,j])+.5)*size[other]/n
            ids.append((axis, sign, i, j)); points.append(T[:3,:3]@p+T[:3,3]); normals.append(T[:3,axis]*sign)
    points, normals = np.array(points), np.array(normals)
    local = points@L[:3,:3].T+L[:3,3]
    beta = np.rad2deg(np.arctan2(local[:,2], np.linalg.norm(local[:,:2],axis=1)))
    lo, hi = scene['fov_deg']
    visible = (np.einsum('ij,ij->i',sensor-points,normals)>0) & (beta>=lo) & (beta<=hi)
    for box in scene['scoring_boxes']:
        if box['name'] == part['name']:
            continue
        visible &= ~blocked(sensor, points, np.array(box['pose']), np.array(box['size_m'])/2)
    return {key for key, valid in zip(ids, visible) if valid}


def stats(values):
    v = np.asarray(values, dtype=float)
    return dict(min=float(v.min()), mean=float(v.mean()), max=float(v.max())) if len(v) else None


def analyse(clouds, truths, imus, scene, cfg, case):
    if not clouds or not imus:
        raise ValueError('Missing cloud or IMU data')
    frame_ns = int(1e9/cfg['frame_rate_hz'])
    epoch = clouds[0].header.stamp.to_nsec()
    start = epoch+round(cfg['warmup_s']*1e9); end = start+round(cfg['duration_s']*1e9)
    selected = [m for m in clouds if start <= m.header.stamp.to_nsec() < end]
    expected_frames = round(cfg['duration_s']*cfg['frame_rate_hz'])
    stamps = np.array([m.header.stamp.to_nsec() for m in selected], dtype=np.int64)
    if len(selected) != expected_frames or np.any(np.diff(stamps) != frame_ns):
        raise ValueError('Missing/duplicate/nonperiodic cloud frame')
    truth = {m.header.stamp.to_nsec():m for m in truths}
    if len(truth) != len(truths):
        raise ValueError('Duplicate truth frame')
    imu_times = np.array([m.header.stamp.to_nsec() for m in imus], dtype=np.int64)
    if np.any(np.diff(imu_times) != round(1e9/cfg['imu_rate_hz'])):
        raise ValueError('IMU timestamps not consecutive')
    force = np.array(scene['lidar_world'])[:3,:3].T@np.array([0.,0.,9.81])
    if any(m.header.frame_id != 'mid360s_lidar' or m.orientation_covariance[0] != -1 or
            np.linalg.norm([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])>1e-10 or
            np.linalg.norm(np.array([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])-force)>1e-8
            for m in imus):
        raise ValueError('Static IMU units/frame/specific-force mismatch')
    all_points, all_times, labels, errors, frame_counts = [], [], [], [], []
    max_bracket = 0.; max_surface_error = 0.; full_scene = 0
    L = np.array(scene['lidar_world'])
    target_parts = {p['label']:p for p in scene['parts']}
    for m in selected:
        if m.header.frame_id != 'mid360s_lidar' or m.header.stamp.to_nsec() not in truth:
            raise ValueError('Frame id or truth pairing mismatch')
        a = cloud_array(m); gt = cloud_array(truth[m.header.stamp.to_nsec()])
        if len(a) != len(gt) or not np.array_equal(a['offset_time'],gt['offset_time']):
            raise ValueError('Raw/truth point correspondence mismatch')
        times = point_times(m,a,frame_ns); raw, clean = xyz(a), xyz(gt)
        where = np.searchsorted(imu_times,times,side='left')
        if np.any(where==0) or np.any(where==len(imu_times)):
            raise ValueError('Point not bracketed by IMU')
        if len(times):
            max_bracket=max(max_bracket,float(np.max(np.maximum(times-imu_times[where-1],imu_times[where]-times)))*1e-9)
        world = clean@L[:3,:3].T+L[:3,3]
        for label, part in target_parts.items():
            p = world[gt['label']==label]
            if len(p):
                T=np.array(part['pose']); local=(p-T[:3,3])@T[:3,:3]; h=np.array(part['size_m'])/2
                residual=np.min(abs(abs(local)-h),axis=1)
                max_surface_error=max(max_surface_error,float(max(residual)))
                if np.any(abs(local)>h+2e-5): raise ValueError('Truth hit outside target box')
        all_points.append(world); all_times.append(times); labels.append(gt['label'])
        errors.extend((np.linalg.norm(raw,axis=1)-np.linalg.norm(clean,axis=1)).tolist())
        frame_counts.append({str(k):int(np.sum(gt['label']==k)) for k in target_parts})
        full_scene += int(np.sum(gt['label']>=4))
    if max_surface_error>2e-5 or max_bracket>1/cfg['imu_rate_hz']+1e-9:
        raise ValueError('Frame geometry or source-time bracket failed')
    points=np.concatenate(all_points); times=np.concatenate(all_times); label=np.concatenate(labels)
    integration=[]
    for seconds in cfg['integration_windows_s']:
        width=round(seconds*1e9); rows=[]
        for begin in range(start,end-width+1,width):
            mask=(times>=begin)&(times<begin+width); row={}
            for key, part in target_parts.items():
                p=points[mask&(label==key)]; T=np.array(part['pose']); size=np.array(part['size_m'])
                local=(p-T[:3,3])@T[:3,:3]
                observed=set(cells(local,size,cfg['coverage_cell_m'])) if len(p) else set()
                visible=visible_cells(part,scene,cfg['coverage_cell_m'])
                row[part['name']]=dict(returns=len(p),occupied_cells=len(observed),visible_cells=len(visible),
                    clean_visible_surface_coverage=len(observed&visible)/len(visible) if visible else None)
            rows.append(row)
        integration.append(dict(window_s=seconds,windows=rows,summary={part['name']:dict(
            returns=stats([r[part['name']]['returns'] for r in rows]),
            clean_visible_surface_coverage=stats([r[part['name']]['clean_visible_surface_coverage'] for r in rows
                                                if r[part['name']]['clean_visible_surface_coverage'] is not None]))
            for part in target_parts.values()}))
    total_target=int(np.sum((label>=1)&(label<=3)))
    expected_zero=case['name'] in ('occluded','blind')
    passed=bool(total_target==0 if expected_zero else
        all(np.any(label==k) for k in (1,2)) and full_scene>0)
    return dict(case=case,frames=len(selected),sim_duration_s=cfg['duration_s'],timing_valid=True,
        point_frame='mid360s_lidar',header_convention='scheduled frame start in simulation time',
        imu_frame='mid360s_lidar (coincident ideal static IMU)',max_imu_bracket_s=max_bracket,
        clean_surface_residual_max_m=max_surface_error,raw_return_count=len(points),
        target_return_count=total_target,background_support_vehicle_returns=full_scene,per_frame_target_returns=frame_counts,
        absolute_range_noise_p50_m=float(np.quantile(abs(np.asarray(errors)),.5)) if errors else None,
        absolute_range_noise_p95_m=float(np.quantile(abs(np.asarray(errors)),.95)) if errors else None,
        integration=integration,expected_no_target_returns=expected_zero,static_case_passed=passed,
        coverage_is_truth_scoring_not_detector_accuracy=True)
