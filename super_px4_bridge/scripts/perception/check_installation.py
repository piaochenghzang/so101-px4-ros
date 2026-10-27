#!/usr/bin/env python3
"""S1 offline geometry only. No ROS, MAVLink, sockets or control publishers."""
import argparse
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys
import numpy as np
import yaml

SCRIPT = Path(__file__).resolve()
BRIDGE = SCRIPT.parents[2]
sys.path.insert(0, str(BRIDGE / 'scripts'))
from grasp_audit.geometry import finite, inverse, pose_transform, rpy_transform


def transform(values):
    v = finite(values, (6,)).copy()
    v[3:] = np.deg2rad(v[3:])
    return rpy_transform(v)


def positive(value, shape=None):
    v = finite(value, shape)
    if np.any(v <= 0):
        raise ValueError('Sizes, ranges and masses must be positive')
    return v


def load_profile(path):
    c = yaml.safe_load(Path(path).read_text())
    if c['schema_version'] != 1:
        raise ValueError('Unsupported schema_version')
    for group in ('lidar', 'vehicle', 'target', 'observations'):
        if not isinstance(c[group], dict):
            raise ValueError('Missing profile group: ' + group)
    l, t, o = c['lidar'], c['target'], c['observations']
    for k in ('housing_size_m', 'plate_size_m'):
        positive(l[k], (3,))
    for k in ('mass_kg', 'plate_density_kg_m3', 'min_range_m', 'max_range_m',
              'peripheral_air_gap_m', 'minimum_exposed_top_area_m2'):
        positive(l[k], ())
    fov = finite(l['vertical_fov_deg'], (2,))
    margin = float(positive(l['fov_margin_deg'], ()))
    if not (-90 < fov[0] < fov[1] < 90) or 2*margin >= fov[1]-fov[0]:
        raise ValueError('Invalid vertical FOV/margin')
    if l['horizontal_fov_deg'] != 360 or l['min_range_m'] >= l['max_range_m']:
        raise ValueError('This S1 checker requires 360 degree horizontal FOV and ordered ranges')
    for k in ('body_from_base_xyz_rpy_deg', 'base_from_lidar_xyz_rpy_deg'):
        transform(l[k])
    if l['body_from_lidar_override_xyz_rpy_deg'] is not None:
        transform(l['body_from_lidar_override_xyz_rpy_deg'])
    transform(c['vehicle']['body_from_arm_xyz_rpy_deg'])
    finite(c['vehicle']['grasp_reference_gripper_m'], (3,))
    b = finite(c['vehicle']['body_box_xyz_size_m'], (6,)); positive(b[3:])
    finite(c['vehicle']['ground_z_m'], ())
    finite(c['vehicle']['attitude_sweep_deg'])
    if len(c['vehicle']['attitude_sweep_deg']) == 0:
        raise ValueError('Empty attitude sweep')
    pose_transform([*t['position_world_m'], *t['quaternion_world_wxyz']])
    transform(t['object_from_grasp_xyz_rpy_deg'])
    positive(t['candidate_total_mass_kg'], ())
    positive(t['support_size_m'], (3,)); transform(t['object_from_support_xyz_rpy_deg'])
    names = [p['name'] for p in t['parts']]
    if len(names) != len(set(names)) or not {'body', 'key', 'handle'} <= set(names):
        raise ValueError('Target requires unique body/key/handle parts')
    for p in t['parts']:
        positive(p['size_m'], (3,)); transform(p['xyz_rpy_deg'])
    if (not set(t['observation_parts']) <= set(names) or not t['observation_parts'] or
            t['observation_reference_part'] not in names or t['grasp_part'] not in names):
        raise ValueError('Unknown observation/grasp part')
    positive(t['nominal_grasp_section_width_m'], ())
    for k in ('horizontal_distances_m',):
        positive(o[k])
    finite(o['lidar_above_reference_m']); finite(o['reference_to_lidar_azimuth_deg'], ())
    if not o['horizontal_distances_m'] or not o['lidar_above_reference_m']:
        raise ValueError('Empty observation sweep')
    if type(o['surface_grid_per_edge']) is not int or not 3 <= o['surface_grid_per_edge'] <= 15:
        raise ValueError('surface_grid_per_edge must be 3..15')
    for k in ('primary', 'backup'):
        positive(o[k]['horizontal_m'], ()); finite(o[k]['lidar_above_reference_m'], ())
    return c


def sensor_transform(c):
    l = c['lidar']
    override = l['body_from_lidar_override_xyz_rpy_deg']
    return (transform(override) if override is not None else
            transform(l['body_from_base_xyz_rpy_deg']) @ transform(l['base_from_lidar_xyz_rpy_deg']))


def samples(T, half, count):
    """Face grid with normals; shared edges intentionally counted per face."""
    points, normals = [], []
    for axis, sign in itertools.product(range(3), (-1., 1.)):
        others = [i for i in range(3) if i != axis]
        for u, v in itertools.product(np.linspace(-1, 1, count), repeat=2):
            p = np.zeros(3); p[axis] = sign*half[axis]
            p[others] = half[others]*[u, v]
            points.append(T[:3, :3]@p+T[:3, 3])
            normals.append(T[:3, axis]*sign)
    return np.asarray(points), np.asarray(normals)


def blocked(start, ends, T, half):
    """Segment/OBB slab test. Endpoint-only contact is not an occlusion."""
    start = (inverse(T)@np.r_[start, 1])[:3]
    end = (ends-T[:3, 3])@T[:3, :3]
    delta = end-start
    parallel = abs(delta) < 1e-12
    outside = np.any(parallel & (abs(start) > half+1e-10), axis=1)
    safe = np.where(parallel, 1., delta)
    a, b = (-half-start)/safe, (half-start)/safe
    near = np.max(np.where(parallel, -np.inf, np.minimum(a, b)), axis=1)
    far = np.min(np.where(parallel, np.inf, np.maximum(a, b)), axis=1)
    return ~outside & (far >= np.maximum(near, 1e-7)) & (near < 1-1e-6)


def elevation_range(points_lidar):
    return np.rad2deg(np.arctan2(points_lidar[:, 2], np.linalg.norm(points_lidar[:, :2], axis=1)))


def visibility(c, M_L, parts, blockers):
    l = c['lidar']; lo, hi = l['vertical_fov_deg']; result = {}
    sensor = M_L[:3, 3]
    for name in c['target']['observation_parts']:
        T, half = parts[name]
        pts, normal = samples(T, half, c['observations']['surface_grid_per_edge'])
        local = (pts-sensor)@M_L[:3, :3]
        beta = elevation_range(local); ranges = np.linalg.norm(local, axis=1)
        front = np.einsum('ij,ij->i', sensor-pts, normal) > 1e-10
        in_fov = ((beta >= lo+l['fov_margin_deg']) & (beta <= hi-l['fov_margin_deg']) &
                  (ranges >= l['min_range_m']) & (ranges <= l['max_range_m']))
        target_block = np.zeros(len(pts), dtype=bool)
        for other, (O, h) in parts.items():
            if other != name:
                target_block |= blocked(sensor, pts, O, h)
        vehicle_block = np.zeros(len(pts), dtype=bool)
        for O, h in blockers:
            vehicle_block |= blocked(sensor, pts, O, h)
        visible = front & in_fov & ~target_block & ~vehicle_block
        result[name] = dict(elevation_min_deg=float(min(beta)), elevation_max_deg=float(max(beta)),
            minimum_fov_margin_deg=float(min(min(beta)-lo, hi-max(beta))),
            sampled_visible_fraction=float(np.mean(visible)),
            sampled_visible_faces=int(len(set(np.where(visible)[0]//(c['observations']['surface_grid_per_edge']**2)))),
            all_surface_samples_in_fov=bool(in_fov.all()))
    return result


def box_gap(A, ah, B, bh):
    """Conservative distance lower bound from three orthogonal projection frames."""
    gap = 0.
    for frame in (np.eye(4), inverse(A), inverse(B)):
        a, b = frame@A, frame@B
        ae, be = abs(a[:3, :3])@ah, abs(b[:3, :3])@bh
        d = np.maximum(np.maximum(b[:3, 3]-be-a[:3, 3]-ae, a[:3, 3]-ae-b[:3, 3]-be), 0.)
        gap = max(gap, float(np.linalg.norm(d)))
    return gap


def load_arm(root):
    root = Path(root).expanduser().resolve()
    tests = root/'src/modules/arm_control/tests'
    sys.path[:0] = [str(tests/'hover'), str(tests/'grasp')]
    # Reuse the tested position-IK branch and production SDF boxes. No runtime client.
    from fixed_demo_plan import FixedDemoPlan, START, origins, mount_transform, fk
    p = FixedDemoPlan(tests/'hover/known_target_fixed.json')
    return p, START, origins, mount_transform, fk


def check(c, root):
    plan, start, origins, mount, fk = load_arm(root)
    target = c['target']; lidar = c['lidar']; vehicle = c['vehicle']
    M_O = pose_transform([*target['position_world_m'], *target['quaternion_world_wxyz']])
    parts = {p['name']: (M_O@transform(p['xyz_rpy_deg']), np.asarray(p['size_m'])/2) for p in target['parts']}
    A_gripper = np.eye(4)
    for o, q in zip(origins, plan.expected_q):
        A_gripper = A_gripper@o@mount([0, 0, 0, 0, 0, q])
    A_G = A_gripper.copy()
    A_G[:3, 3] = (A_gripper@np.r_[c['vehicle']['grasp_reference_gripper_m'], 1])[:3]
    B_A = transform(vehicle['body_from_arm_xyz_rpy_deg']); M_G = M_O@transform(target['object_from_grasp_xyz_rpy_deg'])
    M_B = np.eye(4); M_B[:3, 3] = M_G[:3, 3]-(B_A@A_G)[:3, 3]
    M_A = M_B@B_A; B_L = sensor_transform(c)
    support = (M_O@transform(target['object_from_support_xyz_rpy_deg']), np.asarray(target['support_size_m'])/2)
    inverse_old_arm = inverse(plan.arm)
    def arm_boxes(q, jaw, arm=M_A):
        return [(name, arm@inverse_old_arm@T, h) for name, T, h in plan.boxes(q, jaw)]
    pads = {'gripper_fixed_finger_collision', 'moving_jaw_collision'}
    worst = dict(slack_m=1e9)
    minimum_body = 1e9; open_pad_gap = 1e9
    def audit(q, jaw, phase, object_delta=None):
        nonlocal worst, minimum_body, open_pad_gap
        current_parts = {name: ((object_delta@T if object_delta is not None else T), h) for name, (T, h) in parts.items()}
        for name, T, h in arm_boxes(q, jaw):
            zmin = T[2, 3]-abs(T[2, :3])@h
            rows = [('ground', zmin-vehicle['ground_z_m'], .016)]
            for part, (P, ph) in current_parts.items():
                gap = box_gap(T, h, P, ph)
                if name in pads and part == target['grasp_part'] and phase != 'unfold':
                    if phase == 'route': open_pad_gap = min(open_pad_gap, gap)
                    continue
                if part != target['grasp_part']: minimum_body = min(minimum_body, gap)
                rows.append((part, gap, .016))
            rows.append(('support', box_gap(T, h, *support), .003 if name in pads and phase != 'unfold' else .016))
            for part, gap, required in rows:
                if gap-required < worst['slack_m']:
                    worst = dict(phase=phase, collision=name, obstacle=part, distance_m=gap,
                                 required_m=required, slack_m=gap-required)
    for i, q in enumerate(plan.samples): audit(q, .3, 'unfold' if i <= 130 else 'route')
    for jaw in np.linspace(.04, .3, 53): audit(plan.expected_q, jaw, 'jaw_sweep')
    for q in plan.lift_samples:
        now = np.eye(4)
        for o, value in zip(origins, q): now = now@o@mount([0, 0, 0, 0, 0, value])
        audit(q, .04, 'rigid_lift', M_A@now@inverse(A_gripper)@inverse(M_A))
    orientation_error = math.degrees(math.acos(np.clip((np.trace((M_A@A_G)[:3, :3].T@M_G[:3, :3])-1)/2, -1, 1)))
    handle = next(p for p in target['parts'] if p['name'] == target['grasp_part'])
    H_G = inverse(parts[target['grasp_part']][0])@M_G
    handle_angle = math.degrees(math.acos(np.clip((np.trace(H_G[:3, :3])-1)/2, -1, 1)))
    section_ok = bool(abs(target['nominal_grasp_section_width_m']-.020) < 1e-9 and
        np.allclose(handle['size_m'][:2], [.020, .020], atol=1e-9, rtol=0) and
        np.all(abs(H_G[:2, 3]) <= .001) and
        abs(H_G[2, 3]) <= handle['size_m'][2]/2-.007 and handle_angle <= 5)
    arm_report = dict(position_ik='existing_offline_approximation_not_native_or_dynamic_proof',
        final_q_rad=plan.expected_q.tolist(), tcp_arm_m=fk(plan.expected_q).tolist(),
        hover_body_world_xyz_m=M_B[:3, 3].tolist(), orientation_error_deg=orientation_error,
        worst_pair=worst, min_arm_to_body_or_key_m=minimum_body, open_route_pad_handle_gap_m=open_pad_gap,
        handle_section_matches_tested_20mm=section_ok,
        grasp_reference_in_handle_m=H_G[:3, 3].tolist(),
        sampled_geometry_feasible=bool(worst['slack_m'] > 0 and open_pad_gap > .001 and orientation_error <= 5 and section_ok))
    # Body-relative occluders, including folded arm. No double-application of world pose.
    body = finite(vehicle['body_box_xyz_size_m'], (6,))
    B_plate = transform(lidar['body_from_base_xyz_rpy_deg'])@transform([0, 0, -lidar['plate_size_m'][2]/2, 0, 0, 0])
    blockers = [(transform([*body[:3], 0, 0, 0]), body[3:]/2), (B_plate, np.asarray(lidar['plate_size_m'])/2)]
    blockers += [(inverse(M_B)@T, h) for _, T, h in arm_boxes(start, .3)]
    ref = parts[target['observation_reference_part']][0][:3, 3]
    obs = c['observations']; az = math.radians(obs['reference_to_lidar_azimuth_deg'])
    rows = []
    cases = list(itertools.product(obs['horizontal_distances_m'], obs['lidar_above_reference_m']))
    for k in ('primary', 'backup'):
        case = (obs[k]['horizontal_m'], obs[k]['lidar_above_reference_m'])
        if case not in cases: cases.append(case)
    for distance, above in cases:
        sensor = ref+np.array([distance*math.cos(az), distance*math.sin(az), above])
        body_pose = transform([0, 0, 0, 0, 0, math.degrees(az)+180])
        body_pose[:3, 3] = sensor-body_pose[:3, :3]@B_L[:3, 3]
        variants = []
        for roll, pitch in itertools.product(vehicle['attitude_sweep_deg'], repeat=2):
            b = body_pose@transform([0, 0, 0, roll, pitch, 0])
            variants.append(visibility(c, b@B_L, parts, [(b@T, h) for T, h in blockers]))
        nominal = visibility(c, body_pose@B_L, parts, [(body_pose@T, h) for T, h in blockers])
        full = all(all(p['all_surface_samples_in_fov'] for p in v.values()) for v in variants)
        visible = all(all(p['sampled_visible_fraction'] > 0 for p in v.values()) for v in variants)
        rows.append(dict(horizontal_m=float(distance), lidar_above_reference_m=float(above),
            body_world_xyz_m=body_pose[:3, 3].tolist(), lidar_world_xyz_m=sensor.tolist(),
            nominal=nominal, all_attitudes_body_and_key_in_fov=full,
            all_attitudes_body_and_key_partly_visible=visible,
            robust_min_fov_margin_deg=min(p['minimum_fov_margin_deg'] for v in variants for p in v.values()),
            geometric_candidate=bool(full and visible)))
    selected = {k: next(r for r in rows if r['horizontal_m'] == obs[k]['horizontal_m'] and
                    r['lidar_above_reference_m'] == obs[k]['lidar_above_reference_m']) for k in ('primary', 'backup')}
    grasp_blockers = [(M_B@T, h) for T, h in blockers[:2]]
    grasp_blockers += [(T, h) for _, T, h in arm_boxes(plan.expected_q, .3)]
    grasp_view = visibility(c, M_B@B_L, parts, grasp_blockers)
    plate = np.asarray(lidar['plate_size_m']); housing = np.asarray(lidar['housing_size_m'])
    exposed = float(plate[0]*plate[1]-housing[0]*housing[1])
    B_housing = transform(lidar['body_from_base_xyz_rpy_deg'])@transform([0, 0, housing[2]/2, 0, 0, 0])
    mount_body_gap = min(box_gap(T, h, *blockers[0]) for T, h in
                         ((B_plate, plate/2), (B_housing, housing/2)))
    plate_fit = bool(np.all(plate[:2] >= housing[:2]+2*lidar['peripheral_air_gap_m']) and
                     plate[2] >= .003 and exposed >= lidar['minimum_exposed_top_area_m2'] and mount_body_gap >= .016)
    return dict(profile=c['profile'], offline_only=True, motion_permitted=False, scan_simulated=False,
        hardware_calibrated=False, runtime_profile_updated=False,
        transforms=dict(body_from_lidar=B_L.tolist(), body_from_arm=B_A.tolist(), object_from_grasp=transform(target['object_from_grasp_xyz_rpy_deg']).tolist()),
        mounting=dict(plate_size_m=plate.tolist(), housing_size_m=housing.tolist(),
            plate_mass_kg=float(np.prod(plate)*lidar['plate_density_kg_m3']),
            sensor_plus_plate_mass_kg=float(lidar['mass_kg']+np.prod(plate)*lidar['plate_density_kg_m3']),
            exposed_plate_top_area_m2=exposed, dimensional_plate_check_passed=plate_fit,
            mount_to_central_body_box_gap_m=mount_body_gap,
            body_from_base_xyz_rpy_deg=lidar['body_from_base_xyz_rpy_deg']),
        target_geometry=target, observations=rows, selected=selected, grasp_pose_view=grasp_view, arm_geometry=arm_report,
        s1_geometric_precheck_passed=bool(plate_fit and arm_report['sampled_geometry_feasible'] and all(r['geometric_candidate'] for r in selected.values())),
        limitations=['No scan density/noise/registration/dynamic-flight or physical-grasp validation',
            'Box samples are conservative prechecks, not continuous swept safety proof',
            'Frame convention is ENU/FLU; NED/FRD measurements require explicit conversion',
            'Plate holes/cables/heat transfer/support structure and complete aircraft mass/inertia need measured design',
            'Runtime cube20/contact guards unchanged; new body/handle requires separate S6 profile'])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', type=Path, default=BRIDGE/'config/perception/mid360s_s1.yaml')
    ap.add_argument('--px4-root', type=Path, default=Path.home()/'PX4-RoboMaster')
    ap.add_argument('--output', type=Path, required=True, help='Directory for report.json and observations.csv')
    args = ap.parse_args()
    try:
        c = load_profile(args.config); report = check(c, args.px4_root)
    except (ValueError, KeyError, TypeError) as e:
        ap.error(str(e))
    report['input_config'] = str(args.config.resolve())
    report['input_sha256'] = hashlib.sha256(args.config.read_bytes()).hexdigest()
    root = args.px4_root.expanduser().resolve()
    source_paths = [root/'src/modules/arm_control/tests/hover/fixed_demo_plan.py',
                    root/'src/modules/arm_control/tests/hover/known_target_fixed.json',
                    root/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/so101/so101.sdf',
                    BRIDGE/'config/grasp_frame_audit.yaml', SCRIPT]
    report['geometry_source_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (args.output/'profile_used.yaml').write_text(yaml.safe_dump(c, sort_keys=False))
    with (args.output/'observations.csv').open('w', newline='') as f:
        w = csv.writer(f); w.writerow(['horizontal_m', 'lidar_above_body_reference_m', 'body_x_m', 'body_y_m', 'body_z_m',
            'nominal_body_min_elevation_deg', 'nominal_body_max_elevation_deg', 'robust_min_fov_margin_deg', 'geometric_candidate'])
        for r in report['observations']:
            b = r['nominal']['body']
            w.writerow([r['horizontal_m'], r['lidar_above_reference_m'], *r['body_world_xyz_m'], b['elevation_min_deg'],
                b['elevation_max_deg'], r['robust_min_fov_margin_deg'], r['geometric_candidate']])
    print(json.dumps(dict(passed=report['s1_geometric_precheck_passed'], mounting=report['mounting'],
        selected={k:dict(body_world_xyz_m=v['body_world_xyz_m'], margin_deg=v['robust_min_fov_margin_deg'],
                        passed=v['geometric_candidate']) for k, v in report['selected'].items()},
        arm_geometry=report['arm_geometry'], report=str(args.output/'report.json')), indent=2, allow_nan=False))
    return 0 if report['s1_geometric_precheck_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
