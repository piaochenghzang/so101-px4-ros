"""Pure coordinate math and fail-closed diagnostics, independent of ROS/Gazebo."""
import math
import numpy as np


def finite(value, shape=None):
    out = np.asarray(value, dtype=float)
    if (shape is not None and out.shape != shape) or not np.isfinite(out).all():
        raise ValueError('NONFINITE_OR_SHAPE')
    return out


def quaternion_rotation(q):
    w, x, y, z = finite(q, (4,))
    norm = np.linalg.norm([w, x, y, z])
    if norm < 1e-9 or abs(norm-1) > .01:
        raise ValueError('INVALID_QUATERNION')
    w, x, y, z = np.asarray([w, x, y, z])/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pose_transform(pose):
    pose = finite(pose, (7,))  # xyz, quaternion wxyz
    out = np.eye(4)
    out[:3, :3] = quaternion_rotation(pose[3:])
    out[:3, 3] = pose[:3]
    return out


def rpy_transform(pose):
    x, y, z, r, p, w = finite(pose, (6,))
    cr, sr, cp, sp, cw, sw = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(w), math.sin(w)
    out = np.eye(4)
    out[:3, :3] = [[cw*cp, cw*sp*sr-sw*cr, cw*sp*cr+sw*sr],
                  [sw*cp, sw*sp*sr+cw*cr, sw*sp*cr-cw*sr], [-sp, cp*sr, cp*cr]]
    out[:3, 3] = [x, y, z]
    return out


def inverse(t):
    t = finite(t, (4, 4))
    if not np.allclose(t[3], [0, 0, 0, 1]) or not np.allclose(t[:3, :3].T@t[:3, :3], np.eye(3), atol=1e-8) or np.linalg.det(t[:3, :3]) < .99:
        raise ValueError('NOT_RIGID')
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3]@t[:3, 3]
    return out


def residual(a, b):
    delta = inverse(a)@b
    return (float(np.linalg.norm(delta[:3, 3])),
            math.acos(float(np.clip((np.trace(delta[:3, :3])-1)/2, -1, 1))))


def fk_gripper(config, q):
    q = finite(q, (6,))
    chain = np.eye(4)
    if len(config['joint_origins']) != 5:
        raise ValueError('INVALID_FK_CONFIG')
    for origin, angle in zip(config['joint_origins'], q[:5]):
        chain = chain@rpy_transform(origin)@rpy_transform([0, 0, 0, 0, 0, angle])
    return chain


def interpolate_pose(samples, stamp, max_skew):
    """Bracketed source-time interpolation only; never extrapolate or restamp."""
    before = next((s for s in reversed(samples) if s[0] <= stamp), None)
    after = next((s for s in samples if s[0] >= stamp), None)
    if before is None or after is None:
        raise ValueError('ODOM_NOT_BRACKETED')
    skew = max(stamp-before[0], after[0]-stamp)
    if skew > max_skew+1e-9:
        raise ValueError('SOURCE_SKEW')
    a, b = finite(before[1], (7,)), finite(after[1], (7,))
    pose_transform(a); pose_transform(b)
    if after[0] == before[0]:
        return pose_transform(a), 0.0
    u = (stamp-before[0])/(after[0]-before[0])
    qa, qb = a[3:]/np.linalg.norm(a[3:]), b[3:]/np.linalg.norm(b[3:])
    dot = float(qa@qb)
    if dot < 0:
        qb = -qb; dot = -dot
    if dot > .9995:
        q = (1-u)*qa+u*qb
    else:
        angle = math.acos(np.clip(dot, -1, 1))
        q = (math.sin((1-u)*angle)*qa+math.sin(u*angle)*qb)/math.sin(angle)
    q /= np.linalg.norm(q)
    return pose_transform([*((1-u)*a[:3]+u*b[:3]), *q]), skew


def current_goal(stamp, session_start, latest_request, now):
    return bool(all(math.isfinite(x) for x in (stamp, session_start, latest_request, now))
                and 0 < stamp <= now+.02 and stamp >= max(session_start, latest_request))


class IdentityGuard:
    def __init__(self):
        self.identity = None
        self.stamp = None

    def check(self, identity, stamp):
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError('INVALID_SOURCE_TIME')
        changed = self.identity is not None and identity != self.identity
        rollback = self.stamp is not None and stamp < self.stamp
        duplicate = not changed and self.stamp is not None and stamp == self.stamp
        if duplicate:
            raise ValueError('DUPLICATE_SOURCE')
        self.identity, self.stamp = identity, stamp
        return changed or rollback


class FrozenOrigin:
    """Diagnostic G->L registration. Freeze once; do not fit away later residuals."""
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.candidates = []
        self.transform = None

    def update(self, local_body, truth_body):
        candidate = local_body@inverse(truth_body)
        if self.transform is None:
            if self.candidates:
                dp, dr = residual(self.candidates[0], candidate)
                if dp > self.config['origin_position_scatter_m'] or dr > self.config['origin_rotation_scatter_rad']:
                    self.candidates = []
            self.candidates.append(candidate)
            if len(self.candidates) >= self.config['origin_samples']:
                # Select an actual observed rigid sample, not an unvalidated matrix mean.
                self.transform = self.candidates[len(self.candidates)//2].copy()
        if self.transform is None:
            return None
        return residual(self.transform@truth_body, local_body)


def audit_truth(packet, config):
    """Synchronized actual joint FK vs actual link poses. No command angles used."""
    if packet['frame'] != config['truth_frame'] or packet['vehicle_name'] != config['vehicle_name']:
        raise ValueError('WRONG_FRAME_OR_VEHICLE')
    if packet['target_name'] != config['target_name']:
        raise ValueError('WRONG_TARGET_IDENTITY')
    if not packet['joint_measurement_valid']:
        raise ValueError('STATIC_OR_INVALID_JOINT_MEASUREMENT')
    names = packet['joint_names']
    if len(names) != 6 or len(set(names)) != 6 or set(names) != set(config['joint_names']):
        raise ValueError('INVALID_JOINT_NAMES')
    positions = finite(packet['position'], (6,))
    finite(packet['velocity'], (6,))
    q = [positions[names.index(name)] for name in config['joint_names']]
    links = packet['links']
    required = [config[k] for k in ('body_link', 'arm_link', 'gripper_link', 'lidar_link')]
    if any(name not in links for name in required):
        raise ValueError('MISSING_LINK')
    ids = [links[name]['id'] for name in required]
    if packet['vehicle_id'] <= 0 or any(i <= 0 for i in ids) or len(set(ids)) != len(ids):
        raise ValueError('INVALID_ENTITY_ID')
    body, arm, gripper, lidar = [pose_transform(links[name]['pose']) for name in required]
    mount = rpy_transform(config['mount_pose'])
    fk = fk_gripper(config, q)
    tool = rpy_transform(config['tool_pose'])
    mount_error = residual(body@mount, arm)
    fk_error = residual(arm@fk@tool, gripper@tool)
    lidar_error = residual(body@rpy_transform(config['configured_body_sensor_pose']),
                           lidar@rpy_transform(config['lidar_sensor_pose']))
    reference = finite(config['reference_gripper'], (3,))
    out = dict(truth_valid=True, close_permitted=False, source='SIM_GROUND_TRUTH',
               joint_position_measured=[float(v) for v in q],
               body_pose_world=links[config['body_link']]['pose'],
               arm_pose_world=links[config['arm_link']]['pose'],
               gripper_pose_world=links[config['gripper_link']]['pose'],
               fk_position_error_m=fk_error[0], fk_rotation_error_rad=fk_error[1],
               mount_position_error_m=mount_error[0], mount_rotation_error_rad=mount_error[1],
               lidar_position_error_m=lidar_error[0], lidar_rotation_error_rad=lidar_error[1],
               reference_world=(gripper[:3, 3]+gripper[:3, :3]@reference).tolist(),
               tool_world=(gripper@tool)[:3, 3].tolist(), target_present=packet['target_present'])
    out['chain_valid'] = bool(all(e[0] <= config['fk_position_limit_m'] and e[1] <= config['rotation_limit_rad']
                                  for e in (mount_error, fk_error, lidar_error)))
    if packet['target_present']:
        if packet['target_id'] <= 0:
            raise ValueError('INVALID_TARGET_ID')
        obj = pose_transform(packet['target_pose'])
        object_h = (inverse(gripper)@obj)[:3, 3]
        object_a = (inverse(arm)@obj)[:3, 3]
        error = float(np.linalg.norm(object_h-reference))
        out.update(object_gripper=object_h.tolist(), object_arm=object_a.tolist(),
                   reference_error_m=error,
                   aligned_geometry_only=bool(out['chain_valid'] and error <= config['alignment_diagnostic_limit_m']))
    else:
        out.update(aligned_geometry_only=False, reason='TARGET_MISSING')
    return out, body
