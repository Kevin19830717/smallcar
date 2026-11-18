"""Sparse depth geometry; metres, base axes x forward/y left/z up."""
import math
import numpy as np


def project_depth(depth, k, stride=3):
    if not (k[0] > 0 and k[4] > 0):
        raise ValueError('Invalid camera intrinsics')
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z >= .25) & (z <= 2.)
    z=z[valid];u=u[valid];v=v[valid]
    return np.stack(((u-k[2])*z/k[0], (v-k[5])*z/k[4], z), -1)


def transform_points(points, t, height_correction=0., pitch_correction_deg=0.):
    q = t.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rot = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                    [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                    [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    a = math.radians(pitch_correction_deg)
    correction = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0],
                           [-math.sin(a), 0, math.cos(a)]])
    # Correct orientation about the lens, not about the chassis origin.
    origin = np.array([t.translation.x, t.translation.y,
                       t.translation.z+height_correction])
    return points @ rot.T @ correction.T + origin


def filter_points(points, floor_height=0., min_height=.025, max_height=.35):
    p = points[np.isfinite(points).all(axis=1)]
    p = p[(p[:, 0] > .20) & (p[:, 0] < 2.) & (abs(p[:, 1]) < 1.2)]
    # Preserve only measured returns. Missing depth NEVER generates free rays.
    clear = p[(p[:, 2] > floor_height-.015) & (p[:, 2] < floor_height+.38)]
    obs = p[(p[:, 2] >= floor_height+min_height) & (p[:, 2] <= floor_height+max_height)]
    if len(obs):
        # Reject isolated flying pixels using horizontal 5 cm support cells.
        _, inverse, counts = np.unique(np.floor(obs[:, :2]/.05).astype(int),
                                       axis=0, return_inverse=True, return_counts=True)
        obs = obs[counts[inverse] >= 3]
        if len(obs):
            _, indices = np.unique(np.floor(obs/.025).astype(int), axis=0, return_index=True)
            obs = obs[indices]
    return obs, clear


def vision_failure(required, status, received_age):
    if not required:
        return ''
    if received_age > .7:
        return '深度避障数据中断，请停车检查相机'
    if not (status.get('enabled') and status.get('ready') and status.get('calibration_verified')):
        return '深度避障未就绪：'+str(status.get('error', '等待标定或有效深度'))
    return ''
