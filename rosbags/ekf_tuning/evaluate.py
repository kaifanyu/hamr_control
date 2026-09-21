"""Shared evaluation: equal-time sampling, common initial pose, no path fitting."""
from pathlib import Path

import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def unique_rows(a, time_col=0):
    a = a[np.argsort(a[:, time_col], kind='stable')]
    _, idx = np.unique(a[:, time_col], return_index=True)
    return a[idx]


def interp_pose(times, a):
    """Interpolate [time,x,y,yaw] with continuous yaw."""
    a = unique_rows(a)
    return np.column_stack([np.interp(times, a[:, 0], a[:, i] if i < 3 else
                                      np.unwrap(a[:, i])) for i in range(1, 4)])


def relative_pose(p):
    c, s = np.cos(p[0, 2]), np.sin(p[0, 2])
    xy = p[:, :2] - p[0, :2]
    return np.column_stack([c * xy[:, 0] + s * xy[:, 1],
                            -s * xy[:, 0] + c * xy[:, 1], p[:, 2] - p[0, 2]])


class Recording:
    def __init__(self, path):
        self.name = Path(path).stem
        with np.load(path) as d:
            self.data = dict(d)
        w, i, v, r = (self.data[k] for k in ('wheel', 'imu', 'vicon', 'recorded'))
        # Source clocks of Vicon and robot are not synchronized; receipt times
        # define reference time. All candidates have exactly the same interval.
        start = max(w[0, 0], i[0, 0], v[0, 1], r[0, 0]) + 0.1
        end = min(w[-1, 0], i[-1, 0], v[-1, 1], r[-1, 0]) - 0.1
        self.times = np.arange(start, end, 0.02)
        self.reference = relative_pose(interp_pose(self.times, v[:, [1, 2, 3, 5]]))
        vt = unique_rows(v, 1)[:, 1]
        j = np.clip(np.searchsorted(vt, self.times), 1, len(vt)-1)
        self.valid = (vt[j] - vt[j-1]) <= 0.1
        # A common fixed reference-gap mask is applied to every candidate.
        self.recorded_pose = r[:, [0, 2, 3, 5]]

    def events(self, parameters=None):
        p = parameters or {}
        w, i = self.data['wheel'], self.data['imu']
        wheel = np.zeros((len(w), 8))
        wheel[:, 0] = w[:, 0] + p.get('wheel_time_offset', 0.)
        wheel[:, 2] = w[:, 6] * p.get('vx_scale', 1.)
        wheel[:, 3] = w[:, 7] * p.get('vy_scale', 1.)
        wheel[:, 5] = w[:, 8] * p.get('wheel_wz_scale', 1.)
        if 'linear_velocity_scale' in p or 'yaw_rate_scale' in p or 'lever_arm' in p:
            # Recorded publisher rotated old-heading world velocity by the new
            # heading. Recover axle speed before applying calibrated kinematics.
            dt = np.r_[0., np.diff(w[:, 0])]
            delta = w[:, 8] * dt
            axle = np.sin(delta) * w[:, 6] + np.cos(delta) * w[:, 7]
            wheel[:, 3] = axle * p.get('linear_velocity_scale', 1.)
            wheel[:, 5] = w[:, 8] * p.get('yaw_rate_scale', 1.)
            wheel[:, 2] = -p.get('lever_arm', .45) * wheel[:, 5]
        imu = np.zeros((len(i), 8))
        imu[:, 0] = i[:, 0] + p.get('imu_time_offset', 0.)
        imu[:, 1] = 1
        imu[:, 4] = i[:, 2]
        imu[:, 5] = (i[:, 3] - p.get('gyro_bias', 0.)) * p.get('gyro_scale', 1.)
        imu[:, 6:8] = i[:, 4:6]
        if p.get('deduplicate_imu', False):
            # Exact repeated complete payloads are not independent samples.
            keep = np.r_[True, np.any(np.diff(i[:, 2:], axis=0) != 0, axis=1)]
            imu = imu[keep]
        events = np.vstack([wheel, imu])
        return events[np.argsort(events[:, 0], kind='stable')]

    def score(self, trajectory, return_trace=False):
        trajectory = np.asarray(trajectory)
        if (trajectory.ndim != 2 or trajectory.shape[1] < 4 or len(trajectory) < 2
                or not np.isfinite(trajectory).all()):
            raise ValueError('trajectory must contain at least two finite [t,x,y,yaw] rows')
        if np.min(trajectory[:, 0]) > self.times[0] or np.max(trajectory[:, 0]) < self.times[-1]:
            raise ValueError('trajectory does not cover the fixed evaluation interval')
        pose = relative_pose(interp_pose(self.times, trajectory[:, :4]))
        errors = pose - self.reference
        errors[:, 2] = wrap(errors[:, 2])
        e = errors[self.valid]
        distance = np.linalg.norm(e[:, :2], axis=1)
        result = {'xy_rmse_m': float(np.sqrt(np.mean(distance**2))),
                  'yaw_rmse_deg': float(np.rad2deg(np.sqrt(np.mean(e[:, 2]**2)))),
                  'final_xy_m': float(distance[-1]),
                  'p95_xy_m': float(np.quantile(distance, .95)),
                  'max_xy_m': float(distance.max()),
                  'final_yaw_deg': float(np.rad2deg(e[-1, 2])),
                  'duration_s': float(self.times[-1]-self.times[0]),
                  'sample_count': int(self.valid.sum()),
                  'final_valid_time_s': float(self.times[self.valid][-1]),
                  'reference_gap_samples_excluded': int((~self.valid).sum())}
        if return_trace:
            return result, np.column_stack([self.times, self.reference, pose, errors, self.valid])
        return result


def load_recordings(directory=None):
    directory = Path(directory) if directory else Path(__file__).parent / 'data'
    return [Recording(p) for p in sorted(directory.glob('hamr_hw_*.npz'))]


if __name__ == '__main__':
    import json
    for recording in load_recordings():
        print(recording.name, json.dumps(recording.score(recording.recorded_pose)))
