"""Thread-safe ctypes adapter for the unmodified robot_localization EKF core.

Build with ``bash engine_build.sh``. Inputs must already use base_link axes and
have monotonic, common relative seconds. See engine_README.md for scope.
"""

from pathlib import Path
import ctypes
import numpy as np


DEFAULT_Q = np.array([
    .05, .05, .06, .03, .03, .06, .025, .025, .04,
    .01, .01, .02, .01, .01, .015,
], dtype=np.float64)
DEFAULT_P = np.array([1e-9] * 6 + [1.] * 9, dtype=np.float64)
# R order: wheel vx,vy,yaw,wz; IMU yaw,wz,ax,ay.
# Original recorded message variances; calibrated runs pass their own profile.
DEFAULT_R = np.array([.01, .01, .01, .01, .0009, .0002, .01, .01])
DEFAULT_MASK = np.array([1, 1, 0, 0, 0, 1, 0, 0], dtype=np.int32)
STATE_NAMES = (
    "x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz",
    "vroll", "vpitch", "vyaw", "ax", "ay", "az",
)


class Engine:
    """Each call creates a fresh EKF; calls may run concurrently in threads."""

    def __init__(self, path=None):
        self._library = ctypes.CDLL(str(path or Path(__file__).with_name("engine_core.so")))
        self._run = self._library.hamr_ekf_run
        f64 = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
        i32 = np.ctypeslib.ndpointer(dtype=np.int32, flags="C_CONTIGUOUS")
        self._run.argtypes = [
            f64, ctypes.c_int64, f64, f64, f64, i32,
            ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, f64,
        ]
        self._run.restype = ctypes.c_int

    def run(self, events, q_diag=None, r_diag=None, mask=None, p_diag=None,
            relative_imu=True, relative_wheel=False, dynamic_q=False,
            full_state=False):
        """Replay rows [t,type,vx,vy,yaw,wz,ax,ay] in ascending time order.

        ``type`` is 0 for wheel and 1 for IMU. ``mask`` and ``r_diag`` have
        eight entries: wheel vx,vy,yaw,wz; IMU yaw,wz,ax,ay. Q and P follow
        STATE_NAMES. Returns [t,x,y,yaw,vx,vy,wz], or the 15-state array when
        ``full_state=True``. Input IMU yaw must be absolute if relative_imu is
        true; the first yaw is subtracted here. R values are variances.
        """
        events = np.ascontiguousarray(events, dtype=np.float64)
        if events.ndim != 2 or events.shape[1] != 8:
            raise ValueError("events must have shape (N, 8)")
        if not np.isfinite(events).all() or np.any(np.diff(events[:, 0]) < 0):
            raise ValueError("events must be finite and sorted by time")
        if len(events) and (events[0, 0] < -1 or not np.isin(events[:, 1], (0, 1)).all()):
            raise ValueError("time must be >= -1 and sensor type must be 0 or 1")
        q = np.ascontiguousarray(DEFAULT_Q if q_diag is None else q_diag, dtype=np.float64)
        p = np.ascontiguousarray(DEFAULT_P if p_diag is None else p_diag, dtype=np.float64)
        r = np.ascontiguousarray(DEFAULT_R if r_diag is None else r_diag, dtype=np.float64)
        selection = np.ascontiguousarray(DEFAULT_MASK if mask is None else mask, dtype=np.int32)
        if q.shape != (15,) or p.shape != (15,) or r.shape != (8,) or selection.shape != (8,):
            raise ValueError("Q and P must have 15 entries; R and mask must have eight")
        if not all(np.isfinite(v).all() for v in (q, p, r)):
            raise ValueError("covariance diagonals must be finite")
        if np.any(q < 0) or np.any(p < 0) or np.any(r <= 0):
            raise ValueError("Q/P must be nonnegative, R must be positive")
        output = np.empty((len(events), 15), dtype=np.float64)
        result = self._run(events, len(events), q, p, r, selection,
                           relative_imu, relative_wheel, dynamic_q, output)
        if result:
            raise RuntimeError("robot_localization EKF failed during replay")
        if full_state:
            return output
        return np.column_stack((events[:, 0], output[:, [0, 1, 5, 6, 7, 11]]))
