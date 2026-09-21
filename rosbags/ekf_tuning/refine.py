#!/usr/bin/env python3
"""Fit a small deployable calibrated wheel+gyro model with held-out bags."""
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
from pathlib import Path
import time

import numpy as np
from scipy.optimize import least_squares

from engine import Engine
from evaluate import load_recordings, interp_pose, relative_pose, wrap
from tune import evaluate, settings


def parameters(x):
    return {'mode': 'wheel_gyro', 'linear_velocity_scale': float(x[0]),
            'yaw_rate_scale': float(x[1]), 'lever_arm': float(x[2]),
            'r_vx': float(10**x[3]), 'r_vy': float(10**x[3]),
            'r_wheel_wz': float(10**x[4]), 'r_gyro': float(10**x[5])}


def fit(recordings, train, seed, label):
    engine = Engine()
    history = []
    def residual(x):
        p = parameters(x)
        q, r, mask = settings(p)
        result = []
        for index in train:
            recording = recordings[index]
            out = engine.run(recording.events(p), q_diag=q, r_diag=r, mask=mask)
            pose = relative_pose(interp_pose(recording.times, out[:, :4]))
            err = pose - recording.reference
            err[:, 2] = .35 * wrap(err[:, 2])
            # Every fifth common-grid point keeps fits fast and weights runs equally.
            err = err[recording.valid][::5]
            result.extend((err / np.sqrt(len(err))).ravel())
        history.append(float(np.linalg.norm(result)))
        return np.array(result)
    start = time.monotonic()
    result = least_squares(residual, seed, bounds=(
        [.90, .77, .15, -5, -5, -4], [1.03, .90, .40, -.3, -.5, 1.]),
        diff_step=1e-4, max_nfev=100, ftol=1e-7, xtol=1e-6, gtol=1e-6,
        x_scale=[.05, .05, .15, 2., 2., 2.])
    report = evaluate(parameters(result.x), recordings, engine)
    report.update({'training_bags': train, 'label': label, 'cost': result.cost,
                   'function_evaluations': len(history), 'elapsed_s': time.monotonic()-start,
                   'success': bool(result.success), 'message': result.message})
    print(label, 'evaluations', len(history), 'xy', [round(s['xy_rmse_m'],4) for s in report['scores']], flush=True)
    return report


if __name__ == '__main__':
    recordings = load_recordings()
    jobs = []
    seed = [.955, .821, .26, -2., -3., -1.]
    for heldout in range(3):
        jobs.append(([i for i in range(3) if i != heldout], seed, 'heldout_' + str(heldout)))
    for n, rv in enumerate([-4., -2., -.7]):
        jobs.append(([0, 1, 2], [.955, .821, .26, rv, -3., -1.], 'all_' + str(n)))
    output = Path(__file__).parent / 'results/refined.jsonl'
    with output.open('w') as f, ThreadPoolExecutor(6) as pool:
        futures = [pool.submit(fit, recordings, *job) for job in jobs]
        for future in as_completed(futures):
            f.write(json.dumps(future.result()) + '\n')
            f.flush()
