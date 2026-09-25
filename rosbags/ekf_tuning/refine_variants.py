#!/usr/bin/env python3
"""Ablations: fit fresh-payload gating and weak quaternion yaw alternatives."""
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from engine import Engine
from evaluate import load_recordings, interp_pose, relative_pose, wrap
from refine import parameters
from tune import evaluate, settings


def fit(recordings, mode, dedup):
    engine = Engine()
    calls = 0
    def params(x):
        p = parameters(x)
        p.update(mode=mode, deduplicate_imu=dedup)
        if len(x) > 6:
            p.update(r_yaw=float(10**x[6]), q_yaw=float(10**x[7]))
        return p
    def residual(x):
        nonlocal calls
        calls += 1
        p = params(x)
        q, r, mask = settings(p)
        errors = []
        for recording in recordings:
            out = engine.run(recording.events(p), q_diag=q, r_diag=r, mask=mask)
            err = relative_pose(interp_pose(recording.times, out[:, :4])) - recording.reference
            err[:, 2] = .35 * wrap(err[:, 2])
            err = err[recording.valid][::5]
            errors.extend((err / np.sqrt(len(err))).ravel())
        return np.array(errors)
    seed = [.95, .813, .2825, np.log10(.000165), np.log10(.004), np.log10(.04)]
    lo, hi = [.9, .77, .15, -5, -5, -4], [1.03, .9, .4, -.3, -.5, 1.]
    if mode == 'wheel_combined':
        seed += [-1., -5.]
        lo += [-4., -8.]
        hi += [1., -1.]
    result = least_squares(residual, seed, bounds=(lo, hi), diff_step=1e-4,
                           max_nfev=100, ftol=1e-6, xtol=1e-6, gtol=1e-6)
    report = evaluate(params(result.x), recordings, engine)
    report.update(function_evaluations=calls, success=bool(result.success), message=result.message)
    print(mode, dedup, calls, [round(s['xy_rmse_m'],4) for s in report['scores']], flush=True)
    return report


if __name__ == '__main__':
    recordings = load_recordings()
    output = Path(__file__).parent / 'results/variants.jsonl'
    with output.open('w') as f, ThreadPoolExecutor(3) as pool:
        jobs = [(mode, dedup) for mode, dedup in
                [('wheel_gyro', True), ('wheel_combined', False), ('wheel_combined', True)]]
        futures = [pool.submit(fit, recordings, *job) for job in jobs]
        for future in as_completed(futures):
            f.write(json.dumps(future.result()) + '\n')
            f.flush()
