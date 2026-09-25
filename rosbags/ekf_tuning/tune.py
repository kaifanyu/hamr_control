#!/usr/bin/env python3
"""Seeded parallel experiments using the upstream EKF; Vicon is scoring only."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import time

import numpy as np
from scipy.stats import qmc

from engine import Engine, DEFAULT_Q
from evaluate import load_recordings


R = np.array([.01, .01, .01, .01, .0009, .0002, .01, .01])
MODES = {'live': [0, 1, 0, 0, 1, 1, 0, 0],
         'gyro': [1, 1, 0, 0, 0, 1, 0, 0],
         'orientation': [1, 1, 0, 0, 1, 0, 0, 0],
         'combined': [1, 1, 0, 0, 1, 1, 0, 0],
         'wheel_gyro': [1, 1, 0, 1, 0, 1, 0, 0],
         'wheel_combined': [1, 1, 0, 1, 1, 1, 0, 0],
         'accel': [1, 1, 0, 0, 0, 1, 1, 1]}


def settings(p):
    q = DEFAULT_Q.copy()
    r = R.copy()
    for key, index in [('q_yaw', 5), ('q_vx', 6), ('q_vy', 7),
                       ('q_wz', 11), ('q_ax', 12), ('q_ay', 13)]:
        if key in p:
            q[index] = p[key]
    for key, index in [('r_vx', 0), ('r_vy', 1), ('r_wheel_wz', 3),
                       ('r_yaw', 4), ('r_gyro', 5), ('r_ax', 6), ('r_ay', 7)]:
        if key in p:
            r[index] = p[key]
    return q, r, MODES[p['mode']]


def evaluate(p, recordings, engine):
    q, r, mask = settings(p)
    scores = []
    for recording in recordings:
        trajectory = engine.run(recording.events(p), q_diag=q, r_diag=r, mask=mask)
        score = recording.score(trajectory)
        # Metres plus a modest endpoint term and heading penalty. Each bag has
        # equal weight, regardless of its duration or Vicon publication rate.
        score['objective'] = (score['xy_rmse_m'] + .2 * score['final_xy_m'] +
                              .2 * np.deg2rad(score['yaw_rmse_deg']))
        scores.append(score)
    return {'parameters': p, 'scores': scores,
            'objective': float(np.mean([s['objective'] for s in scores]))}


def proposals(stage, n):
    modes = ['gyro', 'orientation', 'combined', 'wheel_gyro', 'wheel_combined', 'accel']
    points = qmc.Sobol(16, scramble=True, seed=20260920).random_base2(int(np.ceil(np.log2(n))))[:n]
    for j, point in enumerate(points):
        mode = modes[j % len(modes)]
        p = {'mode': mode}
        for k, value, lo, hi in zip(
            ['r_vx', 'r_vy', 'r_yaw', 'r_gyro', 'r_wheel_wz', 'q_wz', 'q_yaw', 'q_ax', 'r_ax'],
            point, [-5, -5, -4, -5, -5, -4, -5, -5, -2],
            [0, 0, 0, -1, -1, 0, -1, -1, 1]):
            p[k] = float(10**(lo + value*(hi-lo)))
        p['q_ay'] = p['q_ax']
        p['r_ay'] = p['r_ax']
        if stage == 'calibrated':
            p['mode'] = ['gyro', 'combined', 'wheel_gyro', 'wheel_combined'][j % 4]
            p['linear_velocity_scale'] = float(.92 + point[9] * .08)
            p['yaw_rate_scale'] = float(.78 + point[10] * .10)
            p['lever_arm'] = float(.20 + point[11] * .15)
            p['deduplicate_imu'] = bool(int(point[12] * 2))
        yield p


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=['baseline', 'covariance', 'calibrated'], default='baseline')
    parser.add_argument('--trials', type=int, default=1536)
    parser.add_argument('--workers', type=int, default=min(12, os.cpu_count() or 1))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(__file__).parent / 'results' / (args.stage + '.jsonl')
    output.parent.mkdir(parents=True, exist_ok=True)
    recordings, engine = load_recordings(), Engine()
    if args.stage == 'baseline':
        candidates = [{'mode': name, **({'q_yaw': .005, 'q_wz': .01} if name == 'live' else {})}
                      for name in MODES]
        candidates += [{'mode': name, 'deduplicate_imu': True} for name in MODES if name != 'live']
    else:
        candidates = list(proposals(args.stage, args.trials))
    start, best = time.monotonic(), None
    with output.open('w') as file, ThreadPoolExecutor(args.workers) as pool:
        futures = [pool.submit(evaluate, p, recordings, engine) for p in candidates]
        for n, future in enumerate(as_completed(futures), 1):
            result = future.result()
            file.write(json.dumps(result) + '\n')
            file.flush()
            if best is None or result['objective'] < best['objective']:
                best = result
            if n % 64 == 0 or n == len(candidates):
                print(f"{n}/{len(candidates)} {time.monotonic()-start:.1f}s best={best['objective']:.5f}", flush=True)
    output.with_suffix('.best.json').write_text(json.dumps(best, indent=2) + '\n')
    print(json.dumps(best, indent=2))


if __name__ == '__main__':
    main()
