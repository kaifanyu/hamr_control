#!/usr/bin/env python3
"""Regenerate frozen-profile metrics, trajectories and publication-ready plots."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter

from engine import Engine
from evaluate import load_recordings
from tune import settings


def main():
    root = Path(__file__).parent
    output = root / 'report'
    output.mkdir(exist_ok=True)
    chosen = json.loads((output / 'selected.json').read_text())['parameters']
    engine = Engine()
    recordings = load_recordings()
    results = {'selected_parameters': chosen, 'bags': {}, 'cross_validation': [],
               'method': {'sample_period_s': .02, 'reference': 'Vicon receipt timestamps',
                          'alignment': 'common overlap; initial SE(2) only; no trajectory fitting',
                          'reference_gap_limit_s': .1,
                          'moving_mask': 'smoothed Vicon speed > 0.03 m/s OR |yaw rate| > 0.06 rad/s; 0.42 s Savitzky-Golay window',
                          'fusion_inputs': 'wheel body vx/vy/yaw rate + IMU gyro z; Vicon scoring only'}}
    profiles = {'Original gyro profile': {'mode': 'gyro'}, 'Tuned EKF': chosen}
    cov_best = root / 'results/covariance.best.json'
    if cov_best.exists():
        profiles['Covariance only'] = json.loads(cov_best.read_text())['parameters']
    fig, axes = plt.subplots(3, 3, figsize=(15, 12), constrained_layout=True)
    colors = {'Recorded EKF': '#D87928', 'Original gyro profile': '#999999', 'Tuned EKF': '#0077AA'}
    for col, recording in enumerate(recordings):
        moving = np.linalg.norm(savgol_filter(recording.reference[:, :2], 21, 3,
                    deriv=1, delta=.02, axis=0), axis=1) > .03
        moving |= np.abs(savgol_filter(np.unwrap(recording.reference[:, 2]), 21, 3,
                                      deriv=1, delta=.02)) > .06
        moving &= recording.valid
        estimates = {'Recorded EKF': recording.recorded_pose}
        for label, params in profiles.items():
            q, r, mask = settings(params)
            estimates[label] = engine.run(recording.events(params), q_diag=q, r_diag=r, mask=mask)
        axes[0, col].plot(recording.reference[:, 0], recording.reference[:, 1],
                          color='#111111', lw=2.1, label='Vicon')
        bag_scores = {}
        for label, trajectory in estimates.items():
            score, trace = recording.score(trajectory, return_trace=True)
            error = trace[:, 7:10]
            score['moving_xy_rmse_m'] = float(np.sqrt(np.mean(np.sum(error[moving, :2]**2, axis=1))))
            score['moving_yaw_rmse_deg'] = float(np.rad2deg(np.sqrt(np.mean(error[moving, 2]**2))))
            score['moving_sample_count'] = int(moving.sum())
            bag_scores[label] = score
            if label == 'Tuned EKF':
                np.savetxt(output / (recording.name + '_trace.csv'), trace, delimiter=',',
                           header='t,reference_x,reference_y,reference_yaw,estimate_x,estimate_y,estimate_yaw,error_x,error_y,error_yaw,valid',
                           comments='')
            if label not in colors:
                continue
            c = colors[label]
            lw = 1.8 if label == 'Tuned EKF' else 1.1
            axes[0, col].plot(trace[:, 4], trace[:, 5], color=c, lw=lw, label=label,
                              alpha=.6 if label == 'Original gyro profile' else 1.)
            e = np.linalg.norm(error[:, :2], axis=1)
            e[~recording.valid] = np.nan
            axes[1, col].plot(recording.times, e, color=c, lw=lw)
            yaw = np.rad2deg(error[:, 2]); yaw[~recording.valid] = np.nan
            axes[2, col].plot(recording.times, yaw, color=c, lw=lw)
        bag_scores['xy_rmse_reduction_percent'] = 100 * (
            1 - bag_scores['Tuned EKF']['xy_rmse_m'] / bag_scores['Recorded EKF']['xy_rmse_m'])
        results['bags'][recording.name] = bag_scores
        axes[0, col].set_title(recording.name[-6:] + '\nXY RMSE: %.1f → %.1f cm' % (
            100*bag_scores['Recorded EKF']['xy_rmse_m'],100*bag_scores['Tuned EKF']['xy_rmse_m']))
        axes[0, col].set_aspect('equal', adjustable='datalim')
        axes[0, col].set(xlabel='x from initial pose (m)', ylabel='y from initial pose (m)')
        axes[1, col].set(xlabel='Time from bag start (s)', ylabel='Position error (m)')
        axes[2, col].set(xlabel='Time from bag start (s)', ylabel='Yaw error (degrees)')
        for row in range(3):
            axes[row, col].grid(alpha=.2)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle('HAMR localization: shared initial alignment, Vicon used only for evaluation', fontsize=15)
    fig.savefig(output / 'comparison.png', dpi=180)
    fig.savefig(output / 'comparison.pdf')
    plt.close(fig)
    refined = root / 'results/refined.jsonl'
    if refined.exists():
        for line in refined.read_text().splitlines():
            row = json.loads(line)
            if row['label'].startswith('heldout_'):
                heldout = int(row['label'].split('_')[-1])
                results['cross_validation'].append({
                    'heldout_bag': recordings[heldout].name,
                    'parameters': row['parameters'], 'score': row['scores'][heldout]})
    results['experiment_counts'] = {}
    for path in sorted((root / 'results').glob('*.jsonl')):
        lines = path.read_text().splitlines()
        results['experiment_counts'][path.stem] = len(lines)
        if path.stem in ('refined', 'variants'):
            results['experiment_counts'][path.stem + '_objective_calls'] = sum(
                json.loads(line)['function_evaluations'] for line in lines)
    (output / 'metrics.json').write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps({name: {k: v[k] for k in ['Recorded EKF', 'Tuned EKF', 'xy_rmse_reduction_percent']}
                      for name, v in results['bags'].items()}, indent=2))


if __name__ == '__main__':
    main()
