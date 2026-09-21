# Independent evaluation review of frozen EKF candidate

Reviewed `evaluate.py`, `engine.py`, `engine.cpp`, `tune.py`, `refine.py`, source
extraction, saved optimization results, and actual-node baseline validation.
No changes to those files were made during this review.

Frozen candidate used for the independent checks:

```text
mode = wheel_gyro
linear_velocity_scale = 0.95
yaw_rate_scale = 0.813
lever_arm = 0.2825 m
wheel vx/vy variance = 0.000165
wheel yaw-rate variance = 0.004
IMU gyro-z variance = 0.04
Q = original ekf.yaml defaults
no sensor time shifts, no IMU deduplication, no Vicon measurement
```

## Correctness and leakage

- Candidate inputs contain only wheel and IMU measurements. Its calibration
  transformation is causal and does not look up Vicon samples. Vicon affects
  offline parameter selection and scoring only.
- The inverse body-twist correction is mathematically correct for the recorded
  publisher and its pi/2 yaw offset. Wheel source timestamps are monotonic in
  these bags, so consecutive header deltas are the appropriate intervals.
- The first sample uses dt=0 because the preceding integration stamp is absent;
  this affects only that first message and is immaterial to these stationary
  starts. For arbitrary bags beginning in motion it should be documented.
- The common initial time, fixed 50 Hz grid, common reference-gap mask, and one
  initial relative SE(2) alignment make comparisons consistent. There is no
  whole-path alignment or fit that conceals accumulated drift.
- The engine uses the original 15-state robot_localization numerical core,
  diagonal covariances, and the same planar pseudo-measurements. It does not
  reproduce transport, scheduling, or timeout effects; native-node validation
  of the final candidate is still necessary.
- Scalar relative-yaw subtraction does not reproduce full-quaternion relative
  IMU orientation under roll/pitch. This limits some exploratory orientation
  candidates, but does not affect the chosen gyro-only IMU configuration.
- Leave-one-bag-out fits assess parameter sensitivity on these three recordings.
  Because the model family and starting calibration were explored using all
  recordings, this is not a pristine held-out evaluation of the final model.
  The final all-three-bag fit should be labeled fitted replay performance.

## Independent metric sensitivity

`review_metrics.py` reruns the exact core using the frozen values above, applies
a stricter source-header-age/backlog mask plus a 0.1 s edge erosion, and splits
motion from stationary intervals using smoothed Vicon derivatives. It also moves
the common alignment origin later by 0.25, 0.5, and 1.0 s. Complete results are in
`frozen_candidate_review.json`.

| Bag suffix | Primary XY RMSE | Strict-mask RMSE | Moving-only RMSE | Gyro baseline moving-only RMSE |
|---|---:|---:|---:|---:|
| 184054 | 0.05015 m | 0.04985 m | 0.05657 m | 0.38961 m |
| 190842 | 0.10221 m | 0.10078 m | 0.13633 m | 0.41504 m |
| 193228 | 0.06550 m | 0.06712 m | 0.06915 m | 0.42517 m |

The gain survives the stricter validity mask and moving-only scoring. The second
bag contains an approximately 37 s stationary lead-in; reporting its full-run
RMSE alone obscures this difference in motion content. Moving-only results help
show the practical localization performance while driving.

Changing the alignment origin by up to one second changes candidate XY RMSE by
less than 0.7 mm in each bag. The measured improvement is not an initial-alignment
artifact.

Stationary angular-velocity RMS is approximately ten times lower than the original
gyro-only configuration. Over the 37 s stationary interval, candidate yaw drift
is -0.0319 degrees versus -0.2677 degrees for the gyro baseline, with zero estimated
position drift in both. These comparisons use a Vicon-defined stationary mask,
not a candidate-dependent mask. The previous yaw-observing live configuration has
still lower yaw drift during that stop; candidate gyro fusion should not be
described as universally better at every stationary-heading metric.

For the three primary candidate position scores:

- Mean per-bag RMSE: **0.07262 m**.
- Equal-bag RMS, `sqrt(mean(per_bag_MSE))`: **0.07584 m**.
- Pooled-time RMS across all valid samples: **0.08215 m**.

These different aggregations should not all be called the same “overall RMSE.”
Per-bag numbers are the clearest primary presentation.

## Remaining limitations and recommended safeguards

1. `score()` uses `np.interp`, which silently holds endpoint values if a supplied
   trajectory does not cover the scoring interval. Reject nonfinite trajectories
   and insufficient time coverage in a reusable evaluator. The frozen candidate
   covers the complete interval, so this does not invalidate these results.
2. The primary gap mask rejects long interpolation intervals but does not reject
   every delayed receipt batch. The stricter independent check above verifies
   that this omission does not drive the winner's gain.
3. “Final error” means the final valid sample. Include that sample's time if the
   last scoring point is excluded, instead of suggesting an unobserved endpoint.
4. The corrected wheel lateral velocity is `vx=-b*wz`, so wheel vx and yaw-rate
   measurement errors are correlated. A diagonal measurement covariance does
   not represent that correlation. The fitted variances are effective fusion
   trust settings, not statistically calibrated noise variances or evidence of
   consistent uncertainty. Future covariance propagation should include the
   cross term and be validated separately.
5. Numerical precision in fitted covariance values does not imply uniquely
   identifiable physical noise. Leave-one-out fits have substantially different
   yaw-rate covariance scales while similar geometry remains effective. Rounded
   stable values are appropriate; do not claim a global or unique optimum.
6. Three recordings from one session do not establish generalization to new
   surfaces, loads, routes, or IMU acquisition firmware. Keep the measured
   centimeter-level residual and these data limits explicit; perfect alignment
   and bounded long-term dead reckoning have not been demonstrated.
