# Independent sensor and kinematic audit

These experiments use all three original MCAP recordings. Vicon is a calibration
and scoring reference only; it is never supplied as an estimator measurement.
Results below are sensor diagnostics and direct dead reckoning, **not EKF results**.

## Findings

1. **The IMU contains approximately 3 Hz new information, despite publishing at
   approximately 65 Hz.** Complete nine-component orientation/gyro/acceleration
   payloads repeat exactly in 95.46–95.48% of consecutive messages. New complete
   payloads arrive every 0.337 s. This persists during turns, ruling out stationary
   gyro quantization as the explanation. There are only 144, 255, and 145 distinct
   consecutive payloads in the three recordings, respectively. See
   `imu_freshness.json` and `imu_payload_refresh.png`.

2. The first occurrence of each changed gyro payload fits Vicon's yaw rate with
   **0.198–0.206 s apparent delay**, a multiplicative scale of 0.958–0.972, and
   0.015–0.019 rad/s residual RMSE. Treating the repeated values as independent
   high-rate measurements adds approximately half a refresh interval of mean age:
   the whole published stream appears about 0.35–0.38 s late. The bridge assigns
   a fresh ROS stamp to every packet and discards `t_tx_ns`. The repository does
   not include MCU acquisition code, so the upstream cause of the repeated
   payloads cannot be uniquely identified here. Suppressing duplicates does not
   recover the missing high-rate acquisition information.

3. **The recorded quaternion yaw sign is correct.** Fitting relative Vicon yaw
   against relative IMU quaternion yaw gives positive slopes 1.014, 1.028, and
   1.031. The existing trial comment describing reversed recorded quaternion yaw
   is inconsistent with these bags. Orientation still has delay and calibration
   problems; a correct sign alone does not justify strong orientation fusion.

4. **Wheel yaw scale is consistently high by approximately 22%.** Corrected wheel
   yaw rate is approximately 0.82 times the recorded rate. Mean wheel speed needs
   approximately 0.955–0.96 scaling. These are effective measured calibrations;
   tire contact, wheel geometry, encoder conversion, and reference placement can
   all contribute. They should not be described as direct physical dimension
   measurements.

5. **Wheel body twist has a rotation-order error.** The implementation derives
   world velocity using the pre-update yaw, updates yaw, then rotates that world
   velocity into the body using the post-update yaw. The exact inverse applied
   to recorded data is:

   ```text
   delta = recorded_yaw_rate * wheel_header_dt
   mean_speed = sin(delta) * recorded_vx + cos(delta) * recorded_vy
   corrected_vy = linear_scale * mean_speed
   corrected_yaw_rate = yaw_scale * recorded_yaw_rate
   corrected_vx = -lever_arm * corrected_yaw_rate
   ```

   The last two body-velocity equations assume the configured pi/2 base yaw
   offset. Production code should calculate body twist directly for arbitrary
   offsets before integrating pose. Offline compensation must be explicit;
   changing live source code does not change existing recorded wheel messages.

6. **The 0.45 m odometry lever arm is inconsistent with the best effective model.**
   A zero-delay fit prefers approximately 0.26–0.28 m. Allowing a causal 0.15 s
   delay of wheel values prefers approximately 0.29–0.30 m, close to the existing
   controller value 0.301 m. Lever arm and timing partly trade off; covariance
   changes cannot repair either systematic error. A latency number fitted to a
   reference is not by itself a validated acquisition timestamp correction.

7. Vicon positions/quaternions have **no rejected norm, height, tilt, or >8 cm
   jump samples** under the repository's existing physical limits. There are
   network receipt bursts and gaps, including approximately 0.64 s maximum raw
   pose header age in the last bag. Source stamps are monotonic but belong to a
   different clock: median Vicon receipt-minus-header age is about -4 ms, -4 ms,
   and +2 ms across the runs. Negative ages preclude assuming synchronized clocks.
   Receipt-time differentiation without smoothing creates spurious >2 m/s
   instantaneous velocities. The regular-grid diagnostic masks remove gaps and
   delayed receipt batches and erode around their edges. Raw Vicon `/pose` and
   `/odom` yaw agree within approximately 0.0015 rad at zero relative lag, so the
   principal lag findings are not an `/odom`-only processing artifact.

## Shared geometry experiment

`fit_geometry.py` and `simplify_geometry.py` independently integrate corrected
body velocities on a 100 Hz grid. All scoring uses one initial position/yaw
alignment and equal per-bag weights. No whole-trajectory SE(2) fitting, per-bag
scale, or reference feedback is used. Least squares fits XY residuals plus yaw
residuals with a 1 m/rad weighting. Invalid reference times are excluded from
scoring but estimator state is integrated continuously through them.

The simple shared parameters are:

```text
linear_velocity_scale = 0.955389
yaw_rate_scale = 0.820400
lever_arm = 0.266844 m
left/right asymmetry = 0
wheel time shift = 0
```

| Bag suffix | Shared-fit XY RMSE | Leave-this-bag-out XY RMSE |
|---|---:|---:|
| 184054 | 0.0474 m | 0.0533 m |
| 190842 | 0.1063 m | 0.1104 m |
| 193228 | 0.0554 m | 0.0627 m |

Each leave-one-out result re-fits the three geometry parameters on the other two
bags and evaluates the untouched bag. A rounded common configuration
`linear=0.955`, `yaw=0.821`, `arm=0.26 m` achieves 0.0454/0.1075/0.0555 m.
Small left/right asymmetry adds little reliable benefit, so neutral asymmetry is
preferable. All recordings come from the same session and similar motion;
generalization to different surfaces, loads, temperatures, and routes is untested.

The noncausal `gyro_gyro_advance` experiment uses future gyro measurements solely
to diagnose lag. It must not be presented as achievable real-time performance.
The primary shared geometry results above use no future measurements or sensor
time shifts. The diagnostic direct integrator uses trapezoidal numerical
integration and interpolated sampled streams; production EKF results must be
validated separately using the actual timestamped measurement pipeline.

## Artifacts and rerun

- `audit.py`: extract all source streams, timestamp/frame/quality audit, derivative
  scale/lag diagnostics; writes `hamr_*.npz`, `hamr_*_grid.npz`, per-bag JSON.
- `imu_freshness.py`: independent payload update count and fresh-sample delay fit.
- `fit_geometry.py`: joint geometry fits and leave-one-bag-out evaluations.
- `simplify_geometry.py`: neutral-asymmetry fits and rounded candidate checks.
- `geometry_fits.json`, `simplified_geometry.json`: complete numeric results.
- `geometry_paths.png`, `imu_payload_refresh.png`: static diagnostic figures.

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 rosbags/ekf_tuning/sensor_audit/audit.py
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 rosbags/ekf_tuning/sensor_audit/imu_freshness.py
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 rosbags/ekf_tuning/sensor_audit/fit_geometry.py
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 rosbags/ekf_tuning/sensor_audit/simplify_geometry.py
```

Residual fits include Vicon derivative smoothing, sensor delay, slip, and frame
effects. They are not independent estimates of measurement-noise covariance R.
