# HAMR corner tracking investigation — 20 September 2026

Commands in this guide assume a checkout of this repository. From any directory
inside that checkout, set these portable paths before running the examples:

```bash
export HAMR_REPO="$(git rev-parse --show-toplevel)"
export HAMR_RUNS="${HAMR_RUNS:-$HOME/Videos/hamr_sim}"
```

Recorded videos and full run reports are external artifacts, not files supplied
by a Git clone. Paths under `${HAMR_RUNS}` identify retained runs or new outputs
created by the recording commands.

The subsequent implementation and validated replay are documented in the
[smooth waypoint guide](../SMOOTH_WAYPOINT.md) and
[implementation validation](../SMOOTH_VALIDATION.md). This report preserves the
earlier investigation of the original abrupt trajectory.

**The dominant problem is an infeasible corner reference interacting with command limits and finite drive response. The implemented Jacobian is algebraically correct for the offset point it controls.** The hardware has an additional, measured wheel-speed tracking problem that makes recovery last considerably longer than in the current simulation.

This conclusion combines an independent derivation and numerical audit of the actual code, the existing Gazebo recording, five new full-vehicle Gazebo experiments, and three recorded hardware runs from 16 September. The experiments use the full COMPA chassis retrofit with the two CAD ball casters. No hardware was actuated. Production controller gains, waypoint generation, and normal simulation settings were not changed.

**What happens at a corner**

The active `waypoint_traj_simple.py` reference goes through:

```
(0,0) → (0,2) → (-2,2) → (-2,4.5) → (0,4.5) → (0,2) → (0,0)
```

It travels at 0.25 m/s along each straight segment and changes direction immediately at the waypoint. At the first corner the requested velocity changes from `(0,+0.25)` to `(-0.25,0)` m/s. The magnitude of this instantaneous velocity change is 0.354 m/s, although speed remains 0.25 m/s. Position is continuous; velocity is not. Exact tracking would require unbounded acceleration at that instant.

The reference keeps advancing according to elapsed time even while the car is still removing its previous velocity. Consequently:

1. The car continues a short distance beyond the corner in its incoming direction.
2. The reference has already moved down the next segment, so a second error develops along that segment.
3. Position and velocity feedback request additional corrective motion.
4. Wheel limits or insufficient wheel-speed response delay that correction; eventually the car catches up.

This is the observed spike-and-recovery pattern. Holonomic velocity control does not remove inertia, acceleration limits, friction limits, or motor bandwidth. A physically exact sharp corner requires the tracked point to reach zero speed at the corner. Continuing at nonzero speed requires rounding the corner and accepting a specified geometric deviation. ROS's own trajectory documentation explicitly discourages position-only linear interpolation because it produces discontinuous waypoint velocities; its cubic and quintic alternatives provide progressively stronger derivative continuity. [ROS 2 trajectory representation](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)

**Is the Jacobian wrong?**

For wheel radius `r`, half-track `a`, forward offset `b` from the wheel axle to the tracked point, chassis heading `θ`, and wheel angular velocities `ωR,ωL`, the underlying planar equations are below. Here `ψ` is world turret yaw and `ωturret` is turret yaw rate relative to the chassis.

```
v         = r (ωR + ωL) / 2
θdot      = r (ωR - ωL) / (2a)
xdot      = v cos(θ) - b θdot sin(θ)
ydot      = v sin(θ) + b θdot cos(θ)
ψdot      = θdot + ωturret
```

Expanding these equations gives exactly the matrix in the active hardware controller, whose columns are ordered **right wheel, left wheel, turret**. The new simulation helper returns **left wheel, right wheel**, but its calling code uses that ordering correctly. Its inverse is algebraically equivalent.

The audit extracted the actual methods from the hardware controller, legacy COMPA controller, and new simulation adapter and checked them against an independently written forward model over 10,000 random parameter/heading/velocity cases. The largest reconstruction residual was `2.67e-15`, consistent with floating-point rounding. The XY Jacobian condition number is only 1.220 for the simulation geometry and 1.163 for the hardware configuration, independent of heading. There is no 90-degree corner singularity.

The offset `b` matters: the differential axle itself cannot move sideways without slip, but a point ahead of that axle can have a lateral velocity as the chassis rotates. This is a familiar abstraction for differential-drive control, with control authority and maneuverability tradeoffs. [Offset-point differential-drive research](https://arxiv.org/abs/1802.07199)

This validates the algebra and code ordering, **not the physical calibration** of wheel radius, wheel separation, Vicon tracking-point location, tire compression, or wheel slip. Those can still introduce secondary error. The current hardware configuration uses `r=.122`, `a=.350`, `b=.301` m; the new simulation uses `.1075`, `.33072`, `.27114` m, matching its nominal model. The simulation and hardware also use different controllers and actuator models, so identical-looking corners do not imply identical dynamics.

With the hardware turret disabled, the offset point remains controllable in XY, but independent platform world-yaw control is unavailable. That is a separate limitation and does not invalidate the XY Jacobian.

**Evidence from the original simulation recording**

The file analyzed is `${HAMR_RUNS}/simple_waypoint_20260920_185228/trajectory.json`, the feedback log accompanying the MP4.

| Event | Peak timed XY error | Geometric overrun in incoming direction |
|---|---:|---:|
| First 90° corner, 8 s | 62.9 mm | 48.7 mm |
| Second 90° corner, 16 s | 64.0 mm | 50.5 mm |
| Third 90° corner, 26 s | 59.4 mm | 44.1 mm |
| Fourth 90° corner, 34 s | 59.3 mm | 46.8 mm |
| Collinear waypoint, 44 s | 2.9 mm | Not a turn |

The corner peaks occur approximately 0.34 seconds after the direction switch. Settled straight-segment RMS error is 1.58 mm, excluding startup and 2.5 seconds of recovery after each true corner. The 44-second waypoint is a useful internal comparison: the segment index changes but the velocity direction does not, and there is no comparable spike. This implicates the velocity transition, rather than waypoint numbering or a general coordinate-frame error.

The new simulation adapter contains a **15 rad/s² per-wheel command ramp that I added when setting up the recording pipeline**. With its geometry, the first corner nominally demands one wheel change from approximately `+2.326` to `−2.837 rad/s`: a 5.162 rad/s jump. At that ramp rate the frozen-heading transition would take about 0.344 seconds. Actual heading and feedback evolve during the transition, but the scale matches the observed transient. The two wheels need very different changes; independently ramping them changes their ratio and therefore the commanded motion direction during the transition.

A separate planar experiment with correct kinematics, no caster, no slip, and no motor dynamics produces 54.8 mm peak tracking error when this ramp is applied. With unrestricted wheel response it produces only 1.0 mm at a 20 ms controller interval. This establishes that the ramp and abrupt reference alone can create a substantial spike without any CAD error.

See [the original recording diagnosis](recorded_run_corner_diagnosis.png), [metrics](recorded_run_metrics.json), and [independent kinematics experiments](kinematics_results.json).

**Controlled full-vehicle Gazebo experiments**

Each experiment starts a fresh model and uses actual Gazebo pose and joint feedback. The caster model, contact settings, vehicle geometry, position gain, and 6 rad/s wheel-speed limit are held constant. Every run completed the route and final hold.

| Experiment | Motion duration | Timed XY RMS | Maximum timed XY error | Maximum distance from geometric route |
|---|---:|---:|---:|---:|
| Baseline: 0.25 m/s, original wheel ramp | 52 s | 13.64 mm | 66.50 mm | 53.67 mm |
| Effectively remove wheel ramp | 52 s | 7.27 mm | 37.36 mm | 28.86 mm |
| Half speed, retain original ramp | 104 s | 2.77 mm | 19.20 mm | 14.93 mm |
| Smooth stops, same points, retain original ramp | 97.5 s | 1.80 mm | 8.24 mm | 6.86 mm |
| No ramp, halve physics timestep | 52 s | 6.75 mm | 36.09 mm | 27.70 mm |

These are single controlled runs, not a statistical uncertainty study. Small differences between the fresh baseline and original MP4 run are expected from asynchronous update timing and contact simulation.

Removing the wheel ramp reduces the maximum error by 44%, but does not eliminate it. That reduction is a counterfactual result, not an additive decomposition of error into independent percentages. Joint-speed feedback tracks the previously published wheel command essentially exactly at the corners, so the remaining error is not simulated motor lag. Time-aligned wheel surface velocities and rigid-body wheel-point velocities differ during the abrupt reversal, consistent with transient slip/contact dynamics and finite chassis inertia. Halving the physics timestep changes the residual peak from 37.36 to 36.09 mm; this is evidence against a coarse-timestep artifact being the main cause, not a complete solver-convergence proof.

The model uses Gazebo's default velocity-command JointController, not a calibrated motor/PWM/current model. Gazebo documents force-command control as a separate optional mode. The current simulation therefore cannot establish real motor torque margin or reproduce firmware PI behavior. [Gazebo 8 JointController](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointController.html)

The smooth-stop experiment changes **only the experimental reference timing**. Each segment uses `s(u)=10u³−15u⁴+6u⁵`, with zero speed and acceleration at both ends. Segment duration is `1.875 × length / .25`, so peak translational speed remains 0.25 m/s. It preserves every waypoint and stops at all of them, including the collinear waypoint. Its four true-corner local peak errors are only 3.17, 0.54, 1.45, and 0.79 mm; its global 8.24 mm maximum occurs elsewhere. The experiment reduces the global peak by approximately 88% while preserving the original wheel ramp and caster physics. It is an illustrative smooth trajectory that tracked well in this simulation, not an optimized traversal-time solution or a demonstrated hardware result.

Measured wheel-command acceleration stays below 4.44 rad/s² and wheel speed below 2.70 rad/s in that experiment; both remain below the unchanged limits. Raw and published wheel commands match throughout, confirming that the smooth reference avoids activating the wheel command limits.

The evidence does not support blaming the hemisphere orientation or replacing the caster geometry as the first fix. It also does not validate the CAD caster's physical mass, friction, compliance, or exact hardware fidelity. Those remain calibration questions.

**What the real hardware recordings show**

Three MCAP bags were analyzed: `hamr_hw_20260916_184054`, `hamr_hw_20260916_190842`, and `hamr_hw_20260916_193228`. All contain the 0.25 m/s discontinuous reference and useful wheel-controller telemetry. Historical flashed firmware and launch overrides are not proven identical to today's source files; direct telemetry findings are stronger than assuming that identity.

Replaying the recorded reference, recorded P/D/I contributions, and Vicon heading through the current Jacobian geometry and common 28 RPM pair scaling reconstructs the 19:08:42 wheel commands with **0.00277 rad/s RMS discrepancy**. Independently clipping each wheel instead gives 0.197 rad/s discrepancy. This strongly supports the identified command-generation mechanism in the historical run, while still not certifying the physical geometry. See [the reconstruction results](hardware_response_reconstruction.json).

The 19:32:28 recording gives particularly clear evidence:

- The first corner overruns the outgoing line by about **67.8 mm**; the third and fourth overrun by 62.0 and 52.2 mm. At the second scheduled turn the car is already behind and does not overrun the corner line, although its old-direction velocity still takes time to decay. A tracking-error spike and geometric overshoot are not interchangeable.
- The old-direction velocity takes approximately **0.29–0.32 s** to fall below 10% of the incoming speed. The robot travels an additional roughly **49–60 mm** in that old direction after the switch.
- The larger, time-aligned first-corner tracking error reaches approximately **0.256 m**, around **3.52 s** later. This includes falling behind the moving reference and is not a 256 mm geometric corner overshoot.
- At least one firmware wheel target is at **28 RPM for about 99% of the first four seconds after each turn**. Measured speed on the capped wheel is commonly only **23–25 RPM**.
- Actual final motor output reaches **4095 PWM** briefly. Sustained target saturation is much more extensive than final PWM saturation; the entire recovery must not be described as four seconds at maximum motor output.
- Telemetry shows a feedforward contribution clamped at **3500 PWM**. Its observed slopes are approximately **175 PWM/RPM** for positive commands and **160 PWM/RPM** for negative commands, reaching that clamp at 20 and 21.875 RPM respectively. The PI controller must supply the rest.
- The recorded PI contribution is consistent with `Kp=60 PWM/RPM` and `Ki=20 PWM/(RPM·s)`. During the first turn the faster wheel begins with a negative accumulated integral from earlier overspeed. That state takes time to unwind while the wheel is now too slow. This is direct evidence of an inner-loop recovery limitation, not merely a hypothetical motor delay.

There is also a velocity-limit mismatch calculable from the current hardware geometry. The maximum wheel demand over heading relative to the desired travel direction is:

```
max |ωwheel| = (speed / r) sqrt(1 + (a/b)²)
```

At 0.25 m/s this is **3.143 rad/s = 30.01 RPM**, exceeding the configured 28 RPM cap even with perfect motors and zero tracking error. The worst heading difference is about 49°. The corresponding all-direction nominal speed ceiling is **0.233 m/s**, before reserving any capacity for feedback correction. Thus the incoming and outgoing straight segments may be feasible, yet the intermediate chassis orientations during a turn are not feasible at the scheduled speed.

Today's ROS hardware controller scales both wheel targets together when the cap is exceeded. That preserves their instantaneous ratio, but slows progress while the time-based reference continues unchanged. This contributes to the prolonged lag. It differs from the new simulation adapter's per-wheel acceleration ramp.

Current hardware XY gains are `P=.7`, `D=.4`, `I=0`. The derivative term uses filtered **reference velocity minus measured world velocity**. The velocity switch therefore causes a finite extra correction of about 0.1 m/s per affected axis; comparable 0.094–0.109 m/s corrections appear in the recordings. This increases demand when the wheel loop already has little headroom. It is not an infinite derivative impulse from a position step, and the current outer XY loop cannot have integral windup because its integral gains are zero. The firmware wheel PI loop is a separate loop and does have accumulated state.

Normal hardware launch closes this controller on Vicon `/HAMR_base/odom`. Wheel odometry/EKF calibration differences and IMU update rates should not be blamed for these particular transients without demonstrating that a different localization input was selected. Occasional Vicon faults in the logs are separate events; they do not explain the repeatable corner response and wheel target-versus-measurement mismatch.

The recordings do not contain enough independent motor voltage/current, torque, contact-force, or battery information to determine how much of the loaded wheel under-response comes from electrical headroom, friction, mechanical load, motor characteristics, or imperfect PI/feedforward calibration. A 28 RPM target is not proof that 28 RPM is physically attainable under every load, and the clipped feedforward map is not proof that it is physically unattainable.

**What should change, in order**

1. **Make the reference dynamically feasible.** For exact waypoint corners, plan deceleration to zero before the corner and smooth acceleration afterward. For continuous motion, specify a corner radius/tolerance and use a smooth blend. Bound wheel speed, wheel acceleration, and preferably jerk over the entire changing chassis heading. The quintic simulation experiment demonstrates the value of doing this before tuning gains. Time scaling must respect actuator constraints, as described in [Modern Robotics, trajectory generation](https://modernrobotics.northwestern.edu/chapters/chapter9/).
2. **Make trajectory progress respect available drive authority.** Reserve speed margin for correction; 0.233 m/s is a nominal upper bound, not a sensible hardware cruising recommendation. A conservative lower-speed trial and a reference governor/retiming method should precede another 0.25 m/s sharp-corner test. Avoid solving an unrestricted reference and only clipping commands afterward while its clock runs ahead.
3. **Characterize and tune the hardware wheel loop under load.** Log requested RPM, firmware target, measured RPM, feedforward, PI terms, integral state, output, and voltage/current where available. Measure positive/negative step responses and loaded steady speed for both wheels. Check feedforward calibration, output headroom, reversal behavior, and saturation-aware PI recovery. Do not simply raise the RPM cap or stack a global feedforward multiplier on an uncharacterized inner loop.
4. **Then tune the outer XY controller.** Evaluate P/D with the feasible reference and a verified wheel response. Increasing gains cannot make an infeasible command realizable and can extend saturation. Preserve the useful velocity-error feedback while reducing unnecessary corner excitation through planning.
5. **Calibrate the physical model and frames with separate tests.** Verify the actual controlled-point offset, loaded wheel radii, track, heading offset, motor signs, command delay, and ground response on straight and constant-curvature runs in both directions. Add identified motor dynamics and measured contact parameters to Gazebo before using it to predict hardware error magnitudes.

The recommendation is to retain the Jacobian's mathematical form. Its inputs and physical parameters should be validated, but changing matrix signs or inserting arbitrary scale factors would obscure the measured constraints rather than solve the corner demand.

**Files and reproduction**

- [Kinematics derivation and source audit](kinematics_review.md)
- [Hardware audit, telemetry interpretation, and code references](hardware_audit.md)
- [Full simulation ablation report](simulation_ablation_report.md)
- [Simulation comparison plot](gazebo_ablation_comparison.png)
- [Original recording corner plot](recorded_run_corner_diagnosis.png)
- [Hardware metrics](hardware_corner_metrics.json), [simulation metrics](experiments/metrics.json), and [analytical results](kinematics_results.json)

The companion reports document their additional scripts and provenance. To reproduce the original-recording analysis and independent algebra experiments without ROS nodes:

```bash
cd "${HAMR_REPO}"
/usr/bin/python3 hamr_ball_caster/docs/corner_analysis/analyze_recorded_run.py \
  "${HAMR_RUNS}/simple_waypoint_20260920_185228/trajectory.json"
/usr/bin/python3 hamr_ball_caster/docs/corner_analysis/kinematics_experiment.py
```

To rerun the isolated Gazebo experiments, using the previously built workspace:

```bash
source "${HAMR_REPO}/hamr_ball_caster/scripts/env.sh"
/usr/bin/python3 "${HAMR_REPO}/hamr_ball_caster/docs/corner_analysis/experiments/run_ablations.py" \
  baseline no_slew half_speed quintic_stop no_slew_half_step
/usr/bin/python3 "${HAMR_REPO}/hamr_ball_caster/docs/corner_analysis/experiments/analyze_ablations.py"
/usr/bin/python3 "${HAMR_REPO}/hamr_ball_caster/docs/corner_analysis/experiments/wheel_body_residual.py"
/usr/bin/python3 "${HAMR_REPO}/hamr_ball_caster/docs/corner_analysis/experiments/plot_ablations.py"
```

These use localhost discovery, separate ROS domains 70–74, and unique Gazebo partitions. Rerunning overwrites the corresponding experiment result files. The smooth reference is confined to the experimental runner; it has not been deployed to the hardware controller or the normal MP4 pipeline.
