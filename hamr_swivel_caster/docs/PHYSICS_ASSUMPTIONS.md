# Caster physics and sensitivity assumptions

This is a rigid-body engineering estimate for POWERTEC 17050 / Amazon B07B7KJ5QZ. The manufacturer gives approximately 160 mm wheel diameter, 50 mm width and 192 mm mounting height; its rounded metric and imperial dimensions are not fully consistent. Trail, component masses, inertia tensors, bearing resistance and rubber contact properties have not been measured. Simulated caster lag or tracking error therefore cannot establish a hardware defect. [Manufacturer dimensions](https://powertecproducts.com/17050-swivel-heavy-duty-industrial-caster-6-5-16/)

The nominal caster uses 45 mm trail and 2.64 kg allocated as 0.70 kg stationary plate/bearing, 0.59 kg rotating fork/axle and 1.35 kg wheel/hub. A separate 0.18 kg adapter attaches it to each rocker. The 2.64 kg figure is a listing estimate that may include packaging. Wheel inertia is a solid cylinder; fork inertia is an envelope box. Wheel spin inertia is 0.00432 kg m²; fork plus wheel inertia about the swivel axis is approximately 0.00594 kg m² by the parallel-axis theorem. These are derived model values. The inherited COMPA chassis and drive inertias are also uncalibrated.

## Parameters that reach Harmonic / DART

The installed runtime contains Gazebo Physics 7.8.0 and DART 6.13.2. Its SDF loader copies joint damping and friction into DART properties, and maps collision `mu` / `mu2` to the DART contact material even though these SDF elements are under `surface/friction/ode`. [Gazebo Physics 7.8.0 loader](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/dartsim/src/SDFFeatures.cc)

Viscous joint damping opposes joint velocity; DART integrates its damping contribution implicitly. Joint Coulomb friction is also implemented: DART creates an explicit friction constraint when the joint's configured friction is nonzero. Here its units are N m, not a dimensionless tire coefficient. Keeping its nominal value at zero is a modeling choice, not evidence that Harmonic ignores it. [DART damping](https://github.com/dartsim/dart/blob/v6.13.2/dart/dynamics/detail/GenericJoint.hpp), [DART friction constraint creation](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ConstraintSolver.cpp), [DART Coulomb constraint](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/JointCoulombFrictionConstraint.cpp)

DART combines the two contacting surfaces using the **minimum** of their primary and secondary coefficients. With tire `mu=0.9` and ground `mu=0.8`, effective caster friction is 0.8. Changing only the caster tire to 0.3 isolates caster contact; also lowering ground friction changes rear-drive traction. Record both values when interpreting a run. [DART contact material combination](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ContactSurface.cpp)

An integration check found that the earlier explicitly named URDF tire collision lost its link-level friction extension during conversion. The tire collision is now unnamed, and the test verifies both converted SDF coefficients. Earlier snapshots may have used the parser's default coefficient; a lower ground coefficient can mask that difference. Joint friction was retained correctly. This conversion issue does not establish that every earlier trajectory changed.

The rear wheels retain the existing `JointController` velocity mode. It sends DART servo velocity commands, and the inherited drive joints specify no finite effort limits. This models ideal velocity tracking with contact interaction, not measured motor torque, current limits or gearbox efficiency. Force-mode PID and measured effort limits would be needed to evaluate actuator feasibility. The casters remain completely unactuated. [Harmonic JointController modes](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointController.html), [Gazebo DART servo commands](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/dartsim/src/JointFeatures.cc)

Knobby tread is visual only. Contact uses rigid cylinders, with no tire deformation, bearing backlash, roughness, tread impacts or measured static/dynamic friction curve. Wheel joint damping is a crude bearing/rolling-loss proxy. Static SDF checks prove parameter transport; dynamic sweeps measure the response of this approximate model.

## Reproducible overrides

`config/physical_properties.yaml` remains the nominal source. Pass a complete alternate file to either vehicle or standalone Xacro using `caster_physical_config:=/absolute/parameters.yaml`. Save that file and expanded URDF with each run. Numerical overrides take precedence over YAML; `default` reads YAML.

| Xacro argument | Default | Meaning |
|---|---:|---|
| `caster_mass_scale` | 1 | All caster component masses and corresponding inertias; excludes adapter |
| `caster_wheel_mass_scale`, `caster_fork_mass_scale` | 1 | Additional component mass and inertia multipliers |
| `caster_wheel_inertia_scale`, `caster_fork_inertia_scale` | 1 | Additional COM tensor multiplier, mass unchanged |
| `swivel_damping` | `default` → 0.015 | N m s/rad |
| `swivel_friction` | `default` → 0 | Coulomb torque, N m |
| `wheel_damping` | `default` → 0.006 | N m s/rad |
| `wheel_friction` | `default` → 0 | Coulomb torque, N m |
| `tire_mu` | `default` → 0.9 | Both tire contact coefficients |
| `caster_trail` | `default` → 0.045 | Axis-to-axle offset, m; also updates fork geometry/inertia |

Mass/inertia factors and trail must be finite and positive; resistance and contact coefficients must be finite and nonnegative. Changing trail or overall height requires checking clearance and camera framing again. Inertia scaling preserves tensor positivity but does not prove a mass distribution fits the real part.

## Suggested estimated sensitivity values

These are exploratory bounds, not published properties or confidence intervals. Vary one factor at a time on the same route before combining adverse values.

| Quantity | Nominal | Suggested exploratory values |
|---|---:|---|
| Swivel viscous damping | 0.015 N m s/rad | 0.005, 0.05, 0.15 |
| Swivel Coulomb torque | 0 N m | 0.02, 0.10 |
| Wheel viscous damping | 0.006 N m s/rad | 0.002, 0.018 |
| Caster tire friction | 0.9 | 0.3, 0.1; hold ground fixed to isolate casters |
| Wheel/fork mass factors | 1 | 0.75, 1.25 |
| Wheel/fork inertia factors | 1 | 0.5, 1.5; 3 is an exaggerated stress case |
| Swivel trail | 45 mm | 30, 60 mm; recheck fit |

At 1 rad/s, nominal swivel damping contributes 0.015 N m versus 0.15 N m for the upper damping stress case. Actual caster alignment also depends on normal load, trail, tire slip, suspension movement and commanded curvature. Compare startup separately from moving alignment, include reversals and lateral direction changes, and use both local caster alignment and vehicle trajectory errors. First hardware measurements should be trail, wheel and fork masses, free-spin decay, swivel breakaway torque under load, and floor-specific traction.
