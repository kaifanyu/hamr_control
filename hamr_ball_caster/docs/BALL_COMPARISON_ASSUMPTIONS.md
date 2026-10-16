# Passive ball-caster comparison assumptions

The comparison reuses the existing COMPA chassis, rocker suspension, rear drives,
controller and recovered CAD ball-caster mechanism. Each front caster has five
passive continuous joints: one carrier, two independently rotating hemispheres
and two polar rollers. There is no powered ball joint or vertical swivel joint.
Carrier phase is an internal assembly angle, not a caster heading. A conventional
swivel-axis alignment error therefore has no direct ball-caster equivalent.

The original ball model is preserved. The recording wrapper adds only a fixed,
collision-free 1 g camera to the right rocker; disabling the camera reproduces
the original expanded model. Initial world yaw changes the spawn pose, not the
caster geometry or constraints. No model fitting to the swivel results is used.

| Quantity | Passive CAD ball caster | Ordinary swivel caster |
|---|---|---|
| Rolling geometry | Separate faceted hemisphere and polar-roller meshes | Rigid 80 mm-radius, 50 mm-wide cylinder |
| Nominal outer radius | 100 mm analytic shell; rolling mesh envelope reaches 101.799 mm | 80 mm |
| Ground contact placement | Original ball center `(0.315, ±0.072, -0.15262)` m relative to its rocker | Swivel axis at rocker `x=0.315`, `y=±0.072` m; estimated 45 mm trail |
| Front assembly mass, each | 5.33808 kg | 2.64 kg caster plus 0.18 kg adapter |
| Inertia basis | Full CAD mesh COM tensors and assumed material densities | Estimated primitive COM tensors |
| Viscous damping | Carrier and each hemisphere: 0.0002 N m s/rad; each polar roller: 0.00005 | Swivel: 0.015; wheel: 0.006 N m s/rad |
| Nominal joint Coulomb torque | 0 N m | 0 N m |
| Nominal contact coefficient | Intended 1.0; converted model falls back to 1.0 | Explicit converted tire coefficient 0.9 |
| Ground coefficient for nominal comparison | 0.8 | 0.8 |
| DART combined nominal coefficient | 0.8 | 0.8 |

The original ball shell bottom is nominally 0.92 mm below the rear-wheel bottom
at zero rocker pitch: `-0.15262 - 0.100 = -0.25262` m versus
`-0.1442 - 0.1075 = -0.25170` m relative to a rocker. The suspension settles;
faceting, gaps and polar-roller protrusion make support height phase-dependent.
This is not an exact spherical contact proxy. See the existing
[CAD findings](CAD_FINDINGS.md) and
[geometry provenance](../assets/provenance/geometry_notes.md).

The ball fork, carrier, positive hemisphere, negative hemisphere and two polar
rollers weigh 0.81535, 0.38698, 2.07094, 2.03836 and 2 × 0.01322 kg respectively.
Their masses and COM inertia tensors come from closed display meshes, assuming
1200 kg/m³ polymer, 2700 kg/m³ aluminum and 7850 kg/m³ steel as assigned in
[physical_properties.yaml](../config/physical_properties.yaml). The source CAD
does not establish actual material densities. Hemisphere COM tensor diagonals
are approximately `(0.00749, 0.00493, 0.00562)` kg m², with significant `Ixz`
terms near `-0.00133` kg m²; these are not scalar wheel-spin inertias. Complete
tensors and joint axes are retained in the [static audit](../assets/provenance/ball_comparison_model_audit.json).
The two ball assemblies make the comparison vehicle about 5.036 kg heavier than
the nominal swivel vehicle. Thus this is a comparison of the two existing model
assemblies, not a mass-matched isolation of joint topology.

## URDF to SDF audit

The saved [converted SDF](../assets/provenance/compa_ball_caster_recording.sdf)
was generated with `gz sdf -p` from the nominal recording wrapper. It retains
all ten passive rolling joints and their damping/friction values. The camera is
correctly fixed-lumped into `right_rocker_link`, preserving its relative pose
and sensor settings. Rear-drive and turret plugins are unchanged; no plugin
commands a caster joint.

The original named mesh collisions lose their link-level URDF `mu1`/`mu2`
extensions during conversion: the serialized ball collision SDF has no explicit
coefficient. The installed SDFormat 14 schema defaults `mu` and `mu2` to 1.0;
the Gazebo DART loader reads those defaults. This happens to match the nominal
intention, so the ball baseline is preserved. Changing only the original
per-mesh URDF `mu` tags would not constitute a reliable friction experiment.
The audit records every affected collision and the installed schema path.
[Gazebo Physics loader](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/dartsim/src/SDFFeatures.cc)

DART uses the minimum coefficient of the contacting materials. With nominal
ground coefficient 0.8, both front assemblies therefore use effective 0.8,
despite their different tire/mesh coefficients. A shared low-friction floor
also changes rear-drive traction and is a different experiment from changing
only the swivel tire. [DART contact material combination](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ContactSurface.cpp)

Neither model has calibrated compliance, contact roughness, bearing drag or
actuator torque/current limits. The same inherited rear-wheel velocity servos
and controller limits apply to both. Large tracking errors can reflect drive
command limits, geometry, mass, contact discretization and startup as well as
caster mechanics; these runs cannot establish a physical hardware defect.
Swivel-only fork damping/inertia variants have no direct ball parameter mapping:
if their routes are repeated using the nominal ball model, label them as nominal
ball repeats rather than matched mechanical sensitivity cases.

## Recording interface

Expand `urdf/compa_ball_caster_recording.urdf.xacro`, then launch
`ball_caster.launch.py model:=compa controller:=false`
with `robot_description_file:=/absolute/robot.urdf` and
`initial_yaw:=<radians>`. Supported inherited model arguments are
`caster_mass_scale` (scales all ball masses and their COM tensors together) and
`carrier_phase`; nominal values are 1 and 0. The wrapper additionally accepts
`enable_closeup_camera`, default true.

The side camera publishes `/hamr/ball_caster/closeup/image` at 1280 × 720,
25 Hz. Its pose relative to the right rocker is
`xyz=(0.185,-0.55,0.01)`, `rpy=(0,0.312,1.3506)` radians, with horizontal
field of view 1 radian. The world overview camera publishes
`/hamr/recording_camera/image`. Each recorder starts its own image bridge;
launch does not add duplicate image bridges. Camera timestamp metadata, rather
than frame numbers alone, is the basis for synchronized comparison clips.
