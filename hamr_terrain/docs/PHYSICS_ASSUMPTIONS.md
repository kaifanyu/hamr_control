# Terrain geometry and physical assumptions

These worlds provide shared terrain geometry for the existing two COMPA caster
variants. They are estimated rigid-body tests, not reconstructed or calibrated
measurements of a real road. The launcher keeps each model's own mass, inertia,
bearing damping and contact geometry. See the
[ball-model assumptions](../../hamr_ball_caster/docs/BALL_COMPARISON_ASSUMPTIONS.md)
and [swivel-model assumptions](../../hamr_swivel_caster/docs/PHYSICS_ASSUMPTIONS.md).

## Geometry and grades

Each patch covers `x=[-3,3]`, `y=[-4,4]` m on a 25 mm grid. The top has 153,600
triangles; side walls and a bottom at Z=-0.25 m close the mesh, for 156,960
triangles total. The collision STL and visual Collada use the same generated
surface. Four flat outer aprons adjoin the patch; no flat collision plane
underlies the ruts or the patch interior. Lane markings are visual only.

The following values come from the generated height arrays. Slopes use
`atan(sqrt((dz/dx)^2+(dz/dy)^2))` with finite-difference derivatives at grid
nodes. They describe the top height field, excluding the vertical closing walls;
individual triangle-face normals can differ slightly.

| Terrain | Top-height range (mm) | Maximum anywhere on top | Maximum in driving corridor | Centerline longitudinal maximum |
|---|---:|---:|---:|---:|
| Road bumps | 0 to 40.00 | 11.70° | 11.70° | 11.70° |
| Gentle hill | 0 to 140.00 | 13.71° | 5.71° | 5.71° |
| Cross slope | 0 to 132.89 | 13.04° | 6.25° | 4.19° |
| Ruts / washboard | -27.95 to 5.99 | 11.06° | 11.06° | 5.25° |
| Fixed rock garden | 0 to 60.18 | 20.22° | 20.22° | 8.76° |

Here the **driving corridor is |x|≤0.6 m**, over the complete Y extent. The
centerline column uses only `|dz/dy|` at x=0. The hill's 13.71° global maximum
comes from its lateral return to the flat apron, not from the intended straight
climb. The cross-slope's nominal 3° describes its central lateral bank; entry,
exit and lateral edge transitions have different combined grades. The rut's
22 mm feature depth combines with the washboard to reach about 28 mm below
the approach datum. Rock overlap and undulation similarly raise the total
maximum above the largest individual 55 mm rock feature.

These heights and grades characterize this generated surface. They are not an
RMS road-roughness standard, a traversability guarantee, or a wheel-resolved
contact measurement. The flat south approach is sampled at Z=0 and supports
the default `(0,-2.8,0)` spawn facing +Y.

## Contact and solver

Worlds use Gazebo Harmonic's DART backend, ODE collision detection, the Dantzig
solver, 1 ms physics steps and gravity 9.80665 m/s². Terrain and apron SDF
declare `mu=mu2=1.0` and zero restitution. These are chosen simulation values,
not measurements of rubber, asphalt or soil.

Gazebo's physics system uses `AttachMeshShape` for a mesh collision. The DART
implementation creates a `DynamicsAspect` without parsing the SDF surface
fields on that path; the primitive-collision SDF path does parse those fields.
Thus a value present in a mesh collision's SDF is not by itself proof that the
attached mesh receives it. The nominal terrain uses 1.0 friction and zero
restitution to match the DART material defaults. Sliding-box probes are required
to verify actual effective contact response. Changing the terrain SDF coefficient
alone must not be presented as a validated friction sensitivity experiment.
[Gazebo mesh attachment](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/physics/Physics.cc),
[DART mesh implementation](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/dartsim/src/ShapeFeatures.cc),
[DART material defaults](https://github.com/dartsim/dart/blob/v6.13.2/dart/dynamics/detail/ShapeFrameAspect.hpp)

DART combines the two contacting surfaces using the minimum coefficient. Even
with one common terrain, the ball mesh and ordinary tire can therefore have
different effective friction: their nominal model values are 1.0 and 0.9
respectively. The previous flat comparison used a 0.8 floor that capped both;
these terrain worlds intentionally do not silently inherit that floor override.
This is a shared-geometry comparison, not a claim that every material parameter
matches. [DART contact combination](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ContactSurface.cpp),
[Gazebo DART SDF loader](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/dartsim/src/SDFFeatures.cc)

Ground, rocks and ruts are static rigid meshes. There are no moving pebbles,
plastic soil deformation, sinkage, vegetation, tire compliance or calibrated
rolling resistance. Mesh faceting and solver settings can affect small impacts
and support heights. A finer terrain grid or smaller timestep can change
contact details and needs its own validation.

## Vehicle and measurement limits

The ball model has five passive joints per caster; the ordinary caster has
passive swivel and rolling joints. Both retain the same inherited chassis,
suspension and rear-wheel velocity servos. Those servos do not have calibrated
motor torque/current limits; software rate or acceleration limits depend on
the controller running the experiment. Merely loading a terrain does not add
a traction controller, a trajectory controller or a motor model. A short
constant-velocity smoke test cannot establish sustained climb capability,
stall margin, efficiency or physical hardware superiority.

Guided drops check support at selected locations, and sliding probes check
contact response on the flat approach. A free rolling sphere does not identify
sliding friction. Actual Gazebo camera PNGs document rendered geometry;
appearance alone does not establish collision correctness. Runtime reports
must identify the exercised locations, model, source hashes and measured
checks. Full vehicle trajectories over all features remain a separate study.

Headless camera capture uses Ogre2 with Gazebo's EGL `--headless-rendering`
option; GUI rendering uses the same terrain assets. [Harmonic headless rendering](https://gazebosim.org/api/sim/8/headless_rendering.html)
