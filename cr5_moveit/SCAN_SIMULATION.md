# CR5 circular scan simulation

## Contact TCP (current convention)

The controlled and observed link is now `probe_tcp`, the nominal center of
the broad planar contact end of the mesh. The mesh has a planar patch near
local X=50 mm, with Y span about 45.4 mm and Z span about 13.5 mm. The TCP
uses the nominal mesh position (0.05, 0, 0) m, not a physical calibration.

- TCP X = mesh Y: long edge of the contact face.
- TCP Y = mesh Z: short edge of the contact face.
- TCP Z = mesh X: outward normal of the contact face.

The fixed TCP joint has RPY (pi/2, 0, pi/2). With the existing mounting,
TCP Z is parallel to flange (`Link6`) Z, and TCP is 181 mm along flange Z;
TCP X/Y are rotated 90 degrees around flange Z. Spinning the flange around
its own Z therefore rotates the contact long edge while preserving its normal.
The mesh's overall long axis describes the handle/body, not the contact long edge.

The SRDF chain tip, motion nodes, read-pose default and trace default all use
`probe_tcp`. The original `ultrasound_probe` mesh link remains available for
comparison via `end_effector_link:=ultrasound_probe`. RViz displays TCP axes
(red X, green Y, blue Z). Restart the demo after rebuilding so all nodes use
the same model. Old positions and circle centers referred to the mesh origin;
new values refer to the contact center and will differ.

```bash
ros2 launch cr5_moveit read_pose.launch.py
ros2 launch cr5_moveit trace_end_effector.launch.py
```

## Discrete position-then-orientation test

### Independent center marker

With the demo stationary, start this independent observer before orbit motion:

```bash
ros2 launch cr5_moveit scan_center_marker.launch.py
```

It captures the current `probe_tcp` position once and displays a red 8 mm
sphere, white "Scan center" label, and a yellow XY reference circle (radius
0.05 m by default). RViz's `Scan Center` MarkerArray display subscribes to
`/cr5_scan_center`. Add that display manually if using an older RViz session.
Use `show_circle:=false` for center/label only, or `radius:=...` to match the
motion radius. The marker node never moves the robot and does not follow it.

Start the orbit from that same stationary pose so both nodes capture the same
center; compare their printed coordinates. If the center changes, stop and
restart the marker node at the new starting pose. Starting it midway through
motion would mark the then-current TCP position, not the original orbit center.

`stepwise_orbit.launch.py` separates translation and orientation planning.
Each 30-degree circle arc holds the current probe orientation. At the next
station, after initial downward alignment, the node generates a trajectory
changing joint6 only. Joints 1-5 are copied unchanged; no full-arm IK is used
for pure contact yaw. FK verifies that joint6 reaches the requested orientation
and preserves the contact center. The initial downward alignment and final
tilt restoration still use Cartesian IK. Failure stops the scan: free
joint-space reorientation is no longer used. TCP position must remain within
1 mm during each rotation; initial/restoration pure-yaw checks also limit
joints 1-5 to within 2 degrees of their stage start.
This is a free-space simulation
without a contact object. The translation arcs are checked for radius, height, held
orientation and signed angular coverage before execution.

Default `allow_axis_flip:=true` treats the long edge as an undirected radial
line, so either end may face the center. Of the two targets 180 degrees apart,
the shortest joint6 movement that remains within its configured position and
travel limits is chosen. Near a wrist limit, this may select a larger wrist
turn (for example 150 degrees instead of an infeasible 30-degree turn), while
the other five joints remain fixed. Set
`allow_axis_flip:=false` if the two ends must retain their inward/outward roles.

The stepwise node coordinates the arm for translation and initial alignment,
but uses joint6-only yaw at stations. Endpoint normal
is downward and contact long edge is radial. The new TCP distinguishes the
contact long edge from the overall mesh length.

The node removes equivalent +/-2*pi revolute angle representation jumps
before recomputing trajectory timing. It then checks total absolute joint
travel of joints 1-5 in each stage (`max_stage_joint_travel_deg`, default 90),
wrist travel (`max_wrist_stage_travel_deg`, default 180), and departure
of joints 1-5 from their startup values (`max_arm_excursion_deg`, default 60).
These checks apply to intermediate states as well as endpoints. Equivalent
angle correction alone does not permit a genuine large joint loop.

A 6 m x 6 m ground slab is added to the MoveIt scene with top at `floor_z`
(default -0.01 m in the model frame, 10 mm below the nominal mounting plane).
The current full planning scene, including this object, is fetched before
planning. Interpolated states are checked for self/environment/ground collision.
The modeled floor height must match the simulated installation; the RViz grid
alone is not collision geometry. The slab stays in the scene after the node
exits. If startup is already in collision, the program refuses to proceed.
Checks use a scene snapshot and sampled linear joint interpolation, and are
not a hardware safety or continuous collision guarantee.

All stages, including the return to the original center and orientation, must
be prepared before execution. A discrete target may still be unreachable;
separating the stages removes the continuous orientation constraint but does
not make unreachable poses reachable. No lowering or rocking is included.
Each prepared station logs its contact long-edge radial error and normal
alignment error; both must be below 0.01 rad. TCP X (red) should point along
the radius and TCP Z (blue) downward at each station. Radial alignment is
required at stations; fixed-orientation translation between them intentionally
does not maintain radial alignment continuously.

When importing a trajectory message, every waypoint is explicitly updated
before FK validation. MoveIt's const waypoint transform accessor does not
recompute dirty transforms. Without this update, cached poses may cause
incorrect coverage, spacing or endpoint failures. Transfer logs include the
number of stored trajectory points, FK validation samples and maximum angular
gap. Stored trajectory points can be sparse even for a complete Cartesian
path. Validation therefore interpolates the returned joint states at a maximum
per-joint step of 0.005 rad before measuring circle geometry. This does not
change the execution trajectory. It checks linear joint interpolation, not
controller-specific spline interpolation. Geometry, spacing and coverage
checks remain enabled on the interpolated FK samples.

```bash
ros2 launch cr5_moveit stepwise_orbit.launch.py
ros2 launch cr5_moveit stepwise_orbit.launch.py execute:=true
```

After reaching and orienting at each of the 12 stations, execution pauses for
`dwell_seconds` (default 2.0 wall-clock seconds). Set it to 0 to disable pauses.
The first station is included; the closing arc and return to center add no
extra station pauses. Planning-only mode does not wait. Ctrl+C interrupts a
pause without starting the next stage.

```bash
ros2 launch cr5_moveit stepwise_orbit.launch.py execute:=true dwell_seconds:=5.0
```

## Incremental orbit test

`orbit.launch.py` now supports a second step: radial orientation while orbiting.
It records the startup probe position as the center, aligns local X with model
+Y and local +Z downward, approaches the +Y circle point, and follows a full
XY circle. Local X remains radial during the circle. Height remains constant;
there is no lowering, rocking or dwell in this test.

By default `follow_radial:=true` enables this orientation change, and
`return_center:=true` restores the original orientation at the first circle
point and returns to the exact startup position. This reduces center drift on
repeated runs, subject to execution tolerance. `return_center:=false` leaves
the probe on the circumference as before. Alignment/restore rotations are
sampled at no more than one degree per waypoint.

```bash
# Plan the new radial-orientation test first.
ros2 launch cr5_moveit orbit.launch.py
ros2 launch cr5_moveit orbit.launch.py execute:=true
# Compare against the previously successful fixed-orientation motion.
ros2 launch cr5_moveit orbit.launch.py follow_radial:=false
# Adjustable radius, direction and joint speed/acceleration scaling.
ros2 launch cr5_moveit orbit.launch.py clockwise:=true radius:=0.05 speed_scale:=0.1
```

The complete path is planned and checked before execution. In addition to
MoveIt's completion fraction, forward kinematics must show a signed winding
of approximately 360 degrees, sufficient circle samples and a correct final
pose. Execution finishes with a measured endpoint check (1 mm / 0.01 rad).
`speed_scale` must be in (0, 1]. Keep the independent trace observer running.
A successful position-only test does not prove the orientation-changing test
is reachable. The original full scan is still in `circular_scan.launch.py`.

## Full scan

The scan uses the robot model frame (`dummy_link`). The probe position at
startup defines the circle center, not a calibrated hemisphere center.

Simulation geometry assumptions for `probe_tcp`:

- Local X is the contact-face long edge (mesh Y).
- Local +Z is the nominal contact normal (mesh X). The code aligns it
  with model -Z before the scan. Confirm the mesh end against actual CAD
  and calibrate the physical probe before hardware use.
- Local Y is the transverse rocking axis. It rotates with each station.
- Rotation is about the nominal contact center. No surface model, contact
  detection, force control, or physical surface fitting is implemented.

The node first selects an IK branch and plans a joint-space setup motion to
the downward pose at the startup position. The setup path may translate the
probe; it is not constrained to rotate in place. It then moves +Y by
50 mm. At each of 12 stations it lowers 50 mm, dwells (default 2 seconds),
rocks +10, -10, +20, -20, 0 degrees about local Y, and raises 50 mm.
Between stations it follows a 30-degree XY arc of radius 50 mm, rotating the
probe so local X remains radial. The final arc closes 360 degrees without
repeating the first station's task. The final pose is the first circle
station's downward-facing pose, not the original center pose.

All 98 stages are planned with collision checks before any execution.
The planner tests up to `ik_attempts` initial IK seeds (default 12), discards
partial scans, and selects a branch only after all scan stages and the setup
motion succeed. Random seeds do not guarantee distinct branches or success.
`clockwise:=true` reverses the circle traversal; the default is counterclockwise.
The full orientation sweep may be unreachable from some CR5 configurations;
failure to plan any stage prevents execution of the entire sequence.

With the demo and independent trace observer already running:

```bash
ros2 launch cr5_moveit circular_scan.launch.py
ros2 launch cr5_moveit circular_scan.launch.py execute:=true dwell_seconds:=2.0
ros2 launch cr5_moveit circular_scan.launch.py clockwise:=true ik_attempts:=24
```

The trace observer draws the contact TCP translation; it does not show
in-place rocking. Observe the probe model to inspect orientation changes.
