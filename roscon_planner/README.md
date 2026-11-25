# ROSCon motion planner

`roscon_planner` provides a standalone ROS 2 Jazzy demo for the existing ABB
IRB 2600 URDF. It offers **OMPL RRTConnect, RRTstar, or PRM**, followed by optional **STOMP or
CHOMP-style covariant refinement**,
with runtime-selectable FCL, sphere, or SDF collision checks. The existing IPOPT
solver provides collision-free joint-space goals for a full Cartesian target.

## Build and launch

Install dependencies on a new machine (the current workspace already has them):

```bash
sudo apt-get install ros-jazzy-ompl ros-jazzy-stomp coinor-libipopt-dev \
  libfcl-dev libassimp-dev libyaml-cpp-dev ros-jazzy-rviz2 \
  ros-jazzy-interactive-markers ros-jazzy-robot-state-publisher
```

Build the package and its local dependencies:

```bash
cd ~/event_ws/roscon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash  # Existing workspace: includes the local IPOPT environment hook.
colcon build --packages-up-to roscon_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch roscon_planner demo.launch.py
```

On this machine IPOPT lives under `~/event_ws/roscon_ws/.deps/ipopt`; see the
kinematics README for its CMake prefix and environment hooks. A fresh system
installation does not need that prefix. No CUDA dependency is needed by the
planner's collision checks; the collision package's separate GPU demo is optional.

Run this launch **instead of** the IK or collision demo launch. It owns
`/joint_states` and starts one robot state publisher. Concurrent demos would
publish conflicting robot configurations.

## RViz workflow

1. In **Motion planning**, press **Load demo target**, or edit XYZ (metres) and
   roll/pitch/yaw (degrees). You can also use RViz's **Interact** tool to drag or
   rotate the always-visible target handles. Mouse release updates the target
   and clears the previous path; it does not move the robot.
2. Choose **fcl**, **spheres**, or **sdf**. Select an **OMPL planner** and an
   **Optimizer** from the two dropdowns. Defaults are RRTConnect + STOMP.
   Choose **None (OMPL only)** to compare the unrefined result. Each Plan request
   uses the currently selected pair; changing these selections does not modify
   a path that was already generated.
3. Press **Plan**. Planning uses the displayed joints as its start and leaves the
   robot there. The panel reports the result. A green line shows the planned
   end-effector path; the full joint trajectory is published separately.
4. Press **Validate stored path** to run the segment checks again. The result
   reports checked samples, or the first failed edge and collision/constraint.
5. Press **Play / resume** to animate the whole robot. **Pause** freezes playback
   and **Play / resume** continues it. A finished path must be replanned from the
   new current configuration or cleared with **Reset to demo start**.
6. Change the collision checker at runtime to compare it against the same path.
   A backend change pauses playback and marks the path red/unvalidated. Validate
   it under the new backend before playback, or plan again. Different backends
   can disagree because their geometric representations differ.

Reset is an explicit visualization jump to the checked initial configuration,
not a planned return motion. All status text stays in the panel. Current joints
and Cartesian position are also shown there. Numeric edits are drafts until Plan
is pressed; the interactive marker shows the last submitted target.

The robot is slightly translucent to keep the tool path visible; adjust the
Robot display's Alpha property to make it opaque. This is a sequential search
and refinement pipeline, not online reactive replanning.

The panel lives in `roscon_rviz_plugins` and is exported as `roscon_rviz_plugins/PlannerPanel`. Its ROS
clients are asynchronous, keeping Qt responsive while the backend computes.
The backend handles one command at a time. OMPL has a time budget; IK and STOMP
have iteration limits. Pause controls playback, not an in-flight plan.

## Planning and acceptance, step by step

1. Load bounded revolute-joint kinematics from `roscon_description`, and load
   collision geometry and static obstacles through `roscon_collision::SceneChecker`.
   The configured base must be the URDF root/environment frame.
2. Check the start configuration's joint bounds, scaled Jacobian singular-value
   margin, and collisions. Invalid starts stop planning immediately.
3. Try the current joints, up to 24 nearby seeds (each joint offset by +/-0.5
   or +/-1.0 rad if inside its limits), and 16 global predefined IPOPT seeds for
   the requested full pose. Nearby seeds help find the current wrist/elbow branch
   instead of only a branch separated from the start by singularities. Keep
   distinct candidates that meet pose tolerances and pass the same
   validity checks. Sort them by joint distance from the start before adding
   them as OMPL goal states. If none are found, report a
   failed search; this does not prove the target is globally infeasible.
4. The selected OMPL planner searches bounded real-valued joint space.
   RRTConnect emphasizes finding a feasible path; RRTstar improves joint-space
   path length during its time budget; PRM constructs a roadmap for this request.
   The RRT variants use a 0.4 rad Euclidean extension range. Both states and the edges
   between them are checked. Approximate OMPL solutions are rejected. Simplification
   uses the same validity checker. The path is interpolated to at least 40 points.
5. Optional **STOMP** refinement uses the actual `stomp::Stomp` implementation.
   `CollisionTask` supplies temporally correlated Gaussian noise and validity
   penalties. STOMP computes rollout probabilities and acceleration control costs.
   Noise and updates are zero at both endpoints. Joint limits are checked, not
   silently clipped. The task noise uses a fixed seed; OMPL remains stochastic.
   **CHOMP-style covariant** refinement instead minimizes joint acceleration
   energy plus a quadratic penalty inside `clearance_margin`. Clearance comes
   from the selected FCL/sphere/SDF model. Central differences estimate obstacle
   gradients; an acceleration-based metric conditions the update. Endpoints
   stay fixed, and backtracking accepts only lower-cost, validated paths.
   This is an educational covariant optimizer, not the upstream MoveIt CHOMP
   implementation: it omits workspace arc-length integration and momentum/HMC.
   Minimum-distance gradients are approximate at changing closest features;
   FCL penetration distances are not treated as exact signed distances. The
   hard collision checker decides final acceptance, regardless of optimizer costs.
6. Independently validate every output edge. An optimizer's waypoint cost alone is not
   sufficient: intermediate collisions can exist between waypoints. Invalid or
   failed refinement retains the already-validated OMPL path and reports this.
   Paths over 200 points skip optimization and report that choice.
7. Recheck the final Cartesian position and orientation. Publish only an accepted
   path. Failed requests clear the old path, preventing stale-plan playback.
8. Playback revalidates the stored path under the active backend before starting.
   It interpolates the validated joint-space segments with display timestamps.

No shortest-path guarantee is made. RRTConnect can choose any reachable IK goal,
and a local STOMP refinement does not guarantee a globally optimal path.

## Collision methods and validation resolution

| Backend | Robot self-collision | Robot against environment |
| --- | --- | --- |
| `fcl` | URDF collision geometry through FCL | Collision geometry through FCL |
| `spheres` | Sphere pairs | Robot spheres against environment spheres |
| `sdf` | Sphere pairs | Robot sphere centres evaluated against obstacle signed-distance fields, accounting for sphere radius |

All three share URDF transforms, static obstacle definitions, adjacent-link
exclusions, and the existing explicit `[link_4, link_6]` wrist-mesh exclusion.
The sphere cover conservatively fills collision-geometry bounding boxes at a
0.06 m requested radius. It can reject close configurations accepted by FCL.
Primitive SDFs are analytic; mesh SDFs use the existing closed-mesh evaluator.
Environment assets must support all three methods; mesh obstacles must be closed
and consistently oriented. All checks here run on the CPU.

`validation_joint_step` defaults to **0.03 rad maximum change of any joint per
final validation sample**. OMPL's Euclidean edge resolution is set at least as
fine. This is discrete validation, not continuous collision detection: very thin
obstacles may lie between samples. Smaller steps improve coverage at increased
cost. The allowed range is 0.001–0.1 rad. Singularity and joint bounds are also
checked at sampled configurations. There is no guarantee about unsampled states.

The emitted `JointTrajectory` times are for visualization (approximately
0.4 rad/s maximum joint increment, at least 0.02 s per stored segment). They are
not hardware-ready timing: no acceleration, jerk, controller, or certified
velocity-limit validation is performed. No hardware commands are sent.

## Configuration and obstacle tests

Edit `config/planner.yaml` for the OMPL time budget, STOMP iterations/rollouts,
noise, CHOMP-style iterations/clearance margin, validation resolution, initial configuration, and singularity threshold.
The default singular-value margin is 0.05, as in the IK demo. Joint-limit margin
and IK tolerances use that solver's defaults (0.01 rad, 5 mm, 0.02 rad orientation).

`config/environment.yaml` includes the workbench, pillar, and ball. Their RViz
surfaces and collision representations are generated from the same definitions.
The environment is static; edit it and relaunch:

```bash
ros2 launch roscon_planner demo.launch.py \
  environment_file:=/absolute/path/environment.yaml \
  parameters:=/absolute/path/planner.yaml
```

A simple rejection test: load the demo target, change XYZ to the ball centre
`[0.65, -0.65, 1.0]`, and press Plan. It should report no collision-free IK goal,
clear the path, and leave the displayed robot unchanged. An empty `objects: []`
scene still performs self-collision checks. Another URDF can be passed with
`urdf_file:=...`; adjust base, tip, initial joints, and RViz fixed frame accordingly.

## ROS interfaces

Shared service definitions live in the generic `roscon_msgs` package.
`PlanMotion.ompl_planner` accepts `rrt_connect`, `rrt_star`, or `prm`;
`PlanMotion.optimizer` accepts `none`, `stomp`, or `chomp`. Empty values select
`rrt_connect` and `stomp`. Unknown values are rejected.

| Interface | Type | Purpose |
| --- | --- | --- |
| `/planner/plan` | `roscon_msgs/srv/PlanMotion` | Full-pose target, `ompl_planner`, and `optimizer`; returns accepted trajectory or failure |
| `/planner/set_backend` | `roscon_msgs/srv/SetCollisionBackend` | Switch `fcl`, `spheres`, `sdf`; pause and invalidate stored-path approval |
| `/planner/validate` | `roscon_msgs/srv/ValidateTrajectory` | Validate explicit positions, or stored path when input points are empty; optional backend override |
| `/planner/play`, `/planner/stop`, `/planner/reset` | `std_srvs/srv/Trigger` | Visualization controls |
| `/planner/trajectory` | `trajectory_msgs/msg/JointTrajectory` | Latched full joint path; empty on invalidation/failure |
| `/planner/path_markers` | `visualization_msgs/msg/MarkerArray` | Green validated or red unvalidated tool path |
| `/planner/environment` | `visualization_msgs/msg/MarkerArray` | Latched obstacle surfaces |
| `/planner/current_pose`, `/planner/target_pose`, `/planner/demo_target` | `geometry_msgs/msg/PoseStamped` | Current, submitted, and example poses |
| `/planner/status`, `/planner/backend` | `std_msgs/msg/String` | GUI feedback and confirmed backend |
| `/joint_states` | `sensor_msgs/msg/JointState` | Displayed robot configuration |
| `/planner_target` | Interactive marker server namespace | Target translation/rotation controls |

Explicit validation inputs must use the robot's exact joint names and order.
Validation checks geometric positions only; velocities and timing are not certified.
A validation with an alternate backend does not approve playback under the active
backend. An unsupported backend or failed collision computation never approves a path.

## Code and tests

- `src/planner.cpp`: IK goals, OMPL search, sampled edge checks, final acceptance.
- `src/stomp_refinement.cpp`: small upstream STOMP task adapter.
- `src/chomp_refinement.cpp`: explicit covariant updates, clearance gradients, and backtracking.
- `src/node.cpp`: target handling, services, markers, trajectory messages, playback.
- `../roscon_rviz_plugins/src/planner_panel.cpp`: asynchronous Qt/ROS controls.
- `../roscon_collision/src/scene_checker.cpp`: swappable collision implementations.

```bash
colcon test --packages-select roscon_planner roscon_rviz_plugins --return-code-on-test-failure
colcon test-result --test-result-base build/roscon_planner --verbose
colcon test-result --test-result-base build/roscon_rviz_plugins --verbose
```

Tests include an obstacle strictly between two collision-free endpoints for all
three checkers, malformed-path rejection, ABB OMPL+STOMP plans, final-pose checks,
blocked goals, all nine OMPL/optimizer combinations, covariant smoothing,
GUI algorithm selections, backend-change playback gating, revalidation,
pause/resume, reset, and stale-path clearing after failure. Existing IK and
collision package tests are run after changes to their shared libraries.

References: [OMPL state and motion validation](https://ompl.kavrakilab.org/core/stateValidation.html)
and [STOMP](https://github.com/ros-industrial/stomp).

A scene package can set the optional `demo_joints` parameter (one finite, bounded
value per robot joint) to change the pose loaded by **Load demo target**. This
only calculates a target pose; it does not move the robot or bypass planning.
See `roscon_gearbox_demo` for an assembly fixture environment.
