# Code guide

Each package has one role:

| Package | Start here | Responsibility |
| --- | --- | --- |
| `roscon_description` | `urdf/irb2600.urdf` | ABB links, joints, limits and mesh resources |
| `roscon_msgs` | `msg/` and `srv/` | Messages shared by the nodes and RViz panels |
| `roscon_collision` | `include/roscon_collision/scene_checker.hpp` | Check robot configurations against the robot and environment |
| `roscon_kinematics` | `include/roscon_kinematics/solver.hpp` | Forward kinematics, Jacobian and constrained IPOPT IK |
| `roscon_planner` | `src/planner.cpp` | Generate IK goals, plan with OMPL, refine and validate the path |
| `roscon_rviz_plugins` | `src/*_panel.cpp` | Display status and send requests from RViz |
| `roscon_gearbox_demo` | `config/` and `launch/demo.launch.py` | Assembly fixtures, target presets and a repeatable pipeline check |

## Follow a planning request

1. `PlannerPanel::plan()` sends the selected target and algorithms.
2. The planner node converts the ROS request into a target for `Planner::plan()`.
3. `findGoalConfigurations()` solves IK with predefined seeds, rejects invalid
   configurations and sorts the remaining solutions by distance from the start.
4. OMPL searches for a path. The chosen optimizer may refine that path.
5. `validatePath()` samples every edge; `validState()` checks joint limits,
   singularity margin and collisions. An invalid refinement is not accepted.
6. The node publishes the accepted trajectory and RViz markers. Playback is a
   separate request.

The collision checker uses one `ignoreSelfPair()` rule when building sphere
pairs, checking FCL collisions and calculating FCL clearance. Keep those checks
consistent when changing collision exclusions.

The GUI timer spins ROS callbacks, then calls `updateControls()`. Busy-state
changes call the same control-update function immediately, without spinning the
executor inside a callback.

## Editing and checking

Use small named functions for separate operations, explicit loops and early
returns for invalid input. Keep units and mathematical assumptions beside the
relevant calculation. Do not hide checks behind generic callback wrappers.

The repository `.clang-format` applies to all C++ and CUDA sources. Generated
Python caches are ignored and do not belong in source control.

From the workspace root, after sourcing ROS and the existing workspace:

```bash
colcon build --packages-up-to roscon_gearbox_demo --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select roscon_collision roscon_kinematics roscon_planner roscon_rviz_plugins roscon_gearbox_demo
colcon test-result --verbose
```

See each package's README and tests for its supported behavior. GPU integration
checks are opt-in; skipped GPU checks do not establish GPU runtime correctness.
