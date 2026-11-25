# ABB migration validation — 2026-11-26

- Built `roscon_description` and `roscon_kinematics` in the ROSCon workspace.
- All six C++ tests passed, including startup and nearby full-pose IK using the
  installed ABB IRB 2600 URDF (`base_link` to `flange`).
- ROS integration verified six joint positions, five visualization markers, and
  a 1 cm target move. IPOPT returned status 0 and TARGET REACHED. Position error
  rounded to 0.0000 m, orientation error was 0.0002 degrees, and the scaled minimum
  singular value was 0.2252 against the configured 0.05 lower bound.
- RViz launched with the ABB model and OpenGL 4.6, with no logged resource/render
  errors during a six-second smoke test. All processes shut down cleanly.
  Manual mouse interaction was not tested.

The existing robot description and collision packages were reused without code
changes. No robot assets were duplicated.

IPOPT and its MUMPS/Scotch dependencies now reside persistently in
`~/event_ws/roscon_ws/.deps/ipopt`. The package was rebuilt with `IPOPT_PREFIX`
pointing there. The installed environment hook supplies runtime library paths
when sourcing `install/setup.bash`.

After clearing `LD_LIBRARY_PATH` and `PKG_CONFIG_PATH`, sourcing ROS and the
workspace resolved all dependencies without `/tmp`. The ABB ROS target-update
check and all six solver tests passed again with this environment.

## Panel and interactive controls

- Both RViz panel tests and all six solver tests passed (eight Google Tests total).
- The new panel test covers live joint/Cartesian readouts, explicit command
  buttons, retention of target drafts during feedback, RPY conversion, reset,
  interactive enable, stale-state disabling, and absence of a panel joint-state
  publisher. The existing collision panel test still passes.
- Live ABB integration rejected an out-of-limit command and a singular joint
  configuration without moving the robot. It accepted a valid joint command,
  reset correctly, published current Cartesian pose, and solved a target from
  interactive-marker release feedback. Interactive controls switched on and off.
- Marker inspection confirmed no TEXT_VIEW_FACING markers, no marker text, and
  no interactive marker descriptions. Status is displayed in the panel.

## Always-active target rendering fix

The earlier smoke tests checked backend messages and RViz startup but missed an
incorrect display property. Jazzy's InteractiveMarkers display requires
`Interactive Markers Namespace: /ik_target`; `Update Topic` was ignored, leaving
RViz disconnected. The configuration now uses the namespace property and disables
marker descriptions explicitly.

Removed the checkbox, enable service, and enabled-state topic. The server always
provides all six manipulation handles. Both panel tests and all six solver tests
pass. Live drag feedback still reaches the IK solver.

A focused RViz rendering harness loaded the installed configuration, confirmed
connection and marker initialization, and found a populated interactive-marker
scene node. Its render-window capture shows the translation arrows and rotation
rings: `docs/images/interactive-target.png`. This verifies rendering, beyond merely
publishing marker messages.

## Predefined-seed alternatives and generic interfaces

The shared interfaces build as `roscon_msgs`. Both RViz panel tests and all
seven solver/ABB tests pass (nine Google Tests total). The panel test covers
search publication, solution choices, preview generation/index, and list clearing.

A live headless ABB run found two distinct configurations from 17 seeds for
the startup target. Every candidate met the position/orientation tolerances and
singularity threshold. Previewing each preserved the requested target; reset
rejected a delayed selection from the old generation and cancelled a running
search. Duplicate filtering checks account for equivalent full turns.

An RViz rendering check using the X11 Qt platform confirmed the interactive
marker scene and two populated GUI solution choices. The inspected panel capture
is saved as `docs/images/multiple-solutions.png`.

## SDF post-filter for multiple IK candidates

The existing SDF distance formulas now live in a shared header used by both CUDA
and explicit CPU evaluation. The reusable SceneChecker loads the URDF and static
environment, checks each candidate snapshot, and produces obstacle markers from
the same shapes. It uses the original adjacent-link and configured wrist exclusion
policy and the conservative sphere cover.

Build and all enabled tests passed across roscon_collision, roscon_kinematics,
and roscon_rviz_plugins. Ten existing opt-in GPU ROS integration cases were
skipped; the CUDA unit tests ran. A separate ROSCON_ENABLE_CUDA=OFF build of the
checker and CPU SDF tests also passed. Panel tests now use isolated ROS domains
so a running demo cannot add unexpected joint publishers to their assertions.

New checks cover synthetic self-collision, environment collision, invalid joint
input, marker geometry, CPU signed-distance/tangency, ABB seed candidates, GUI
rejection details, and the real ROS node. Live search results with the exact
startup orientation were:

| Target XYZ (m) | Available solutions | Self rejections | Environment rejections |
| --- | --- | --- | --- |
| Startup target | 2 | 0 | 0 |
| Ball: 0.65, -0.65, 1.0 | 0 | 3 | 8 |
| Workbench: 1.15, 0.0, 0.45 | 0 | 0 | 7 |
| Pillar: 0.65, 0.65, 0.9 | 0 | 0 | 6 |

These rejection counts are per checked candidate and categories may overlap.
All searches left displayed joints unchanged and reported zero check errors.
A delayed selection from an older target was rejected. RViz initialized three
obstacle scene nodes, the interactive marker, and two startup solution choices.

This change filters multiple-solution generation/selection only. Single-pose
solving and direct joint controls still enforce kinematics constraints only.
No path checking or hardware control was added.

Inspected RViz captures are saved as `docs/images/collision-environment.png` and
`docs/images/collision-filter-panel.png`. The default camera keeps the target
handles visible alongside all three obstacles.
