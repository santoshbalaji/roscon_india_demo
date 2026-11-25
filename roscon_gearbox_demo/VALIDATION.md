# Validation performed

- Built roscon_gearbox_demo and its dependencies with ROS 2 Jazzy.
- Ran validate_cell successfully: all three ordered pose requests passed IK,
  RRTConnect + STOMP planning, Cartesian endpoint checks, and sampled edge
  validation with each of FCL, spheres and SDF (nine successful plans).
- All three backends detected environment collision for the lowered test arm and
  rejected the path into the fixture. FCL identified output_shaft; conservative
  backends identified housing_front_wall along the sampled path.
- Loaded the scene in actual RViz with the roscon_rviz_plugins planner panel.
  Load demo target followed by PRM + CHOMP-style planning succeeded with FCL.
  Confirmed 44 environment objects, an interactive target and a tool-path marker.
- Captured docs/gearbox-scene.png from that RViz run.

The validator is registered as gearbox_pipeline for colcon/CTest. The RViz
check used an isolated ROS domain and stopped only its own launched processes.
No physical robot, contact dynamics or grasped-part validation was performed.
