# ABB model attribution

URDF and meshes are adapted from ROS-Industrial abb_irb2600_support:
https://github.com/ros-industrial/abb/tree/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb2600_support

Upstream license: Apache-2.0, reproduced in LICENSE.
Upstream authors: Cody Bressler, Jonathan Meyer, Anouk Leunissen, Menno Sytsma,
Levi Armstrong, and the ROS-Industrial community.

Changes: expanded the IRB 2600-12/1.65 xacro into a standalone URDF, embedded
its orange material, changed mesh package paths to roscon_description, and removed
the empty tool0 frame and its fixed joint. The link_6 visual mesh uses a 0.1 URDF
scale to match the dimensions of the supplied collision mesh.
Kinematic dimensions, joint limits, visual meshes, and collision meshes are
retained from upstream. No ROS 1 packages are needed.
