cmake_minimum_required(VERSION 3.16)
project(roscon_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IKSolution.msg" "msg/IKSolutions.msg" "msg/IKSelection.msg"
  "srv/PlanMotion.srv" "srv/SetCollisionBackend.srv" "srv/ValidateTrajectory.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs trajectory_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
