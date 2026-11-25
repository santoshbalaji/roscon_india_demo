cmake_minimum_required(VERSION 3.16)
project(roscon_kinematics)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(roscon_msgs REQUIRED)
find_package(roscon_collision REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
# Optional user-local installation; system IPOPT remains the default.
set(IPOPT_PREFIX "" CACHE PATH "Persistent IPOPT prefix containing lib/pkgconfig/ipopt.pc")
if(IPOPT_PREFIX)
  if(NOT EXISTS "${IPOPT_PREFIX}/lib/pkgconfig/ipopt.pc")
    message(FATAL_ERROR "IPOPT_PREFIX must contain lib/pkgconfig/ipopt.pc")
  endif()
  set(ENV{PKG_CONFIG_PATH} "${IPOPT_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
  # Install environment hooks so transitive dependencies (MUMPS, Scotch) resolve
  # after sourcing this workspace, not just inside the build shell.
  configure_file(config/ipopt_paths.dsv.in ${CMAKE_CURRENT_BINARY_DIR}/ipopt_paths.dsv @ONLY)
  ament_environment_hooks(${CMAKE_CURRENT_BINARY_DIR}/ipopt_paths.dsv)
endif()
pkg_check_modules(IPOPT REQUIRED IMPORTED_TARGET ipopt)
add_library(ik_solver SHARED src/solver.cpp src/multi_seed.cpp)
target_include_directories(ik_solver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(ik_solver PUBLIC Eigen3::Eigen PkgConfig::IPOPT ${orocos_kdl_LIBRARIES})
ament_target_dependencies(ik_solver PUBLIC kdl_parser urdf)
add_executable(ik_node src/node.cpp)
target_link_libraries(ik_node ik_solver roscon_collision::scene_checker)
ament_target_dependencies(ik_node rclcpp sensor_msgs geometry_msgs visualization_msgs std_msgs std_srvs interactive_markers roscon_msgs)
install(TARGETS ik_solver ik_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(roscon_description REQUIRED)
  ament_add_gtest(test_abb test/test_abb.cpp)
  target_link_libraries(test_abb ik_solver)
  target_compile_definitions(test_abb PRIVATE
    ABB_TEST_URDF="${roscon_description_DIR}/../urdf/irb2600.urdf")
  ament_add_gtest(test_collision_filter test/test_collision_filter.cpp)
  target_link_libraries(test_collision_filter ik_solver roscon_collision::scene_checker)
  target_compile_definitions(test_collision_filter PRIVATE
    ABB_TEST_URDF="${roscon_description_DIR}/../urdf/irb2600.urdf"
    ABB_SCENE="${CMAKE_CURRENT_SOURCE_DIR}/config/environment.yaml"
    EMPTY_SCENE="${CMAKE_CURRENT_SOURCE_DIR}/test/empty_scene.yaml"
    BLOCKED_SCENE="${CMAKE_CURRENT_SOURCE_DIR}/test/blocked_scene.yaml")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(collision_filter_integration test/test_collision_integration.py
    TIMEOUT 120
    ENV IK_EXECUTABLE=$<TARGET_FILE:ik_node>
        IK_PARAMETERS=${CMAKE_CURRENT_SOURCE_DIR}/config/solver.yaml
        IK_URDF=${roscon_description_DIR}/../urdf/irb2600.urdf
        IK_ENVIRONMENT=${CMAKE_CURRENT_SOURCE_DIR}/config/environment.yaml
        ROS_DOMAIN_ID=96)
  ament_add_gtest(test_solver test/test_solver.cpp)
  target_link_libraries(test_solver ik_solver)
endif()
ament_export_include_directories(include)
ament_export_libraries(ik_solver)
ament_export_dependencies(Eigen3 kdl_parser urdf orocos_kdl_vendor orocos_kdl)
ament_package()
