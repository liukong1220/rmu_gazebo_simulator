cmake_minimum_required(VERSION 3.8)
project(rmu_gazebo_simulator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

# PointCloud2 -> livox CustomMsg adapter. C++ rather than Python because the
# rclpy version cost a fixed 0.204 s per frame in per-point message allocation,
# which pushed the downstream /odometry stamps far enough behind /clock to break
# the planning-grid adapter's TF lookup. See src/gz_livox_bridge.cpp.
ament_auto_add_executable(gz_livox_bridge_node
  src/gz_livox_bridge.cpp
)

# Gazebo publishes /clock once per simulation step. Forwarding every step into
# ROS makes each use_sim_time node process hundreds of redundant clock samples
# per wall-clock second on the target laptop. Keep the original simulated time
# value while bounding that fan-out in one, single-owner relay.
ament_auto_add_executable(gz_clock_relay_node
  src/gz_clock_relay.cpp
)
target_link_libraries(gz_clock_relay_node
  ignition-msgs8::ignition-msgs8
  ignition-transport11::ignition-transport11
)

# A one-shot C++ health witness used by the Gazebo regression.  It observes the
# existing localization/map contracts without starting several rclpy CLI
# subscribers that can perturb the resource-constrained simulation under test.
ament_auto_add_executable(ats_navigation_health_probe
  src/ats_navigation_health_probe.cpp
)

# Collect bounded action-lifetime evidence without multiple ros2cli Python
# subscribers perturbing the same sensor-to-control loop under test.
ament_auto_add_executable(ats_navigation_evidence_recorder
  src/ats_navigation_evidence_recorder.cpp
)

# This isolated test client owns the action it cancels after the first actual
# MPC command. It is never launched by the normal simulation profile.
ament_auto_add_executable(ats_navigation_cancel_on_command_client
  src/ats_navigation_cancel_on_command_client.cpp
)

# Keep the ATS 4WD4WS actuator beside the Gazebo integration it serves.  The
# generic rmoss plugin package remains a dependency for its other systems, but
# this library is installed by the user-owned simulator fork so a clean
# `vcs import` does not depend on a local ignored source patch.
add_library(AtsSwerveDrive4WS SHARED
  src/ats_swerve_drive4ws.cpp
)
target_compile_features(AtsSwerveDrive4WS PRIVATE cxx_std_17)
target_include_directories(AtsSwerveDrive4WS PRIVATE include)
target_link_libraries(AtsSwerveDrive4WS
  ignition-gazebo6::ignition-gazebo6
)

install(TARGETS AtsSwerveDrive4WS
  DESTINATION plugins
)

install(PROGRAMS
  scripts/referee_system/simple_competition_1v1.py
  scripts/ats_bridge/chassis_cmd_adapter.py
  scripts/ats_bridge/executed_path_observer.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  add_executable(test_swerve_kinematics
    test/test_swerve_kinematics.cpp
  )
  target_compile_features(test_swerve_kinematics PRIVATE cxx_std_17)
  target_include_directories(test_swerve_kinematics PRIVATE include)
  add_test(NAME test_swerve_kinematics COMMAND test_swerve_kinematics)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  resource
  config
  launch
  rviz
)
