cmake_minimum_required(VERSION 3.8)
project(ur3_end_effector_stabilization)

# Enable strict compiler warnings during development
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# SECTION 0: FINDING DEPENDENCIES
# ==============================================================================
# This section finds all required ROS 2 packages for the project build.
# Each find_package() call searches for the CMake files of the specified package.

# Build system
find_package(ament_cmake REQUIRED)           # ROS 2 build system (ament)

# Core ROS 2 packages
find_package(rclcpp REQUIRED)                # ROS 2 C++ client library
find_package(std_msgs REQUIRED)              # Standard message types
find_package(geometry_msgs REQUIRED)         # Geometry message types (poses, transforms)
find_package(sensor_msgs REQUIRED)           # Sensor message types (joint states)

# Robot description and state publishing
find_package(urdf REQUIRED)                  # URDF file parsing
find_package(xacro REQUIRED)                 # XACRO macro processing
find_package(robot_state_publisher REQUIRED) # Publish robot state from joint values

# Transform library (TF2 - transform handling)
find_package(tf2 REQUIRED)                   # Core transform library
find_package(tf2_ros REQUIRED)               # ROS 2 integration for TF2
find_package(tf2_geometry_msgs REQUIRED)     # Geometry message support for TF2

# Linear algebra and math
find_package(Eigen3 REQUIRED)                # Eigen matrix library for computations

# Testing dependencies (optional, only if tests are enabled)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ==============================================================================
# SECTION 1: BUILDING C++ NODES (Executables)
# ==============================================================================
# Each add_executable() line creates an executable from a C++ source file.
# The ament_target_dependencies() line specifies required ROS 2 packages
# and linking libraries for that program.

# Computed Torque Controller Node
# Purpose: Manipulator output control (computed torque control)
add_executable(computed_torque_node src/computed_torque.cpp)
target_include_directories(computed_torque_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(computed_torque_node rclcpp sensor_msgs std_msgs)

# Target Pose with Full RPY Node
# Purpose: Publish target pose with individual Roll, Pitch, Yaw angle control
add_executable(target_pose_full_rpy src/target_pose_full_rpy.cpp)
ament_target_dependencies(target_pose_full_rpy rclcpp sensor_msgs std_msgs)

# Inverse Kinematics Node
# Purpose: Transform Cartesian position to joint angles (inverse kinematics)
add_executable(inverse_kinematics_node src/inverse_kinematics.cpp)
ament_target_dependencies(inverse_kinematics_node rclcpp std_msgs)


# ==============================================================================
# SECTION 2: INSTALLING NODES TO THE SYSTEM (Install Executables)
# ==============================================================================
# This section installs the compiled C++ programs to the system.
# After installation, they can be run with 'ros2 run' command.
# DESTINATION lib/${PROJECT_NAME}: Programs are copied to the
# install/lib/ur3_end_effector_stabilization/ directory.

install(TARGETS
  computed_torque_node
  target_pose_full_rpy
  inverse_kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# SECTION 3: INSTALLING RESOURCE FILES (Install Resources)
# ==============================================================================
# This section installs files used by the nodes (required at runtime) to the system.
# These files include launch configurations, URDF robot definitions, SDF world files,
# controller configurations, and RViz settings.
# DESTINATION share/${PROJECT_NAME}: Files are copied to the
# install/share/ur3_end_effector_stabilization/ directory.

install(DIRECTORY
  launch          # ROS 2 launch files (for automatic node startup)
  urdf            # Robot URDF definition files
  worlds          # Gazebo simulation world files (SDF format)
  config          # Controller configuration files (YAML)
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
