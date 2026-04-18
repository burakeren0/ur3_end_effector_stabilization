cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(clearpath_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(computed_torque_node src/computed_torque.cpp)
target_include_directories(computed_torque_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(computed_torque_node rclcpp sensor_msgs std_msgs)

add_executable(computed_torque_v_node src/computed_torque_Vposition.cpp)
target_include_directories(computed_torque_v_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(computed_torque_v_node rclcpp sensor_msgs std_msgs tf2 tf2_ros geometry_msgs)

install(DIRECTORY
  launch
  urdf
  worlds
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/robot_dancer.py
  scripts/velocity_kinematics_analysis.py
  scripts/generate_bumpy_world.py
  scripts/generate_street_world.py
  scripts/raw_data_logger.py
  scripts/active_stabilizer_v3.py
  scripts/angle_calculator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  computed_torque_node
  computed_torque_v_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
