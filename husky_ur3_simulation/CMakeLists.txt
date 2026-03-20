cmake_minimum_required(VERSION 3.8)
project(husky_ur3_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# Python scriptlerini yükleme komutu

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

install(PROGRAMS
  scripts/ur3_mover.py
  scripts/husky_circle.py
  scripts/master_mover.py
  scripts/forward_kinematic_analysis.py
  scripts/velocity_kinematics_analysis.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  urdf 
  rviz 
  config
  DESTINATION share/${PROJECT_NAME}
)
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

add_executable(computed_torque_cpp src/computed_torque.cpp)
target_include_directories(computed_torque_cpp PUBLIC
  $<BUILD_INTERFACE:${Eigen3_INCLUDE_DIRS}>
)
# ROS 2 paketleri ament ile bağlanır
ament_target_dependencies(computed_torque_cpp rclcpp sensor_msgs std_msgs)

# Eigen3 gibi standart kütüphaneler target_link_libraries ile bağlanır
target_link_libraries(computed_torque_cpp Eigen3::Eigen)

install(TARGETS
  computed_torque_cpp
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
