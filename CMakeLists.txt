cmake_minimum_required(VERSION 3.8)
project(ur3_end_effector_stabilization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- BAĞIMLILIKLARIN BULUNMASI ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(clearpath_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ==============================================================================
# 1. C++ DÜĞÜMLERİNİN (NODES) DERLENMESİ
# ==============================================================================

# Eski Düğümlerin
add_executable(computed_torque_node src/computed_torque.cpp)
target_include_directories(computed_torque_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(computed_torque_node rclcpp sensor_msgs std_msgs)

add_executable(computed_torque_v_node src/computed_torque_Vposition.cpp)
target_include_directories(computed_torque_v_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(computed_torque_v_node rclcpp sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs)

add_executable(imu_test_node src/imu_test_node.cpp)
ament_target_dependencies(imu_test_node rclcpp sensor_msgs tf2 tf2_geometry_msgs)

# ---> YENİ EKLENEN AKTİF 3D ORYANTASYON KONTROLCÜSÜ <---
add_executable(world_frame_torque_control_node src/world_frame_torque_control.cpp)
target_include_directories(world_frame_torque_control_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(world_frame_torque_control_node rclcpp sensor_msgs std_msgs)

add_executable(quaternion_inverse_node src/quaternion_inverse.cpp)
target_include_directories(quaternion_inverse_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(quaternion_inverse_node rclcpp sensor_msgs std_msgs)


# ==============================================================================
# 2. C++ DÜĞÜMLERİNİN SİSTEME KURULMASI (ros2 run İÇİN ZORUNLU)
# ==============================================================================
install(TARGETS
  computed_torque_node
  computed_torque_v_node
  imu_test_node
  world_frame_torque_control_node
  quaternion_inverse_node
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# 3. DİĞER KLASÖR VE PYTHON SCRİPTLERİNİN KURULUMU
# ==============================================================================
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
  scripts/world_generators/generate_bumpy_world.py
  scripts/world_generators/generate_street_world.py
  scripts/world_generators/generate_hilly_world.py
  scripts/neural_network/raw_data_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()