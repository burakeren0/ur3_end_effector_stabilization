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
add_executable(computed_torque_node src/computed_torque.cpp)
target_include_directories(computed_torque_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(computed_torque_node rclcpp sensor_msgs std_msgs)

# ---> YENİ EKLENEN PARKOUR DÜĞÜMLERİ <---

# 1. Target Pose Düğümü
add_executable(target_pose src/target_pose.cpp)
ament_target_dependencies(target_pose rclcpp sensor_msgs std_msgs)

# 2. Inverse Kinematics Düğümü
add_executable(inverse_kinematics_node src/inverse_kinematics.cpp)
ament_target_dependencies(inverse_kinematics_node rclcpp std_msgs)


# ==============================================================================
# 2. C++ DÜĞÜMLERİNİN SİSTEME KURULMASI (ros2 run İÇİN ZORUNLU)
# ==============================================================================
install(TARGETS
  computed_torque_node
  #computed_torque_v_node
  #imu_test_node
  #world_frame_torque_control_node
  #quaternion_inverse_node
  target_pose
  inverse_kinematics_node
  # computed_torque_cpp  # Eski hedef adı; computed_torque_node yukarıda derlenip kuruluyor.
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
