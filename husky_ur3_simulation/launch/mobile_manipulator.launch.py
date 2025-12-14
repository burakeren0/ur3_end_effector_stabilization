import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # --- 1. ARGÜMANLARI AL ---
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    
    controllers_file = LaunchConfiguration("controllers_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    description_file = LaunchConfiguration("description_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    
    world = LaunchConfiguration('world')
    
    # --- 2. ROBOT DESCRIPTION (XACRO İŞLEME) ---
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "safety_limits:=", safety_limits, " ",
            "safety_pos_margin:=", safety_pos_margin, " ",
            "safety_k_position:=", safety_k_position, " ",
            "name:=", "ur", " ",
            "ur_type:=", ur_type, " ",
            # DÜZELTME: tf_prefix burada devreye giriyor
            "tf_prefix:=", tf_prefix, " ", 
            "simulation_controllers:=", controllers_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # --- 3. NODE'LAR ---
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    ur_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/controller_manager"],
    )
    
    husky_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller", "-c", "/controller_manager"],
    )

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # --- 4. GAZEBO SİMÜLASYONU ---
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # BOŞ DÜNYA AYARI BURADA YAPILIYOR
    # -r: run (otomatik başlat), -v 4: verbose (hata detayları)
    gz_args_options = [world, ' -r -v 4']

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': gz_args_options}.items(),
    )

    # --- 5. ROBOTU GAZEBO'YA EKLEME (SPAWN) ---
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", "mobile_manipulator",
            "-allow_renaming", "true", # İsim çakışırsa otomatik yeniden adlandır
            "-z", "0.2",
        ],
    )

    # --- 6. BRIDGE ---
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu_data@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen",
    )

    return [
        robot_state_publisher_node,
        gz_sim,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        ur_controller_spawner,
        husky_controller_spawner,
        delay_rviz,
        gz_sim_bridge,
    ]

def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH', '').split(':')]

    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_clearpath_gz, 'worlds') + ':',
            os.path.join(pkg_clearpath_gz, 'meshes') + ':',
            ':' + ':'.join(packages_paths)
        ]
    )

    declared_arguments = []
    
    # --- DEĞİŞİKLİK 1: BOŞ DÜNYA AYARI ---
    # Varsayılan değeri 'warehouse' yerine 'empty.sdf' yaptık.
    declared_arguments.append(DeclareLaunchArgument('world', default_value='empty.sdf', description='Gazebo World Name'))
    
    declared_arguments.append(DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']))
    declared_arguments.append(DeclareLaunchArgument("ur_type", default_value="ur3"))
    declared_arguments.append(DeclareLaunchArgument("safety_limits", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("safety_pos_margin", default_value="0.15"))
    declared_arguments.append(DeclareLaunchArgument("safety_k_position", default_value="20"))

    # --- DEĞİŞİKLİK 2: TF PREFIX (ÇAKIŞMAYI ÖNLER) ---
    # UR3'ün tüm parçalarına 'ur_' öneki ekler.
    # Böylece UR3'ün 'base_link'i -> 'ur_base_link' olur ve Husky ile çakışmaz.
    declared_arguments.append(DeclareLaunchArgument("tf_prefix", default_value='ur_'))

    # Dosya Yolları
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("husky_ur3_simulation"), "config", "husky_ur3_controllers.yaml"]
            ),
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("husky_ur3_simulation"), "urdf", "mobile_manipulator.urdf.xacro"]
            ),
        )
    )
    
    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true"))
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
            ),
        )
    )

    ld = LaunchDescription(declared_arguments)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld