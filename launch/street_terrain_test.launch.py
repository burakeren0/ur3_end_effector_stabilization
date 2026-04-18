import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    tf_prefix = LaunchConfiguration("tf_prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world = LaunchConfiguration('world')
    
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        LaunchConfiguration("description_file"), " ",
        "safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20 name:=ur ",
        "ur_type:=", ur_type, " tf_prefix:=", tf_prefix, " ", 
        "simulation_controllers:=", LaunchConfiguration("controllers_file"),
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(package="robot_state_publisher", executable="robot_state_publisher", output="both", parameters=[{"use_sim_time": True}, robot_description])
    joint_state_broadcaster_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "-c", "/controller_manager"])
    ur_controller_spawner = Node(package="controller_manager", executable="spawner", arguments=["scaled_joint_trajectory_controller", "-c", "/controller_manager"])
    ur_vel_controller_spawner = Node(package="controller_manager", executable="spawner", arguments=["ur_velocity_controller", "-c", "/controller_manager", "--inactive"])
    ur_eff_controller_spawner = Node(package="controller_manager", executable="spawner", arguments=["ur_effort_controller", "-c", "/controller_manager", "--inactive"])
    husky_controller_spawner = Node(package="controller_manager", executable="spawner", arguments=["diff_drive_base_controller", "-c", "/controller_manager"])

    rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2", output="log", arguments=["-d", LaunchConfiguration("rviz_config_file")], condition=IfCondition(launch_rviz))
    delay_rviz = RegisterEventHandler(event_handler=OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[rviz_node]), condition=IfCondition(launch_rviz))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': [world, ' -r -v 4']}.items(),
    )
    gz_spawn_entity = Node(package="ros_gz_sim", executable="create", output="screen", arguments=["-string", robot_description_content, "-name", "ur3_end_effector_stabilization", "-allow_renaming", "true", "-z", "0.3"])
    gz_sim_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock", "/imu_data@sensor_msgs/msg/Imu[gz.msgs.IMU"], output="screen")

    return [robot_state_publisher_node, gz_sim, gz_spawn_entity, joint_state_broadcaster_spawner, ur_controller_spawner, ur_vel_controller_spawner, ur_eff_controller_spawner, husky_controller_spawner, gz_sim_bridge, delay_rviz]

def generate_launch_description():
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH', '').split(':')]
    gz_sim_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[os.path.join(get_package_share_directory('clearpath_gz'), 'worlds') + ':', ':' + ':'.join(packages_paths)])

    return LaunchDescription([
        gz_sim_resource_path,
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([FindPackageShare("ur3_end_effector_stabilization"), "worlds", "street_world.sdf"])),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument("ur_type", default_value="ur3"),
        DeclareLaunchArgument("tf_prefix", default_value='ur_'),
        DeclareLaunchArgument("controllers_file", default_value=PathJoinSubstitution([FindPackageShare("ur3_end_effector_stabilization"), "config", "husky_ur3_controllers.yaml"])),
        DeclareLaunchArgument("description_file", default_value=PathJoinSubstitution([FindPackageShare("ur3_end_effector_stabilization"), "urdf", "mobile_manipulator.urdf.xacro"])),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config_file", default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "rviz", "view_robot.rviz"])),
        OpaqueFunction(function=launch_setup)
    ])
