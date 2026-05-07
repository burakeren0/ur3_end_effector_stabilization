import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
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
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    controllers_file = LaunchConfiguration("controllers_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    description_file = LaunchConfiguration("description_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    world = LaunchConfiguration("world")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "safety_limits:=", safety_limits, " ",
            "safety_pos_margin:=", safety_pos_margin, " ",
            "safety_k_position:=", safety_k_position, " ",
            "name:=ur ",
            "ur_type:=", ur_type, " ",
            "tf_prefix:=", tf_prefix, " ",
            "simulation_controllers:=", controllers_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    ur_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/controller_manager", "--inactive"],
    )
    ur_vel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_velocity_controller", "-c", "/controller_manager", "--inactive"],
    )
    ur_eff_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_effort_controller", "-c", "/controller_manager"],
    )
    husky_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller", "-c", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": [world, " -r -v 4"]}.items(),
    )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "mobile_manipulator",
            "-z",
            "0.3",
        ],
    )
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu_data@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen",
    )

    # --- 5 saniye sonra Teleop başlat (Terminalde) ---
    delayed_teleop = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "gnome-terminal",
                    "--",
                    "bash",
                    "-lc",
                    (
                        "source /opt/ros/jazzy/setup.bash && "
                        "source /home/burak/ur3_ws/install/setup.bash && "
                        "ros2 run teleop_twist_keyboard teleop_twist_keyboard "
                        "--ros-args -p stamped:=true --remap "
                        "cmd_vel:=/diff_drive_base_controller/cmd_vel; "
                        "exec bash"
                    ),
                ],
                output="screen",
            )
        ]
    )

    # --- 10 saniye sonra Target Pose başlat ---
    delayed_target_pose = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="husky_ur3_simulation",
                executable="target_pose",
                output="screen",
            )
        ]
    )

    # --- 15 saniye sonra Inverse Kinematics başlat ---
    delayed_inverse_kinematics = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="husky_ur3_simulation",
                executable="inverse_kinematics_node",
                output="screen",
            )
        ]
    )

    # --- 20 saniye sonra Computed Torque başlat ---
    delayed_computed_torque = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="husky_ur3_simulation",
                executable="computed_torque_cpp",
                output="screen",
            )
        ]
    )

    return [
        robot_state_publisher_node,
        gz_sim,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        ur_controller_spawner,
        ur_vel_controller_spawner,
        ur_eff_controller_spawner,
        husky_controller_spawner,
        gz_sim_bridge,
        delayed_teleop,
        delayed_target_pose,
        delayed_inverse_kinematics,
        delayed_computed_torque,
        delay_rviz,
    ]

def generate_launch_description():
    packages_paths = [
        os.path.join(p, "share")
        for p in os.getenv("AMENT_PREFIX_PATH", "").split(":")
        if p
    ]
    gz_sim_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.path.join(get_package_share_directory("clearpath_gz"), "worlds") + ":",
            ":" + ":".join(packages_paths),
        ],
    )

    return LaunchDescription([
        gz_sim_resource_path,
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("husky_ur3_simulation"), "worlds", "hilly_world.sdf"]
            ),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("ur_type", default_value="ur3"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("tf_prefix", default_value="ur_"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("husky_ur3_simulation"), "config", "husky_ur3_controllers.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("husky_ur3_simulation"), "urdf", "mobile_manipulator.urdf.xacro"]
            ),
        ),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
