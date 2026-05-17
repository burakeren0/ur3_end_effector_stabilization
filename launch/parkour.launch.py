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
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "simulation_controllers:=",
            controllers_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

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
        arguments=["ur_effort_controller", "-c", "/controller_manager"],
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

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_args_options = [world, " -r -v 4"]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gz_args_options}.items(),
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
            "-allow_renaming",
            "true",
            "-z",
            "0.2",
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

    delayed_diff_drive_teleop = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "gnome-terminal",
                    "--",
                    "bash",
                    "-lc",
                    (
                        "ros2 run teleop_twist_keyboard teleop_twist_keyboard "
                        "--ros-args -p stamped:=true "
                        "--remap cmd_vel:=/diff_drive_base_controller/cmd_vel; "
                        "exec bash"
                    ),
                ],
                output="screen",
            )
        ],
    )

    delayed_target_pose = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="ur3_end_effector_stabilization",
                executable="target_pose_full_rpy",  # Roll, pitch ve yaw korunumu icin eski target_pose yerine yeni dugum kullanilir.
                output="screen",
            )
        ],
    )

    delayed_inverse_kinematics = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="ur3_end_effector_stabilization",
                executable="inverse_kinematics_node",
                output="screen",
            )
        ],
    )

    delayed_computed_torque = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="ur3_end_effector_stabilization",
                executable="computed_torque_node",  # CMakeLists.txt içinde tanımlı gerçek computed torque executable adı.
                output="screen",
            )
        ],
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
        delayed_diff_drive_teleop,
        delayed_target_pose,
        delayed_inverse_kinematics,
        delayed_computed_torque,
    ]


def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory("clearpath_gz")
    packages_paths = [
        os.path.join(p, "share") for p in os.getenv("AMENT_PREFIX_PATH", "").split(":") if p
    ]

    gz_sim_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.path.join(pkg_clearpath_gz, "worlds") + ":",
            ":" + ":".join(packages_paths),
        ],
    )

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur3_end_effector_stabilization"),
                    "worlds",
                    "extreme_disturbance.world",
                ]
            ),
            description="Gazebo world file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"])
    )
    declared_arguments.append(DeclareLaunchArgument("ur_type", default_value="ur3"))
    declared_arguments.append(DeclareLaunchArgument("safety_limits", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("safety_pos_margin", default_value="0.15"))
    declared_arguments.append(DeclareLaunchArgument("safety_k_position", default_value="20"))
    declared_arguments.append(DeclareLaunchArgument("tf_prefix", default_value="ur_"))

    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur3_end_effector_stabilization"),
                    "config",
                    "husky_ur3_controllers.yaml",
                ]
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur3_end_effector_stabilization"),
                    "urdf",
                    "mobile_manipulator.urdf.xacro",
                ]
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
