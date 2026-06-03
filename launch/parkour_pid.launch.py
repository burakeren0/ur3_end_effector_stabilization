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

# Launch file for the parkour UR3 scenario using a PID joint controller.
# It starts the simulation and then launches the pose stabilization, inverse kinematics,
# and PID controller nodes with delayed sequencing.
# Use this launch file when you want the PID variant of the joint-space controller.

def resolve_world_file(world_arg):
    if os.path.isabs(world_arg) or os.path.dirname(world_arg):
        return world_arg

    package_worlds_dir = os.path.join(
        get_package_share_directory("ur3_end_effector_stabilization"),
        "worlds",
    )
    candidates = [world_arg]
    if not world_arg.endswith((".sdf", ".world")):
        candidates.extend([f"{world_arg}.sdf", f"{world_arg}.world"])

    for candidate in candidates:
        world_path = os.path.join(package_worlds_dir, candidate)
        if os.path.exists(world_path):
            return world_path

    return os.path.join(package_worlds_dir, world_arg)


def launch_setup(context, *args, **kwargs):
    # Collect launch arguments and build nodes from their values.
    # `launch_setup` is called by OpaqueFunction after the argument parsing stage.
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    controllers_file = LaunchConfiguration("controllers_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    description_file = LaunchConfiguration("description_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    ee_pose_logger_enabled = LaunchConfiguration("ee_pose_logger_enabled")  # Read the optional end-effector logger switch.
    ee_pose_log_file = LaunchConfiguration("ee_pose_log_file")  # Read the end-effector CSV output path.
    ee_pose_log_rate = LaunchConfiguration("ee_pose_log_rate")  # Read the end-effector logging frequency.
    ee_pose_imu_topic = LaunchConfiguration("ee_pose_imu_topic")  # Read the base IMU topic used for world-relative RPY logging.

    # Resolve the world file path. If a relative world name is provided,
    # look under this package's `worlds` directory and append .sdf/.world if needed.
    world = resolve_world_file(LaunchConfiguration("world").perform(context))

    # Generate the robot_description parameter from the URDF xacro file.
    # This lets Gazebo and the ROS robot_state_publisher use the same model.
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

    # Publish the robot state from the URDF model to TF and joint_state topics.
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

    # Include the Gazebo simulation launch from ros_gz_sim.
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
                executable="target_pose_full_rpy",
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

    delayed_pid_controller = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="ur3_end_effector_stabilization",
                executable="pid_controller_node",
                output="screen",
            )
        ],
    )

    delayed_end_effector_pose_logger = TimerAction(  # Start the pose logger after TF and target-pose publishers are available.
        period=20.0,  # Match the controller startup delay so logging begins with active control.
        actions=[  # Hold the logger node inside the delayed action list.
            Node(  # Launch the installed Python logger as a ROS 2 node.
                package="ur3_end_effector_stabilization",  # Use this package's installed executable.
                executable="end_effector_pose_target_logger.py",  # Log target/actual XYZ and world-frame RPY.
                output="screen",  # Show logger startup and TF errors in the launch terminal.
                arguments=[  # Forward output, rate, and IMU launch arguments to the logger CLI.
                    "--output-file",  # Select the CSV output path option.
                    ee_pose_log_file,  # Use the configured CSV output path.
                    "--rate",  # Select the logging frequency option.
                    ee_pose_log_rate,  # Use the configured logging frequency.
                    "--imu-topic",  # Select the base IMU topic used for world-relative RPY angles.
                    ee_pose_imu_topic,  # Use the configured base IMU topic.
                ],  # Finish logger CLI arguments.
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],  # Timestamp samples with simulation time.
                condition=IfCondition(ee_pose_logger_enabled),  # Allow disabling automatic logging from the launch command.
            )  # Finish the logger node definition.
        ],  # Finish the delayed logger action list.
    )  # Finish the delayed logger action.

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
        delayed_pid_controller,
        delayed_end_effector_pose_logger,  # Start the end-effector pose logger with the simulation by default.
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
            default_value="hilly_world",
            description=(
                "Gazebo world file. Use a name from this package's worlds directory "
                "(for example bumpy_world), or pass a full path."
            ),
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
    declared_arguments.append(  # Declare whether automatic end-effector pose logging is enabled.
        DeclareLaunchArgument(  # Create the logger enable/disable launch option.
            "ee_pose_logger_enabled",  # Name used by LaunchConfiguration in launch_setup.
            default_value="true",  # Enable pose logging by default.
            choices=["true", "false"],  # Restrict the option to valid boolean strings.
            description="Start the end-effector target/actual pose CSV logger.",  # Explain the option in --show-args output.
        )  # Finish the logger switch argument.
    )  # Finish appending the logger switch argument.
    declared_arguments.append(  # Declare the CSV output path passed to the logger.
        DeclareLaunchArgument(  # Create the logger output-file launch option.
            "ee_pose_log_file",  # Name used by LaunchConfiguration in launch_setup.
            default_value="/home/taylan/ur3_ws/src/ur3_end_effector_stabilization/analysis/end_effector_pose_pid_log.csv",  # Keep PID samples in a controller-specific CSV.
            description="CSV path for end-effector target/actual pose samples.",  # Explain the output path option.
        )  # Finish the logger output-file argument.
    )  # Finish appending the logger output-file argument.
    declared_arguments.append(  # Declare the logger sampling-frequency option.
        DeclareLaunchArgument(  # Create the logger rate launch option.
            "ee_pose_log_rate",  # Name used by LaunchConfiguration in launch_setup.
            default_value="20.0",  # Record twenty target/actual samples per second by default.
            description="End-effector pose logging rate in Hz.",  # Explain the sampling-rate option.
        )  # Finish the logger rate argument.
    )  # Finish appending the logger rate argument.
    declared_arguments.append(  # Declare the base IMU topic used for world-relative end-effector RPY logging.
        DeclareLaunchArgument(  # Create the logger IMU-topic launch option.
            "ee_pose_imu_topic",  # Name used by LaunchConfiguration in launch_setup.
            default_value="/imu_data",  # Use the Gazebo base IMU bridged by this launch file.
            description="Base IMU topic used for world-relative end-effector roll, pitch, and yaw.",  # Explain the world-orientation source.
        )  # Finish the logger IMU-topic argument.
    )  # Finish appending the logger IMU-topic argument.
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
