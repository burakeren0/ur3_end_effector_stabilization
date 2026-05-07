from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 1. İlk olarak dünyayı ve robotu başlatan launch dosyası
    hilly_terrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('husky_ur3_simulation'),
                'launch',
                'hilly_terrain_test.launch.py'
            ])
        )
    )

    # 2. Teleop Twist Keyboard (Klavye girdisi alabilmesi için ayrı bir terminalde başlatıyoruz)
    teleop_cmd = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard', '--ros-args', '-p', 'stamped:=true', '--remap', 'cmd_vel:=/diff_drive_base_controller/cmd_vel'],
        output='screen'
    )

    # 3. Target Pose Node
    target_pose_node = Node(
        package='husky_ur3_simulation',
        executable='target_pose',
        output='screen'
    )

    # 4. Inverse Kinematics Node
    inverse_kinematics_node = Node(
        package='husky_ur3_simulation',
        executable='inverse_kinematics_node',
        output='screen'
    )

    # 5. Computed Torque CPP Node
    computed_torque_node = Node(
        package='husky_ur3_simulation',
        executable='computed_torque_cpp',
        output='screen'
    )

    return LaunchDescription([
        hilly_terrain_launch,
        TimerAction(period=5.0, actions=[teleop_cmd]),
        TimerAction(period=10.0, actions=[target_pose_node]),
        TimerAction(period=15.0, actions=[inverse_kinematics_node]),
        TimerAction(period=20.0, actions=[computed_torque_node]),
    ])
