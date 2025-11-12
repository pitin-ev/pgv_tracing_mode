from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = FindPackageShare('pgv_tracing_mode')

    default_ctrl_params = PathJoinSubstitution([pkg_share, 'config', 'line_drive.params.yaml'])
    default_sim_params  = PathJoinSubstitution([pkg_share, 'config', 'sim.params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('controller_params', default_value=default_ctrl_params),
        DeclareLaunchArgument('sim_params', default_value=default_sim_params),
        DeclareLaunchArgument('use_relative', default_value='true'),
        DeclareLaunchArgument('relative_goal', default_value='1.5'),
        DeclareLaunchArgument('absolute_goal', default_value='5.0'),
        DeclareLaunchArgument('goal_delay_sec', default_value='1.0'),
        DeclareLaunchArgument('call_align_first', default_value='true'),

        # 1) Dummy PGV Simulator
        Node(
            package='pgv_tracing_mode',
            executable='dummy_pgv_sim',
            namespace=LaunchConfiguration('namespace'),
            name='dummy_pgv_sim',
            output='screen',
            parameters=[LaunchConfiguration('sim_params')],
        ),

        # 2) Line Drive Controller
        Node(
            package='pgv_tracing_mode',
            executable='line_drive',
            namespace=LaunchConfiguration('namespace'),
            name='line_drive',
            output='screen',
            parameters=[
                LaunchConfiguration('controller_params'),
            ],
        ),

        # 3) One-shot Goal Sender
        Node(
            package='pgv_tracing_mode',
            executable='one_shot_goal',
            namespace=LaunchConfiguration('namespace'),
            name='one_shot_goal',
            output='screen',
            parameters=[{
                'use_relative': LaunchConfiguration('use_relative'),
                'relative_goal': LaunchConfiguration('relative_goal'),
                'absolute_goal': LaunchConfiguration('absolute_goal'),
                'goal_delay_sec': LaunchConfiguration('goal_delay_sec'),
                'call_align_first': LaunchConfiguration('call_align_first'),
            }],
        ),
    ])
