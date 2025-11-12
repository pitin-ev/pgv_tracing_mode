from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('pgv_tracing_mode'),
        'config',
        'line_drive.params.yaml'
    ])

    return LaunchDescription([
        # --- common args ---
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML file with ros__parameters'),
        # (옵션) 개별 오버라이드도 함께 지원하고 싶으면 유지
        # DeclareLaunchArgument('holonomic', default_value='true'),
        # DeclareLaunchArgument('pose_topic', default_value='/amr1/bcd_pose'),
        # DeclareLaunchArgument('cmd_topic', default_value='/cmd_vel'),

        Node(
            package='pgv_tracing_mode',
            executable='line_drive',
            namespace=LaunchConfiguration('namespace'),
            name='line_drive',
            output='screen',
            # YAML을 1순위로 넣고, 필요한 소수 파라미터만 launch arg로 덮어쓸 수 있게 dict 추가
            parameters=[
                LaunchConfiguration('params_file'),
                # {
                #     'holonomic': LaunchConfiguration('holonomic'),
                #     'pose_topic': LaunchConfiguration('pose_topic'),
                #     'cmd_topic': LaunchConfiguration('cmd_topic'),
                # }
            ],
        )
    ])
