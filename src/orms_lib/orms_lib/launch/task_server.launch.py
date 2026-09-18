from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sim_mode = LaunchConfiguration('sim_mode')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='False',
            description='Run in simulation mode (bypasses force guards)'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='orms_config_fr3.yaml',
            description='Config file name (relative to orms_lib/config/) or absolute path'
        ),
        Node(
            package='orms_lib',
            executable='task_server_node',
            name='task_server',
            output='screen',
            parameters=[
                {'sim_mode': sim_mode},
                {'config_file': config_file},
            ]
        )
    ])
