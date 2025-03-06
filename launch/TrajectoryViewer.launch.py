import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config_file = os.path.join(
        get_package_share_directory('trajectory_analyzer'), 
        'rviz',
        'trajectory_reader_view.rviz' 
    )

    return LaunchDescription([
        DeclareLaunchArgument('trajectory_file', default_value='default_trajectory.csv', description='The CSV file name'),
        Node(
            package='trajectory_analyzer', 
            executable='trajectory_reader', 
            name='trajectory_reader',
            output='screen',
            parameters=[{'trajectory_file': LaunchConfiguration('trajectory_file')}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_file]
        )
    ])
