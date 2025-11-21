import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('prob_rob_labs')
    launch_path = os.path.join(pkg_dir, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        Node(
            package='prob_rob_labs',
            executable='ekf_localization',
            name='ekf_localization',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'landmark_ekf_launch.py')
        ),
        ),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'ekf_pose_error_launch.py')
        ),
        )
    ])
