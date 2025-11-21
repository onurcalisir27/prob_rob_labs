import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('prob_rob_labs')
    launch_path = os.path.join(pkg_dir, 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),



        # Error publisher
        Node(
            package='prob_rob_labs',
            executable='lab4_odom_error',
            name='lab4_odom_error',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
