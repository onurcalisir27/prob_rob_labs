import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gt_publisher_path = os.path.join(get_package_share_directory(
        'prob_rob_labs'), 'launch')
    
    gt_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gt_publisher_path, 'lab4_launch.py')
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),

        gt_publisher,
        
        Node(
            package='prob_rob_labs',
            executable='lab4_ekf',
            name='lab4_ekf',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
