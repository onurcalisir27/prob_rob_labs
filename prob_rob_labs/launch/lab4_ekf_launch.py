import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('prob_rob_labs')
    launch_path = os.path.join(pkg_dir, 'launch')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'ekfodom.rviz') 

    sim_timer_arg = DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation')

    ld = LaunchDescription()

    turtlebot_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_path,  'turtlebot3_empty_launch.py')
        ),
    )

    gt_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'lab4_launch.py')
        ),
    )

    odom_error = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'lab4_odom_error_launch.py')
        ),
    )

    ekf_node = Node(
                package='prob_rob_labs',
                executable='lab4_ekf',
                name='lab4_ekf',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
    
    rviz = Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ]
            )
    
    delay_ekf = TimerAction(
        period=10.0,
        actions=[gt_publisher, ekf_node, rviz, odom_error]
    )
    ld.add_action(sim_timer_arg)
    ld.add_action(turtlebot_launcher)
    ld.add_action(delay_ekf)
    return ld 

