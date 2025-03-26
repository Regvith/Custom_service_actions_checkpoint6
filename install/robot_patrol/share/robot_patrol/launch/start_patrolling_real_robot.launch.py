from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import TimerAction

def generate_launch_description():
    pkg_turtlebot3 = get_package_share_directory('turtlebot3_description')
    urdf_file = os.path.join(pkg_turtlebot3, 'urdf', 'turtlebot3_burger.urdf')

    pkg_patrol = get_package_share_directory('robot_patrol')

    return LaunchDescription([
       
        Node(
            package='robot_patrol',
            executable='patrol_real_exe',
            name='patrol_node',
            output='screen'
        )

       
    ])
