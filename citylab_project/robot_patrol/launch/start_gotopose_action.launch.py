from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import TimerAction

def generate_launch_description():
    pkg_turtlebot3 = get_package_share_directory('turtlebot3_description')
    urdf_file = os.path.join(pkg_turtlebot3, 'urdf', 'turtlebot3_burger.urdf')

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
         parameters=[{'robot_description': open(urdf_file).read()}]

        ),
        
      
              Node(
            package='robot_patrol',
            executable='go_to_pose_exe',
            name='patrol_with_service_node',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/user/ros2_ws/src/citylab_project/robot_patrol/rviz/config.rviz']
        )
    ])
