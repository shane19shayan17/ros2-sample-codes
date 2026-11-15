from launch import LaunchDescription
from launch_ros.actions import Node

from os import path
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    
    pkg_root = get_package_share_directory('robot_description')
    
    urdf_path = path.join(
        pkg_root,
        'urdf',
        'arm.urdf.xacro'
    )
    rviz_config_path = path.join(
        pkg_root,
        'rviz',
        'arm.rviz'
    )
    
    xacro_file = xacro.process_file(urdf_path)
    robot_description = xacro_file.toxml()
    
    return LaunchDescription([
        Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            parameters = [
                {'robot_description' : robot_description}
            ],
        ),
        Node(
            package = "joint_state_publisher_gui",
            executable = "joint_state_publisher_gui",
        ),
        Node(
            package = "rviz2",
            executable = "rviz2",
            arguments=[
                '-d', rviz_config_path
            ]
        ),
    ])