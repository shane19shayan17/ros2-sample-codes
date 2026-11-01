from launch import LaunchDescription
from launch_ros.actions import Node

from os import path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_root = get_package_share_directory('robot_description')
    
    urdf_path = path.join(
        pkg_root,
        'urdf',
        'test_urdf.urdf'
    )
    rviz_config_path = path.join(
        pkg_root,
        'rviz',
        'r2d2.rviz'
    )
    
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    
    return LaunchDescription([
        Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            parameters = [
                {'robot_description' : robot_description}
            ],
        ),
        Node(
            package = "joint_state_publisher",
            executable = "joint_state_publisher",
        ),
        Node(
            package = "rviz2",
            executable = "rviz2",
            arguments=[
                '-d', rviz_config_path
            ]
        ),
    ])