from launch import  LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_pkg",
            executable="cpp_publisher_node",
            name="temp_node_name",
            parameters=[
                {"frequency" : 300}
            ],
            remappings=[
                ("first_topic", "ft")
            ]
        ),
        Node(
            package="cpp_pkg",
            executable="cpp_publisher_node",
            name="another_temp_name",
            parameters=[
                {"frequency" : 1000}
            ],
            remappings=[
                ("first_topic", "ft")
            ]
        ),
        Node(
            package="cpp_pkg",
            executable="cpp_subscriber_node",
            remappings=[
                ("first_topic", "ft")
            ]
        ),
    ])