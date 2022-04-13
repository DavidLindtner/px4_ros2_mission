from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node1=Node(
        package = 'px4_missions',
        executable = 'simpleMission',
    )

    ld.add_action(node1)
    return ld

