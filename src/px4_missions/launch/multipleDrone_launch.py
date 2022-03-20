import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
 
    node1=Node(
        package = 'px4_missions',
        executable = 'simpleMission',
        namespace = 'vhcl0'
    )
    
    node2=Node(
        package = 'px4_missions',
        executable = 'simpleMission',
        namespace = 'vhcl1'
    )
 
    ld.add_action(node1)
    ld.add_action(node2)
    return ld
