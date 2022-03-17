import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('px4_missions'),
        'params',
        'mission2.yaml'
        )
        
    node1=Node(
        package = 'px4_missions',
        executable = 'supervisedMission',
    )
    node2=Node(
        package = 'px4_missions',
        executable = 'missionSupervisor',
        parameters = [config]
    )
    
    ld.add_action(node1)
    ld.add_action(node2)
    return ld
