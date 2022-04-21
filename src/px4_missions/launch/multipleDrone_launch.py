from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node1=Node(
        package = 'mavros',
        executable = 'mavros_node',
        namespace = 'vhcl0',
        parameters=[
            {"fcu_url": "udp://:14540@"},
            {"tgt_system": 1}
        ]
    )    

    node2=Node(
        package = 'mavros',
        executable = 'mavros_node',
        namespace = 'vhcl1',
        parameters=[
            {"fcu_url": "udp://:14541@"},
            {"tgt_system": 2}
        ]
    )    
 
#    node2=Node(
#        package = 'px4_missions',
#        executable = 'simpleMission',
#    )

    ld.add_action(node1)
    ld.add_action(node2)
    return ld

