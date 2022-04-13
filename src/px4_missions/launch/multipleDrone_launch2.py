from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    node2=Node(
        package = 'mavros',
        executable = 'mavros_node',
        #namespace = 'vhcl0/mavros',
        parameters=[
            {"fcu_url": "udp://:14540@"},
            {"tgt_system": 1}
        ]
        #parameters = [os.path.join(get_package_share_directory('px4_missions'), 'params', 'mavros1_param.yaml')]

    )    
 
    ld.add_action(node2)
    return ld

