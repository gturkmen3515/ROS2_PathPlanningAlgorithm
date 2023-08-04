from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('dummy_controller'),
        'config',
        'params.yaml')

    dummy_controller_cmd = Node(
            package='dummy_controller',
            executable='dummy_controller_node',
            name='dummy_controller_node',
            parameters=[{'use_sim_time': True}, params_file],
            output='screen')
    
    ld = LaunchDescription()

    ld.add_action(dummy_controller_cmd)

    return ld