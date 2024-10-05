import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to your configuration file
    config_file_path = os.path.join(
        get_package_share_directory('swarm_connect'),  # Your package name
        'cfg',
        'cfg1_battery.cfg'  # Configuration file
    )

    # Define the node
    return LaunchDescription([
        Node(
            package='swarm_connect',  # Your package name
            executable='connectivity_control',  # Your node executable
            name='connectivity_control_node',  # Node name
            parameters=[{'config': config_file_path}]  # Passing the config file
        )
    ])