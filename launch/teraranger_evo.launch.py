from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('teraranger_evo')
    
    # Define the default config file
    default_config = os.path.join(pkg_dir, 'config', 'teraranger_params.yaml')
    
    # Declare the config file as a launch argument
    config_file = LaunchConfiguration('config_file')
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the parameter file for the TeraRanger Evo node'
    )
    
    # Create the node
    teraranger_node = Node(
        package='teraranger_evo',
        executable='teraranger_evo_node',
        name='teraranger_evo_node',
        parameters=[config_file],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        declare_config_file,
        teraranger_node
    ])
