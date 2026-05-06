import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vehicle_env = os.environ.get('VEHICLE_NAME', 'e4')
    config_file = vehicle_env + '_stanley.yaml'
    config_path = os.path.join(
        get_package_share_directory('gem_gnss_control'),
        'config',
        config_file,
    )

    return LaunchDescription([
        Node(
            package='gem_gnss_control',
            executable='stanley',
            name='stanley_node',
            output='screen',
            parameters=[config_path],
        ),
    ])
