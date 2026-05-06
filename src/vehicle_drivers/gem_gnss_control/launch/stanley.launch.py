"""Launch the Stanley path-tracking node.

Two launch arguments:
  vehicle_name    — picks `${vehicle_name}_stanley.yaml` from the
                    package share dir.  Defaults to the `VEHICLE_NAME`
                    env var if set, else 'e4'.
  obstacles_yaml  — overrides the `obstacles_yaml` parameter from the
                    yaml file.  Pass an empty string to DISABLE the
                    sampling replanner (pure trajectory tracking, no
                    obstacle avoidance).

Examples:
  ros2 launch gem_gnss_control stanley.launch.py
      → e4 + obstacles_lane2.yaml (replanner ON)

  ros2 launch gem_gnss_control stanley.launch.py obstacles_yaml:=''
      → e4 + replanner OFF (pure trajectory tracking)

  ros2 launch gem_gnss_control stanley.launch.py vehicle_name:=e2
      → e2_stanley.yaml + whatever obstacles_yaml that file specifies
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_vehicle = os.environ.get('VEHICLE_NAME', 'e4')

    vehicle_arg = DeclareLaunchArgument(
        'vehicle_name', default_value=default_vehicle,
        description="Picks ${vehicle_name}_stanley.yaml from share/config")
    obstacles_arg = DeclareLaunchArgument(
        'obstacles_yaml', default_value='__from_yaml__',
        description="Override the obstacles_yaml param. "
                    "Use '' to disable the replanner; "
                    "leave as '__from_yaml__' to use the value in the config file.")

    config_path = [
        get_package_share_directory('gem_gnss_control'), '/config/',
        LaunchConfiguration('vehicle_name'), '_stanley.yaml',
    ]

    # If the user explicitly set obstacles_yaml on the command line, pass
    # it through as a parameter override.  Otherwise the yaml file's
    # value wins.  We emit an OpaqueFunction so we can inspect the
    # runtime value of LaunchConfiguration.
    from launch.actions import OpaqueFunction

    def _build_node(context, *args, **kwargs):
        obs_override = LaunchConfiguration('obstacles_yaml').perform(context)
        params = [{'__path': ''.join(c.perform(context) if hasattr(c, 'perform') else c
                                     for c in config_path)}]
        # Build the config path as a string and pass it as a yaml file.
        cfg = ''.join(c.perform(context) if hasattr(c, 'perform') else c
                      for c in config_path)
        node_params = [cfg]
        if obs_override != '__from_yaml__':
            node_params.append({'obstacles_yaml': obs_override})
        return [Node(
            package='gem_gnss_control',
            executable='stanley',
            name='stanley_node',
            output='screen',
            parameters=node_params,
        )]

    return LaunchDescription([
        vehicle_arg,
        obstacles_arg,
        OpaqueFunction(function=_build_node),
    ])
