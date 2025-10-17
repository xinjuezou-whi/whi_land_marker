# Copyright 2025 WheelHub Intelligent
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Input parameters declaration
    robot_name = LaunchConfiguration('robot_name')

    # Declare arguments
    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='',
        description='Robot name'
    )

    # Path to config file
    config_file = PathJoinSubstitution([
        FindPackageShare('whi_land_marker'),
        'config',
        'config.yaml'
    ])

    # Node definition
    start_whi_land_marker_node = Node(
        package='whi_land_marker',
        executable='whi_land_marker_node',
        name='whi_land_marker',
        output='screen',
        parameters=[
            config_file,
        ]
    )

    return LaunchDescription([
        declare_robot_name_arg,
        start_whi_land_marker_node
    ])
