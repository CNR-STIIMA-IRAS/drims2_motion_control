
# Copyright 2024 National Research Council STIIMA
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():
    pkg_dir = get_package_share_directory('easy_motion_behavior_tree')

    motion_server_config_path_cmd = DeclareLaunchArgument(
        'bt_executer_config_path',
        default_value=pkg_dir + '/config/behavior_tree_config.yaml',
        description='Full path to the config file')

    bt_executer_node = Node(
        package='easy_motion_behavior_tree',
        executable='bt_executer_node',
        name='bt_executer_node',
        output='screen',
        parameters=[
            LaunchConfiguration('bt_executer_config_path'),
        ])
    
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(motion_server_config_path_cmd)
    ld.add_action(bt_executer_node)
    return ld