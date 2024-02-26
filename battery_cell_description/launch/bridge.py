# Copyright 2023 Áron Svastits
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    print('A01')

    ros_bridge_config = PathJoinSubstitution(
        [FindPackageShare("battery_cell_description"),
         "config",
         "ros_bridge_config.yaml"]
    )

    print('A02')
    bridge_node = Node(
#        name="ros_bridge",
        package="ros1_bridge",
        executable="parameter_bridge",
        arguments=["topics", "services_1_to_2", "services_2_to_1"],
        parameters=[ros_bridge_config],
        output="both",
    )

    print('A03')
    nodes_to_start = [
        bridge_node
    ]
    print('A04')

    return nodes_to_start


def generate_launch_description():
    launch_arguments = []
    print('A00')

    return LaunchDescription(launch_arguments +
               [OpaqueFunction(function=launch_setup)])
