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

    fake = LaunchConfiguration('fake')
    rviz_gui = LaunchConfiguration('rviz_gui')

    robot_description = {
        'robot_description':
        ParameterValue(
           Command(
               [PathJoinSubstitution([FindExecutable(name='xacro')]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare('battery_cell_description'),
                     "urdf",
                     "battery_cell_rviz.xacro"]
                ),
                " ",
                "use_fake_hardware:='",
                fake.perform(context),
                "'",]
           ),
           value_type=str
        )
    }

    moveit_config = (
        MoveItConfigsBuilder("battery_cell")
        .robot_description(file_path="config/battery_cell.urdf.xacro")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .robot_description_semantic(file_path="config/battery_cell.srdf")
        .planning_pipelines("ompl", pipelines=["ompl",
                            "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    controller_config = PathJoinSubstitution(
        [FindPackageShare("battery_cell_description"),
         "config",
         "ros2_controller_config.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("battery_cell_moveit_config"),
         "config",
         "moveit.rviz"]
    )

    ethercat_utils_config = PathJoinSubstitution(
                                [FindPackageShare('battery_cell_description'),
                                 "config",
                                 "ethercat_utils_config.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics
        ],
        condition=IfCondition(rviz_gui),
    )

    delta_utils_node = Node(
        name="delta_utils",
        package="ethercat_utils",
        executable="cia402_slave_manager",
        parameters=[ethercat_utils_config]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager",
                   "/controller_manager"],
    )

    structure_jt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["structure_jt_controller",
                   "-c",
                   "/controller_manager",
                   "--stopped"],
    )

    move_group_node = Node(
           package="moveit_ros_move_group",
           executable="move_group",
           output="screen",
           parameters=[moveit_config.to_dict()],
           arguments=["--ros-args", "--log-level", "info"],
    )

    delta_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["delta_controller",
                   "-c",
                   "/controller_manager"],
    )
    digital_io_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["digital_io_controller",
                   "-c",
                   "/controller_manager"],
    )
    ft_ati_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ft_ati_controller",
                   "-c",
                   "/controller_manager"],
    )

    nodes_to_start = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        structure_jt_controller_spawner,
        delta_controller_spawner,
        digital_io_controller_spawner,
        ft_ati_controller_spawner,
        move_group_node,
        delta_utils_node
        ]

    return nodes_to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument(
        'fake',
        default_value='true'
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'rviz_gui',
        default_value="true"
    ))

    return LaunchDescription(launch_arguments +
               [OpaqueFunction(function=launch_setup)])
