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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
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
        .robot_description(file_path="config/battery_cell.urdf.xacro", mappings={"use_fake_hardware":fake},)
        .robot_description_semantic(file_path="config/battery_cell.srdf")
        # .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .planning_pipelines(
            "ompl", pipelines=["ompl"]
        )
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
        output="both",
        # parameters=[controller_config],
        # remappings=[("/controller_manager/robot description", "/robot_description")],
        parameters=[robot_description, controller_config],
        # arguments=["--ros-args", "--log-level", "debug"],
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
        output="both",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics
        ],
        condition=IfCondition(rviz_gui),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    delta_utils_node = Node(
        name="delta_utils",
        package="ethercat_utils",
        executable="cia402_slave_manager",
        parameters=[ethercat_utils_config]
    )

    battery_cell_utils_node = Node(
        name="battery_cell_utils",
        package="ethercat_utils",
        executable="battery_cell_utils_manager",
        parameters=[ethercat_utils_config],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager",
                   "/controller_manager"],
    )

    kuka_jt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kuka_jt_controller",
                   "--controller-manager",
                   "/controller_manager",]
                #    "--inactive"], # start the controller in an INACTIVE state
    )

    # comau_jt_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["comau_jt_controller",
    #                "--controller-manager",
    #                "/controller_manager",]
    #             #    "--inactive"], # start the controller in an INACTIVE state
    # )
    
    delta_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["delta_controller",
                   "--controller-manager",
                   "/controller_manager"],
    )
    digital_io_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["digital_io_controller",
                   "--controller-manager",
                   "/controller_manager"],
    )
    ft_ati_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ft_ati_controller",
                   "--controller-manager",
                   "/controller_manager"],
    )

    cameras_tf_spawner = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '1.086547', '--y', '0.676920', '--z', '1.546772', '--qx', '0.037883', '--qy', '0.035933', '--qz', '-0.692783', '--qw', '0.719253', '--frame-id', 'world', '--child-frame-id', 'zed_camera_link'],
    )

    closed_tip_tf_spawner = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0.0', '--y', '0.0', '--z', '0.22', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0', '--frame-id', 'kuka_sensor', '--child-frame-id', 'kuka_closed_tip'],
    )

    # map_tf_spawner = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = ['--x', '0.0', '--y', '0.0', '--z', '0.0', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0', '--frame-id', 'world', '--child-frame-id', 'omron/map'],
    # )

    # omron_base_link_tf_spawner = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = ['--x', '0.0', '--y', '0.0', '--z', '0.0', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0', '--frame-id', 'omron/map', '--child-frame-id', 'omron/base_link'],
    # )

    omron_utils_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("battery_cell_description"),
                "launch",
                "omron_integration.launch.py"
            ])
        ]),
        launch_arguments={
            'use_fake_hardware': fake,
            # 'robot_description': moveit_config.robot_description,
        }.items()
    )

    nodes_to_start = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        kuka_jt_controller_spawner,
        # comau_jt_controller_spawner,
        # delta_controller_spawner,
        # digital_io_controller_spawner,
        # ft_ati_controller_spawner,
        move_group_node,
        delta_utils_node,
        battery_cell_utils_node,
        cameras_tf_spawner,
        closed_tip_tf_spawner,
        omron_utils_launch_description,
        # map_tf_spawner,
        # omron_base_link_tf_spawner,
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

    cameras_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2',
            'publish_tf': 'false',
        }.items()
    )


    return LaunchDescription(
        launch_arguments +
        [
            cameras_launch_description,
            # omron_utils_launch_description,
            OpaqueFunction(function=launch_setup)
        ]
    )
