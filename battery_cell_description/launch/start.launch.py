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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable, NotSubstitution, OrSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

import os

def launch_setup(context, *args, **kwargs):

    fake = LaunchConfiguration('fake')
    kuka_fake = LaunchConfiguration('kuka_fake')
    linear_axis_fake = LaunchConfiguration('linear_axis_fake')
    comau_fake = LaunchConfiguration('comau_fake')
    rviz_gui = LaunchConfiguration('rviz_gui')
    include_omron = LaunchConfiguration('include_omron')
    launch_trj_loader = LaunchConfiguration('launch_trj_loader')

    robot_description = {
        'robot_description':
        ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name='xacro')]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare('battery_cell_description'),
                        "urdf",
                        "battery_cell_rviz.xacro"]
                    ),
                    " ", "use_fake_hardware:='",             fake.perform(context),"'",
                    " ", "use_kuka_fake_hardware:='",        OrSubstitution(fake.perform(context),kuka_fake.perform(context)),"'",
                    " ", "use_linear_axis_fake_hardware:='", OrSubstitution(fake.perform(context),linear_axis_fake.perform(context)),"'",
                    " ", "use_comau_fake_hardware:='",       OrSubstitution(fake.perform(context),comau_fake.perform(context)),"'",
                    " ", "include_omron:=",                  include_omron,
                ]
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
        arguments=["--ros-args", "--log-level", "info"],
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
                   "/controller_manager",
                   "--inactive"], # start the controller in an INACTIVE state
    )

    comau_jt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["comau_jt_controller",
                   "--controller-manager",
                   "/controller_manager",
                    "--inactive"], # start the controller in an INACTIVE state
    )

    comau_scaled_fjt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["comau_scaled_fjt_controller",
                   "--controller-manager",
                   "/controller_manager"],
                   #"--inactive"], # start the controller in an INACTIVE state
    )

    kuka_scaled_fjt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kuka_scaled_fjt_controller",
                   "--controller-manager",
                   "/controller_manager"],
                   #"--inactive"], # start the controller in an INACTIVE state
    )
    
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

    # # Used just for camera calibration
    # kuka_closed_tip_tf_spawner = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = ['--x', '0.0', '--y', '0.0', '--z', '0.22', '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0', '--frame-id', 'kuka_sensor', '--child-frame-id', 'kuka_closed_tip'],
    # )

    ###########
    ## Omron ##
    ###########

    omron_robot_description = {
        'robot_description':
        ParameterValue(
            Command(
                [PathJoinSubstitution([FindExecutable(name='xacro')]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare('battery_cell_description'),
                        "urdf",
                        "omron_real.xacro"]
                    ),
                    ],
            ),
            value_type=str
        )
    }

    # TODO: sostituisci con pubblicatore solo di robot_description
    omron_robot_description_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="omron",
        parameters=[omron_robot_description],
        remappings=[#("/tf", "/omron/tf"), ("/tf_static","/omron/tf_static"), # intercept tf
                    ("/omron/joint_states", "/joint_states")],
        condition=IfCondition(include_omron),
    )

    ########################
    ## Trj loader actions ##
    ########################
    #Execute bash to merge trajectories files from config/trajectories
    os.system(f"bash {PathJoinSubstitution([FindPackageShare('battery_cell_description'),'config','trajectories','merge_trajectories.bash']).perform(context)} {PathJoinSubstitution([FindPackageShare('battery_cell_description'),'config','trajectories']).perform(context)}")

    trj_loader_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('battery_cell_description'),
                'launch',
                'trajectory_loader_server.launch.py'
            ])
        ]),
        condition=IfCondition(launch_trj_loader),
    )

    sleep_server_node = Node(
        package="btcpp_ros2_samples",
        executable="sleep_server"
                )
   
    ##############
    ## To Start ##
    ##############

    what_to_launch = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        kuka_jt_controller_spawner,
        comau_jt_controller_spawner,
        kuka_scaled_fjt_controller_spawner,
        comau_scaled_fjt_controller_spawner,
        delta_controller_spawner,
        digital_io_controller_spawner,
        # ft_ati_controller_spawner,
        move_group_node,
        delta_utils_node,
        battery_cell_utils_node,
        cameras_tf_spawner,
        omron_robot_description_pub,
        # kuka_closed_tip_tf_spawner,
        trj_loader_launch_description,
        sleep_server_node,
        ]

    return what_to_launch


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument(
        'fake',
        default_value='true'
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'kuka_fake',
        default_value='false'
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'linear_axis_fake',
        default_value='false'
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'comau_fake',
        default_value='false'
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'rviz_gui',
        default_value="true"
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'include_omron',
        description="Include omron imm in the cell, both nodes and robot_description",
        default_value="true"
    ))
    launch_arguments.append(DeclareLaunchArgument(
        'launch_trj_loader',
        description="Launch trajectory loader servers",
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
            # cameras_launch_description,
            OpaqueFunction(function=launch_setup)
        ]
    )
