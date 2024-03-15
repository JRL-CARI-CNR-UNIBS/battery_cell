from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Steps:
# 1. start your robot's MoveIt! stack, e.g. include its moveit_planning_execution.launch.py
# 2. start your tracking system's ROS driver, e.g. include its cameras.launch.py

def launch_setup(context, *args, **kwargs):
    spawn_tag_detector_node = ExecuteProcess(
        cmd=[['ros2 run ',
              'apriltag_ros ',
              'apriltag_node ',
              '--ros-args ',
              '-r image_rect:=/zed/zed_node/rgb/image_rect_color ',
              '-r camera_info:=/zed/zed_node/rgb/camera_info ',
              '--params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml ',
        ]],
        shell=True
    )

    return [spawn_tag_detector_node]


def generate_launch_description():
    launch_arguments = []
    
    launch_arguments.append(
        DeclareLaunchArgument(
            'calibration_type',
            default_value='eye_on_base',
            description='Type of calibration [eye_on_base, eye_in_hand]',
        ),
    )

    launch_arguments.append(
        DeclareLaunchArgument(
            'name',
            default_value='hand_eye_calibration',
            description='Name of the calibration. You can choose any identifier, as long as you use the same for publishing the calibration',
        ),
    )

    launch_arguments.append(
        DeclareLaunchArgument(
            'robot_base_frame',
            default_value='world',
            description='Robot base frame',
        ),
    )

    launch_arguments.append(
        DeclareLaunchArgument(
            'robot_effector_frame',
            default_value='kuka_closed_tip',
            description='Robot effector frame',
        ),
    )

    launch_arguments.append(
        DeclareLaunchArgument(
            'tracking_base_frame',
            default_value='zed_camera_link',
            description='Tracking base frame',
        ),
    )

    launch_arguments.append(
        DeclareLaunchArgument(
            'tracking_marker_frame',
            default_value='handeye_target',
            description='Tracking marker frame',
        ),
    )

    calibrator_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('easy_handeye2'),
                'launch',
                'calibrate.launch.py'
            ])
        ]),
        launch_arguments={
            'calibration_type': LaunchConfiguration('calibration_type'),
            'name': LaunchConfiguration('name'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
        }.items(),
    )
    
    return LaunchDescription(
        launch_arguments + 
        [
            OpaqueFunction(function=launch_setup),
            calibrator_launch_description,
        ]
    )