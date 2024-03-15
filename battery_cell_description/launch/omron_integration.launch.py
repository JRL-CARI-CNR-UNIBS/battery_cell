from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument("use_fake_hardware"),
        DeclareLaunchArgument("robot_description"),
        # DeclareLaunchArgument("robot_description_semantic"),
        DeclareLaunchArgument("ns", default_value="omron"),
        # DeclareLaunchArgument("controller_manager")
    ]

    return LaunchDescription([*launch_args, OpaqueFunction(function=launch_setup)])

def launch_setup(context):
    use_fake_hardware  = LaunchConfiguration("use_fake_hardware")
    robot_description =  LaunchConfiguration("robot_description")
    ns =                 LaunchConfiguration("ns")
    # controller_manager = LaunchConfiguration("controller_manager")

    ros2_controllers_path = PathJoinSubstitution([
        FindPackageShare("battery_cell_description"),
        "config",
        "omron_ros2_controllers.yaml"
    ])

    ld60_params = PathJoinSubstitution([
        FindPackageShare("omron_imm_description"),
        "config",
        "omron_ld60.yaml"
    ])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # name="controller_manager",
        namespace=ns,
        parameters=[ros2_controllers_path, ld60_params, robot_description],
        output="screen",
    )

    controller_manager = f"{ns.perform(context)}/controller_manager"
    
    omron_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
          "joint_state_broadcaster",
          "-c", controller_manager
        ],
      )

    omron_tm12_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-p", ros2_controllers_path,
                    "-c", controller_manager
                    ],
        output="screen"
    )

    support_nodes = Node(
      package="omron_hardware_interface",
      executable="omron_support_nodes",
      namespace=ns,
      parameters=[ld60_params],
      output="screen",
      condition=IfCondition(NotSubstitution(use_fake_hardware))
    )

    omron_state_bcast_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["omron_state_broadcaster",
                 "-c", controller_manager],
      condition=IfCondition(NotSubstitution(use_fake_hardware))
    )

    return [
        ros2_control_node,
        omron_joint_state_broadcaster_spawner,
        omron_tm12_controller_spawner,
        support_nodes,
        omron_state_bcast_spawner
    ]