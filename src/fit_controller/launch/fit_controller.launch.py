import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

# near top of generate_launch_description()
start_controller_manager = LaunchConfiguration("start_controller_manager")
is_sim = LaunchConfiguration("is_sim")

is_sim_arg = DeclareLaunchArgument(
    "is_sim",
    default_value="true"
)

start_controller_manager_arg = DeclareLaunchArgument(
    "start_controller_manager",
    default_value="true",
    description="If true, start ros2_control_node here; otherwise assume another launch starts it"
)


def generate_launch_description():

    is_sim = LaunchConfiguration("is_sim")
    
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("fit_description"),
                    "urdf",
                    "fit.urdf.xacro",
                ),
                " is_sim:=", LaunchConfiguration("is_sim"),
                " is_ignition:=", LaunchConfiguration("is_ignition")
            ]
        ),
        value_type=str,
    )

    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{"robot_description": robot_description,
    #                  "use_sim_time": is_sim}],
    # )

    controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        {"robot_description": robot_description,
         "use_sim_time": is_sim},
        os.path.join(
            get_package_share_directory("fit_controller"),
            "config",
            "fit_controllers.yaml",
        ),
    ],
    condition=IfCondition(start_controller_manager)
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            is_sim_arg,
            start_controller_manager_arg,
            controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )