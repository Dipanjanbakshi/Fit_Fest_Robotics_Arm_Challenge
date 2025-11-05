import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ------------------- Gazebo -------------------
    # ------------------- Gazebo -------------------
    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("fit_description"),
            "launch",
            "gazebo.launch.py"
        )
    ),
    launch_arguments={
        "is_sim": "True",        # if the included launch expects is_sim
        "is_ignition": "False"   # set True if you intend to use Ignition
    }.items()
)


    # ------------------- Controllers -------------------
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fit_controller"),
                "launch",
                "fit_controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- MoveIt -------------------
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fit_moveit"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- Vision Node -------------------
    vision_node = Node(
        package="fit_vision",
        executable="color_detector",
        name="color_detector",
        output="screen"
    )

    # ------------------- MoveIt Color Picker Node -------------------
    color_picker_node = Node(
        package="pymoveit2",
        executable="pick_and_place",
        name="pick_and_place",
        output="screen",
        parameters=[
            {"target_color": "B"}  # {"target_color": "R"}, {"target_color": "G"}
        ]
    )

    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        vision_node,
        # color_picker_node,
    ])
