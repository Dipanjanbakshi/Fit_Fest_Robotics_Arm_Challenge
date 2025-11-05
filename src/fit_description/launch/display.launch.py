import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    fit_description_dir = get_package_share_directory('fit_description')
    model_arg= DeclareLaunchArgument(
        name='model',default_value=os.path.join(fit_description_dir,'urdf','fit.urdf.xacro'),
        description='Absolute path to robot urdf file') 
    ros_distribution = os.environ.get('ROS_DISTRO')

    robot_desc=ParameterValue(Command(['xacro ',LaunchConfiguration('model')]),value_type=str)  
    
    robot_state_publisher_Node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]) 
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'robot_description': robot_desc}])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(fit_description_dir, 'rviz', 'display.rviz')],
       )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_Node,
        joint_state_publisher_gui_node,
        rviz_node
    ])