from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
    DeclareLaunchArgument('launch_rviz',
            default_value='false',
            description='set to true if you want to launch the rviz gui to view the mapping process'
        )
    )

    launch_rviz = LaunchConfiguration('launch_rviz')     


    #lauch the realsense package with all dependencies
    load_realsense = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
                ])
            ])
        )
    
    rviz_config_file = PathJoinSubstitution([FindPackageShare("neobotix_realsense415"), "rviz", "camera.rviz"]) # define path to rviz-config file

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz)
    )

    image_capture = Node(
        package="neobotix_realsense415",
        executable="capture_image_server",
        name="capture_image_server",
    )

    get_aruco_pose = Node(
        package="neobotix_realsense415",
        executable="get_aruco_pose_server",
        name="get_aruco_pose_server",
    )

    nodes_to_start = [
        load_realsense,
        rviz_node,
        get_aruco_pose,
        image_capture
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
   