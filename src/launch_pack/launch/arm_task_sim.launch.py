from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_pack_share = get_package_share_directory("launch_pack")

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Whether to start RViz2 together with MuJoCo simulation",
    )

    show_gui_arg = DeclareLaunchArgument(
        "show_gui",
        default_value="true",
        description="Whether to show the MuJoCo simulator window",
    )

    arm_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_pack_share, "launch", "arm_mujoco_sim.launch.py")
        ),
        launch_arguments={
            "show_rviz": LaunchConfiguration("show_rviz"),
            "show_gui": LaunchConfiguration("show_gui"),
        }.items(),
    )

    arm_ctrl = Node(
        package="arm_calc",
        executable="arm_calc",
        output="screen",
    )

    arm_task = Node(
        package="arm_task",
        executable="arm_task",
        output="screen",
    )

    static_tf_camera_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "-0.05", "0.05", "0.0",
            "0.0", "0.0", "0.0", "1.0",
            "left4",
            "camera_left_link",
        ],
        output="screen",
    )

    static_tf_camera_right = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "-0.05", "0.0", "0.05",
            "0.0", "0.0", "0.0", "1.0",
            "right4",
            "camera_right_link",
        ],
        output="screen",
    )

    return LaunchDescription([
        show_rviz_arg,
        show_gui_arg,
        arm_sim,
        arm_ctrl,
        static_tf_camera_left,
        static_tf_camera_right,
        arm_task,
    ])
