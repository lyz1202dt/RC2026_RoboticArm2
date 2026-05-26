from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_pack_share = get_package_share_directory("launch_pack")

    arm_task_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_pack_share, "launch", "arm_task_sim.launch.py")
        ),
        launch_arguments={
            'show_rviz': LaunchConfiguration('show_rviz')
        }.items()
    )

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Whether to start RViz2 together with simulation",
    )
    show_gui_arg = DeclareLaunchArgument(
        "show_gui",
        default_value="true",
        description="Whether to start move_kfs_test GUI",
    )

    move_kfs_test = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="arm_task",
                executable="move_kfs_test",
                output="screen",
                condition=IfCondition(LaunchConfiguration("show_gui")),
            )
        ],
    )

    return LaunchDescription([
        show_rviz_arg,
        show_gui_arg,
        arm_task_sim_launch,
        move_kfs_test,
    ])