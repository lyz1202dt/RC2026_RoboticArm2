from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_pack_share = get_package_share_directory("launch_pack")
    
    arm_mujoco_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_pack_share, "launch", "arm_mujoco_sim.launch.py")
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

    data_measure_node=Node(
        package="arm_measure",
        executable="parameter_measure_node",
        output="screen"
    )
    
    return LaunchDescription([
        data_measure_node,
        show_rviz_arg,
        arm_mujoco_sim_launch
    ])
