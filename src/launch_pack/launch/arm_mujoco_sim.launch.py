from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    arm_share = get_package_share_directory("arm")
    launch_pack_share = get_package_share_directory("launch_pack")

    urdf_path = os.path.join(arm_share, "model", "arm4.urdf")
    controller_yaml = os.path.join(launch_pack_share, "config", "ros2_controller.yaml")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")

    with open(urdf_path, "r", encoding="utf-8") as inf:
        robot_desc = inf.read()

    default_mjcf = os.path.join(arm_share, "model", "scene.xml")

    mjcf_path_arg = DeclareLaunchArgument(
        "mjcf_path",
        default_value=default_mjcf,
        description="Path to the robotic arm MJCF/scene XML used by MuJoCo",
    )

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

    start_arm_calc_arg = DeclareLaunchArgument(
        "start_arm_calc",
        default_value="false",
        description="Whether to start arm_calc together with the MuJoCo driver-equivalent controller",
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            {"robot_description": robot_desc},
            controller_yaml,
            {"simulation_frequency": 500.0},
            {"real_time_factor": 1.0},
            {"robot_model_path": LaunchConfiguration("mjcf_path")},
            {"show_gui": ParameterValue(LaunchConfiguration("show_gui"), value_type=bool)},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dog_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_calc = Node(
        package="arm_calc",
        executable="arm_calc",
        condition=IfCondition(LaunchConfiguration("start_arm_calc")),
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
    )

    load_controller = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="Spawning dog_controller after MuJoCo setup delay"),
            joint_controller,
        ],
    )

    return LaunchDescription([
        mjcf_path_arg,
        show_rviz_arg,
        show_gui_arg,
        start_arm_calc_arg,
        robot_state_pub,
        mujoco,
        load_controller,
        arm_calc,
        rviz2,
    ])
