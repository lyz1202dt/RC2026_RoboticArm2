from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    arm_share = get_package_share_directory("arm")
    launch_pack_share = get_package_share_directory("launch_pack")

    urdf_path = os.path.join(arm_share, "model", "robotic_arm.urdf")
    controller_yaml = os.path.join(launch_pack_share, "config", "ros2_controller.yaml")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")
    default_mjcf = os.path.join(arm_share, "model", "scene.xml")

    with open(urdf_path, "r", encoding="utf-8") as inf:
        robot_desc = inf.read()

    mjcf_path_arg = DeclareLaunchArgument(
        "mjcf_path",
        default_value=default_mjcf,
        description="Path to the robotic arm MJCF/scene XML used by MuJoCo",
    )

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Whether to start RViz2 together with measurement simulation",
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
            {"show_gui": True},
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

    parameter_measure = Node(
        package="parameter_measure",
        executable="parameter_measure_node",
        output="screen",
        parameters=[
            {
                "model_path": urdf_path,
                "joint_state_topic": "myjoints_state",
                "joint_target_topic": "myjoints_target",
            }
        ],
    )

    load_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="MuJoCo started, spawning dog_controller"),
                joint_controller,
            ],
        )
    )

    return LaunchDescription(
        [
            mjcf_path_arg,
            show_rviz_arg,
            robot_state_pub,
            mujoco,
            load_controller,
            parameter_measure,
        ]
    )
