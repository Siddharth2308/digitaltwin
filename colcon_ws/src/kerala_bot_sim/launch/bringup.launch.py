"""Live machine-state bringup: RViz twin driven by real position feedback.

Does what kerala_bot_description/display.launch.py does (robot_state_publisher + RViz) but
replaces joint_state_publisher_gui with the state_listener node, so instead of manual
sliders the twin follows the machine's live topic(s).

    ros2 launch kerala_bot_sim bringup.launch.py
    ros2 launch kerala_bot_sim bringup.launch.py config:=/abs/path/machine_state.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory("kerala_bot_description")
    sim_share = get_package_share_directory("kerala_bot_sim")

    xacro_path = os.path.join(desc_share, "urdf", "robot.xacro")
    rviz_config = os.path.join(desc_share, "rviz", "robot.rviz")
    default_cfg = os.path.join(sim_share, "config", "machine_state.yaml")

    robot_description = ParameterValue(Command(["xacro ", xacro_path]), value_type=str)

    cfg_arg = DeclareLaunchArgument(
        "config", default_value=default_cfg,
        description="path to machine_state.yaml")

    return LaunchDescription([
        cfg_arg,
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen",
             parameters=[{"robot_description": robot_description}]),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", rviz_config]),
        Node(package="kerala_bot_sim", executable="state_listener", output="screen",
             parameters=[{"config": LaunchConfiguration("config")}]),
    ])
