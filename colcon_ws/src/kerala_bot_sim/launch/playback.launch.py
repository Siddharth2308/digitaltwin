"""Play a simulation log into RViz.

Starts robot_state_publisher + RViz (reusing the description package) + the rviz_player
node. Does NOT start joint_state_publisher_gui (rviz_player owns /joint_states here).

    ros2 launch kerala_bot_sim playback.launch.py log:=/abs/path/sim_log.csv
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
    default_map = os.path.join(sim_share, "config", "axis_map.yaml")

    robot_description = ParameterValue(Command(["xacro ", xacro_path]), value_type=str)

    log_arg = DeclareLaunchArgument("log", description="path to sim_log.csv")
    map_arg = DeclareLaunchArgument("axis_map", default_value=default_map)
    rate_arg = DeclareLaunchArgument("rate", default_value="1.0")

    return LaunchDescription([
        log_arg, map_arg, rate_arg,
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen",
             parameters=[{"robot_description": robot_description}]),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", rviz_config]),
        Node(package="kerala_bot_sim", executable="rviz_player", output="screen",
             parameters=[{
                 "log": LaunchConfiguration("log"),
                 "axis_map": LaunchConfiguration("axis_map"),
                 "rate": LaunchConfiguration("rate"),
             }]),
    ])
