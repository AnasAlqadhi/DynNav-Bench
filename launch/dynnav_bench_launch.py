#!/usr/bin/env python3
"""
DynNav-Bench — Launch File
============================
Starts the Gazebo simulation with the DynNav arena.

The world file already contains:
  - TurtleBot3 Waffle Pi with all sensor plugins (LiDAR, diff_drive, etc.)
  - All dynamic obstacle models with planar_move plugins
  - Static walls and corridor structures

The goal marker is spawned at runtime by the goal_manager node.

Usage:
  ros2 launch dynnav_bench dynnav_bench_launch.py
  ros2 launch dynnav_bench dynnav_bench_launch.py gui:=true    # with Gazebo GUI
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("dynnav_bench")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    tb3_launch_dir = os.path.join(tb3_gazebo_dir, "launch")

    world_file = os.path.join(pkg_dir, "worlds", "dynnav_arena.world")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    gui = LaunchConfiguration("gui", default="false")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false",
                              description="Launch Gazebo GUI (gzclient)"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle_pi"),

        # Gazebo server (world file contains the robot)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")),
            launch_arguments={
                "world":   world_file,
                "verbose": "false",
                "pause":   "false",
            }.items(),
        ),

        # Gazebo GUI (optional — disabled by default for headless training)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")),
            condition=IfCondition(gui),
        ),

        # Robot state publisher (URDF -> TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, "robot_state_publisher.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
    ])
