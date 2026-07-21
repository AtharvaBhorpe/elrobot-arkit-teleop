"""View the Elrobot stick-figure URDF in rviz2 with joint sliders.

    pixi run view

Sliders (joint_state_publisher_gui) -> /joint_states -> robot_state_publisher
-> TF -> rviz2. Later, M2's ik node replaces the sliders on the same topic.
"""

import os
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription

HERE = Path(__file__).resolve().parent
URDF = (HERE.parent / "docs" / "urdf_Elrobot_viz.urdf").read_text()

# Conda Qt must not load the SYSTEM ibus input-method plugin (built against
# a different Qt): heap corruption the moment a dialog opens - rviz2 died
# with free(): invalid pointer. 'compose' is Qt's built-in IM, always safe.
QT_SAFE_ENV = {"QT_IM_MODULE": "compose", "QT_IM_MODULES": "",
               "QT_QPA_PLATFORMTHEME": ""}  # no system gtk3 theme plugin


def generate_launch_description():
    return LaunchDescription([
        # rviz is optional: `pixi run <task> rviz:=false` or RVIZ=0 env
        # (e.g. when Foxglove Studio + `pixi run bridge` is the GUI)
        DeclareLaunchArgument("rviz",
                              default_value=os.environ.get("RVIZ", "true")),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": URDF}]),
        Node(package="joint_state_publisher_gui",
             executable="joint_state_publisher_gui",
             additional_env=QT_SAFE_ENV),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(HERE.parent / "config" / "view.rviz")],
             additional_env=QT_SAFE_ENV,
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
