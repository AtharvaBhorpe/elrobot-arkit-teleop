"""Jog the REAL arm with sliders - joint_state_publisher_gui -> driver.

    pixi run jog

The GUI's /joint_states output is remapped onto /joint_command, so slider
moves go through the driver and ALL its safety gates (slew/velocity clamp,
workspace box, sigma floor, URDF limits, deadman - the GUI republishes at
~10 Hz, which keeps the deadman fed). The driver's sync_read publishes the
real /joint_states, so rviz shows the actual arm, not the sliders.

STARTUP MOVE WARNING: the sliders initialize at ZERO = URDF neutral. The
moment the GUI appears, the driver will slew the arm from wherever it is
to the neutral pose (velocity-clamped, workspace-gated, but it WILL move).
Clear the arm's space before launching.
"""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
URDF = (HERE.parent / "docs" / "urdf_Elrobot_viz.urdf").read_text()


def generate_launch_description():
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": URDF}]),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(HERE / "view.rviz")]),
        ExecuteProcess(cmd=[sys.executable, str(HERE / "elrobot_driver.py")],
                       output="screen"),
        Node(package="joint_state_publisher_gui",
             executable="joint_state_publisher_gui",
             remappings=[("/joint_states", "/joint_command")]),
    ])
