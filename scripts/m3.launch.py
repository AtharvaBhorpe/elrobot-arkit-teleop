"""M3 — phone drives the REAL arm. First powered milestone.

    pixi run m3-arm

iPhone -> arkit_receiver -> /target_pose -> ik (--no-sim-state, seeded from
the real arm) -> /joint_command -> elrobot_driver -> serial -> arm.
The driver's sync_read publishes /joint_states, so rviz shows the REAL arm.
"""

import os
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
        ExecuteProcess(cmd=[sys.executable, str(HERE / "ik_node.py"),
                            "--no-sim-state"], output="screen"),
        # ORIENT=0 pixi run m3-arm -> position-only mode: TCP orientation
        # frozen at clutch engage, phone rotation ignored. Often feels far
        # more controlled; the Franka project's v1 shipped this way.
        ExecuteProcess(cmd=[sys.executable, str(HERE / "arkit_receiver.py")]
                       + ([] if os.environ.get("ORIENT", "1") != "0"
                          else ["--no-orient"]),
                       output="screen"),
    ])
