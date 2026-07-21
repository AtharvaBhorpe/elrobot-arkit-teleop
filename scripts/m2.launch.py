"""M2 — phone drives the model in rviz2, no hardware.

    pixi run m2

iPhone (ZIG SIM PRO, UDP :50000) -> arkit_receiver -> /target_pose -> ik
-> /joint_states -> robot_state_publisher -> rviz2.

Same viewer as `pixi run view`, with the ik node in place of the sliders.
"""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

HERE = Path(__file__).resolve().parent
URDF = (HERE.parent / "docs" / "urdf_Elrobot_viz.urdf").read_text()


def generate_launch_description():
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": URDF}]),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(HERE / "view.rviz")]),
        ExecuteProcess(cmd=[sys.executable, str(HERE / "ik_node.py")],
                       output="screen"),
        ExecuteProcess(cmd=[sys.executable, str(HERE / "arkit_receiver.py")],
                       output="screen"),
    ])
