"""View the Elrobot stick-figure URDF in rviz2 with joint sliders.

    pixi run view

Sliders (joint_state_publisher_gui) -> /joint_states -> robot_state_publisher
-> TF -> rviz2. Later, M2's ik node replaces the sliders on the same topic.
"""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
URDF = (HERE.parent / "docs" / "urdf_Elrobot_viz.urdf").read_text()


def generate_launch_description():
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": URDF}]),
        Node(package="joint_state_publisher_gui",
             executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(HERE / "view.rviz")]),
    ])
