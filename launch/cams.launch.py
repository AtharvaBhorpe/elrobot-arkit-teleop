"""Both cameras: wrist (on the gripper) + external (context webcam).

    pixi run cams
    WRIST_DEV=/dev/video0 EXT_DEV=/dev/video2 pixi run cams

Find which /dev/videoN is which with: v4l2-ctl --list-devices  (or trial).
Each USB camera typically claims two /dev/video nodes; use the even one.
"""

import os
import sys
from pathlib import Path

from launch.actions import ExecuteProcess

from launch import LaunchDescription

HERE = Path(__file__).resolve().parent
CAM = "elrobot.nodes.cam_node"


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=[sys.executable, "-m", CAM,
                            "--device", os.environ.get("WRIST_DEV", "/dev/video0"),
                            "--topic", "/wrist_cam/image",
                            "--name", "wrist_cam"], output="screen"),
        ExecuteProcess(cmd=[sys.executable, "-m", CAM,
                            "--device", os.environ.get("EXT_DEV", "/dev/video2"),
                            "--topic", "/ext_cam/image",
                            "--name", "ext_cam"], output="screen"),
    ])
