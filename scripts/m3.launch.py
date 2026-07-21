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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
URDF = (HERE.parent / "docs" / "urdf_Elrobot_viz.urdf").read_text()

# Conda Qt must not load the SYSTEM ibus input-method plugin (built against
# a different Qt): heap corruption the moment a dialog opens - rviz2 died
# with free(): invalid pointer. 'compose' is Qt's built-in IM, always safe.
QT_SAFE_ENV = {"QT_IM_MODULE": "compose", "QT_IM_MODULES": "",
               "QT_QPA_PLATFORMTHEME": ""}  # no system gtk3 theme plugin

# Tuning knobs, passed as environment variables:
#   ORIENT=0            position-only (phone rotation ignored; often calmer)
#   SCALE=0.4           phone->TCP translation gain          (receiver)
#   SMOOTH=0.35         target EMA alpha, 1=off              (ik)
#   MAX_VEL=0.6         per-joint velocity clamp, rad/s      (driver)
#   ACCEL=40            servo acceleration register          (driver)
#   GRIP_SQUEEZE=40     ticks of squeeze past grasp contact  (driver)
#   GRIP_LOAD_THRESH=200  grasp load threshold, 0.1%/LSB     (driver)
# e.g.:  SCALE=0.3 ORIENT=0 pixi run m3-arm


def _env_args(mapping):
    return [tok for env, flag in mapping
            if (v := os.environ.get(env)) is not None for tok in (flag, v)]


DRIVER_ENV = [("PORT", "--port"),  # e.g. PORT=/dev/ttyACM1 pixi run m3-arm
              ("MAX_VEL", "--max-vel"), ("ACCEL", "--accel"),
              ("GRIP_SQUEEZE", "--grip-squeeze"),
              ("GRIP_LOAD_THRESH", "--grip-load-thresh"),
              ("Z_MIN", "--z-min"), ("R_MAX", "--r-max")]


def generate_launch_description():
    return LaunchDescription([
        # rviz is optional: `pixi run <task> rviz:=false` or RVIZ=0 env
        # (e.g. when Foxglove Studio + `pixi run bridge` is the GUI)
        DeclareLaunchArgument("rviz",
                              default_value=os.environ.get("RVIZ", "true")),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": URDF}]),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(HERE / "view.rviz")],
             additional_env=QT_SAFE_ENV,
             condition=IfCondition(LaunchConfiguration("rviz"))),
        ExecuteProcess(cmd=[sys.executable, str(HERE / "elrobot_driver.py")]
                       + _env_args(DRIVER_ENV),
                       output="screen"),
        ExecuteProcess(cmd=[sys.executable, str(HERE / "ik_node.py"),
                            "--no-sim-state"]
                       + _env_args([("SMOOTH", "--smooth"),
                                    ("FREEZE", "--freeze")]),
                       output="screen"),
        ExecuteProcess(cmd=[sys.executable, str(HERE / "arkit_receiver.py")]
                       + _env_args([("SCALE", "--scale")])
                       + ([] if os.environ.get("ORIENT", "1") != "0"
                          else ["--no-orient"]),
                       output="screen"),
    ])
