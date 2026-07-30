"""Jog the REAL arm with sliders - joint_state_publisher_gui -> driver.

    pixi run jog

The GUI's /joint_states output is remapped onto /joint_command, so slider
moves go through the driver and ALL its safety gates (slew/velocity clamp,
workspace box, sigma floor, URDF limits, deadman - the GUI republishes at
~10 Hz, which keeps the deadman fed). The driver's sync_read publishes the
real /joint_states, so rviz shows the actual arm, not the sliders.

The sliders START AT THE ARM'S CURRENT POSE: launch-time code reads the
bus (the port is free before any node starts) and passes the decoded
angles as the GUI's `zeros` parameters - so appearing does not move the
arm. If that read fails, sliders fall back to zero = URDF neutral and the
arm WILL slew there on GUI start (a warning is printed).
"""

import os
import sys
from pathlib import Path

from launch.actions import DeclareLaunchArgument, ExecuteProcess
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


def _env_args(mapping):
    return [tok for env, flag in mapping
            if (v := os.environ.get(env)) is not None for tok in (flag, v)]


def current_pose_zeros():
    """Read the real arm pose -> {'zeros.<joint>': rad} for the slider GUI."""
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    from elrobot.control.cartesian_ik import ARM_JOINTS, GRIPPER_JOINT
    from elrobot.nodes.elrobot_driver import Converter

    conv = Converter(str(HERE.parent / "calibration" / "urdf_ticks.json"))
    bus = FeetechMotorsBus(
        port=os.environ.get("PORT", "/dev/ttyACM0"),
        motors={n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
                for i, n in enumerate(ARM_JOINTS + [GRIPPER_JOINT], start=1)},
        calibration=None)
    bus.connect(handshake=False)
    try:
        ticks = bus.sync_read("Present_Position",
                              ARM_JOINTS + [GRIPPER_JOINT], normalize=False)
    finally:
        bus.disconnect()
    zeros = {f"zeros.{n}": conv.arm_q(n, ticks[n]) for n in ARM_JOINTS}
    zeros[f"zeros.{GRIPPER_JOINT}"] = conv.grip_q(ticks[GRIPPER_JOINT])
    return zeros


def generate_launch_description():
    try:
        zeros = current_pose_zeros()
        print("jog: sliders seeded from the real arm pose")
    except Exception as e:  # noqa: BLE001 - degrade to neutral with a warning
        zeros = {}
        print(f"jog: could NOT read arm pose ({e}) - sliders start at ZERO, "
              "the arm WILL move to neutral on GUI start!")
    return LaunchDescription([
        # rviz is optional: `pixi run <task> rviz:=false` or RVIZ=0 env
        # (e.g. when Foxglove Studio + `pixi run bridge` is the GUI)
        DeclareLaunchArgument("rviz",
                              default_value=os.environ.get("RVIZ", "true")),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": URDF}]),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(HERE.parent / "config" / "view.rviz")],
             additional_env=QT_SAFE_ENV,
             condition=IfCondition(LaunchConfiguration("rviz"))),
        ExecuteProcess(cmd=[sys.executable, "-m", "elrobot.nodes.elrobot_driver"]
                       + _env_args([("PORT", "--port"),
                                    ("MAX_VEL", "--max-vel"),
                                    ("MAX_ACCEL", "--max-accel"),
                                    ("ACCEL", "--accel")]),
                       output="screen"),
        Node(package="joint_state_publisher_gui",
             executable="joint_state_publisher_gui",
             additional_env=QT_SAFE_ENV,
             parameters=[zeros] if zeros else [],
             remappings=[("/joint_states", "/joint_command")]),
    ])
