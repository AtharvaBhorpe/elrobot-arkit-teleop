"""Damped-least-squares Cartesian servo IK for the Elrobot arm.

Ported from franka-isaac-arkit-teleop's CartesianServoIK with the robot
swapped: URDF from docs/, TCP frame Gripper_Base_v1_1, 7 arm DoFs
(rev_motor_01..07 occupy q[0:7]; rev_motor_08 and the two jaw DoFs follow and
are never servoed).

    dq = J^T (J J^T + lambda^2 I)^-1 * twist

with singularity-adaptive lambda (the spec's verified finding: 13.9% of the
workspace has sigma_min < 0.01, so adaptive damping is load-bearing),
joint-velocity clamping, and URDF joint-limit clamping.

Self-test (no ROS): pixi run python scripts/cartesian_ik.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin

URDF = str(Path(__file__).resolve().parent.parent / "docs" / "urdf_Elrobot.urdf")
TCP_FRAME = "Gripper_Base_v1_1"
ARM_JOINTS = [f"rev_motor_{i:02d}" for i in range(1, 8)]
GRIPPER_JOINT = "rev_motor_08"
# rev_motor_08 URDF range is [0, 2.2028]; exact tick mapping is the driver's
# job (calibration/urdf_ticks.json). These are viz/command targets only.
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 2.0


def _clamp_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    n = np.linalg.norm(v)
    return v * (max_norm / n) if n > max_norm else v


class CartesianServoIK:
    """Resolved-rate Cartesian servo for the fixed-base Elrobot arm."""

    def __init__(
        self,
        urdf_path: str = URDF,
        ee_frame: str = TCP_FRAME,
        n_arm: int = 7,
        kp_lin: float = 2.0,
        kp_ang: float = 2.0,
        max_lin_vel: float = 0.2,     # m/s — half the Franka's; 0.42 m reach
        max_ang_vel: float = 0.9,
        max_joint_vel: float = 2.0,
        damping: float = 1e-3,
        sing_threshold: float = 1e-2,
        max_sing_boost: float = 10.0,
        lin_tol: float = 1e-3,
        ang_tol: float = 5e-3,
    ) -> None:
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        if not self.model.existFrame(ee_frame):
            raise ValueError(f"EE frame '{ee_frame}' not in {urdf_path}")
        self.ee_id = self.model.getFrameId(ee_frame)
        self.n_arm = n_arm
        # Verified: rev_motor_01..07 occupy q[0:7] in URDF declaration order.
        assert [self.model.names[j] for j in range(1, 1 + n_arm)] == ARM_JOINTS
        self.q = pin.neutral(self.model)
        self.q_min = self.model.lowerPositionLimit
        self.q_max = self.model.upperPositionLimit

        self.kp_lin, self.kp_ang = kp_lin, kp_ang
        self.max_lin_vel, self.max_ang_vel = max_lin_vel, max_ang_vel
        self.max_joint_vel = max_joint_vel
        self.damping = damping
        self.sing_threshold, self.max_sing_boost = sing_threshold, max_sing_boost
        self.lin_tol, self.ang_tol = lin_tol, ang_tol

    def set_q(self, q) -> None:
        q = np.asarray(q, float)
        if q.shape[0] != self.model.nq:
            raise ValueError(f"expected nq={self.model.nq}, got {q.shape[0]}")
        self.q = q.copy()

    def arm_q(self) -> np.ndarray:
        return self.q[: self.n_arm].copy()

    def ee_pose(self) -> pin.SE3:
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacement(self.model, self.data, self.ee_id)
        return self.data.oMf[self.ee_id].copy()  # copy: oMf is a live buffer

    def servo(self, target: pin.SE3, dt: float) -> bool:
        """One tick toward `target`; True when within tolerance."""
        current = self.ee_pose()
        err = pin.log6(current.actInv(target)).vector
        lin_err, ang_err = err[:3], err[3:]

        twist = np.concatenate([
            _clamp_norm(self.kp_lin * lin_err, self.max_lin_vel),
            _clamp_norm(self.kp_ang * ang_err, self.max_ang_vel),
        ])

        J = pin.computeFrameJacobian(self.model, self.data, self.q,
                                     self.ee_id, pin.LOCAL)
        Ja = J[:, : self.n_arm]

        manip = float(np.sqrt(max(np.linalg.det(Ja @ Ja.T), 0.0)))
        damp = self.damping
        if manip < self.sing_threshold:
            damp *= min(self.sing_threshold / (manip + 1e-9), self.max_sing_boost)

        dq = Ja.T @ np.linalg.solve(Ja @ Ja.T + damp * np.eye(6), twist)
        dq = np.clip(dq, -self.max_joint_vel, self.max_joint_vel)

        self.q[: self.n_arm] = np.clip(
            self.q[: self.n_arm] + dq * dt,
            self.q_min[: self.n_arm], self.q_max[: self.n_arm],
        )
        return (np.linalg.norm(lin_err) < self.lin_tol
                and np.linalg.norm(ang_err) < self.ang_tol)


if __name__ == "__main__":
    ik = CartesianServoIK()
    q0 = pin.neutral(ik.model)
    q0[1] = 0.5  # bend joint 2 off neutral so the pose is generic
    ik.set_q(q0)

    start = ik.ee_pose()
    target = start.copy()
    target.translation = start.translation + np.array([0.05, -0.05, 0.05])
    print("start TCP :", np.round(start.translation, 4))
    print("target TCP:", np.round(target.translation, 4))

    dt = 1 / 100
    for step in range(2000):
        if ik.servo(target, dt):
            print(f"reached in {step} steps ({step*dt:.2f} s)")
            break
    else:
        raise SystemExit("FAIL: did not converge in 2000 steps")
    final = ik.ee_pose()
    err = float(np.linalg.norm(final.translation - target.translation))
    assert err < 2e-3, err
    lo, hi = ik.q_min[:7], ik.q_max[:7]
    assert (ik.arm_q() >= lo).all() and (ik.arm_q() <= hi).all()
    print("final TCP :", np.round(final.translation, 4))
    print(f"pos error : {err*1000:.2f} mm, all joints within URDF limits")
    print("SELF-TEST PASSED")
