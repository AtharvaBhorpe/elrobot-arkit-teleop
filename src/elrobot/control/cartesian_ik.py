"""Damped-least-squares Cartesian servo IK for the Elrobot arm.

Ported from franka-isaac-arkit-teleop's CartesianServoIK with the robot
swapped: URDF from docs/, TCP frame Gripper_Base_v1_1, 7 arm DoFs
(rev_motor_01..07 occupy q[0:7]; rev_motor_08 and the two jaw DoFs follow and
are never servoed).

    dq = J^T (J J^T + lambda^2 I)^-1 * twist

with singularity-adaptive lambda (the spec's verified finding: 13.9% of the
workspace has sigma_min < 0.01, so adaptive damping is load-bearing),
joint-velocity clamping, and URDF joint-limit clamping.

Self-test (no ROS): pixi run python -m elrobot.control.cartesian_ik
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin

URDF = str(Path(__file__).resolve().parents[3] / "docs" / "urdf_Elrobot.urdf")
TCP_FRAME = "Gripper_Base_v1_1"
ARM_JOINTS = [f"rev_motor_{i:02d}" for i in range(1, 8)]
GRIPPER_JOINT = "rev_motor_08"
# rev_motor_08 URDF range is [0, 2.2028]; exact tick mapping is the driver's
# job (calibration/urdf_ticks.json). These are viz/command targets only.
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 2.0
# jaw prismatic mimic ratios from the vendor URDF (m per rad of rev_motor_08);
# published with /joint_states so robot_state_publisher animates the jaws
JAW_MIMIC = {"rev_motor_08_1": -0.0115, "rev_motor_08_2": 0.0115}


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
        frozen: tuple = (),   # joint NAMES held at their seed pose (SO-101 mode)
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

        # SO-101 mode: frozen joints keep their seeded position forever; the
        # DLS solve runs on the ACTIVE columns only (a zeroed-column solve
        # would also drive sigma_min to 0 and max-boost the damping).
        assert all(f in ARM_JOINTS for f in frozen), frozen
        self.active = np.array([i for i, n in enumerate(ARM_JOINTS)
                                if n not in frozen])
        self.q_ref = None            # null-space posture anchor (see servo)
        self.kp_posture = 0.5        # gentle: 1/s pull, null-space only
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
        Ja = J[:, : self.n_arm][:, self.active]   # active columns only

        task = 6 if len(self.active) >= 6 else 3

        def _dls(Jm, v):
            """Damped least squares with the sigma_min-adaptive guard
            (the spec's verified criterion: 13.9% of the workspace has
            sigma_min < 0.01; a det-based metric max-boosted damping at
            every generic pose on this short-linked arm)."""
            _, S, Vt = np.linalg.svd(Jm, full_matrices=True)
            damp = self.damping
            if S[-1] < self.sing_threshold:
                damp *= min(self.sing_threshold / (S[-1] + 1e-9),
                            self.max_sing_boost)
            dq = Jm.T @ np.linalg.solve(
                Jm @ Jm.T + damp * np.eye(Jm.shape[0]), v)
            return dq, S, Vt

        n_act = len(self.active)
        if task == 6:
            dq_act, S, Vt = _dls(Ja, twist)
            # Null-space posture anchor: redundant DOFs wander unanchored
            # ("possessed elbow"). Pull gently toward the reference posture,
            # through the EXACT orthogonal complement of the controllable
            # directions (a damped-pinv projector leaked 6.4 mm into the TCP).
            if self.q_ref is not None:
                Vr = Vt[: np.count_nonzero(S > self.sing_threshold)]
                N = np.eye(n_act) - Vr.T @ Vr
                dq_act += N @ (self.kp_posture
                               * (self.q_ref[self.active]
                                  - self.q[: self.n_arm][self.active]))
        else:
            # Underactuated (SO-101 mode): TASK-PRIORITY. Position solved
            # exactly as the primary task (3 dims, 5 joints); orientation is
            # served strictly inside position's null space - the wrist tracks
            # whatever pitch/roll it can reach without ever costing position.
            # (Equal-weight 6-DOF least squares stalled 37 mm short.)
            dq1, Sp, Vtp = _dls(Ja[:3], twist[:3])
            Vr = Vtp[: np.count_nonzero(Sp > self.sing_threshold)]
            Np = np.eye(n_act) - Vr.T @ Vr           # exact position null space
            Jo = Ja[3:] @ Np
            dq2, _, _ = _dls(Jo, twist[3:] - Ja[3:] @ dq1)
            dq_act = dq1 + Np @ dq2
            # no posture anchor: 3 + 2 task dims consume all 5 joints

        dq = np.zeros(self.n_arm)                 # frozen joints never move
        dq[self.active] = np.clip(dq_act, -self.max_joint_vel,
                                  self.max_joint_vel)

        self.q[: self.n_arm] = np.clip(
            self.q[: self.n_arm] + dq * dt,
            self.q_min[: self.n_arm], self.q_max[: self.n_arm],
        )
        return (np.linalg.norm(lin_err) < self.lin_tol
                and (task == 3 or np.linalg.norm(ang_err) < self.ang_tol))


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

    # Null-space anchor mechanism test - the contract is exactly two-sided:
    # with q_ref far from the current posture and the target pinned at the
    # CURRENT TCP pose, the anchor must (a) visibly move the posture toward
    # q_ref through the redundant DOF while (b) leaving the TCP untouched.
    # (Long-horizon wander suppression is a hardware-feel property that a
    # short simulation cannot reproduce - DLS is minimum-norm per step, so
    # sim wander is noise-floor with or without the anchor.)
    s = CartesianServoIK()
    s.set_q(q0)
    hold = s.ee_pose()
    q_ref = q0[:7].copy()
    q_ref[2] += 0.6  # ask for a very different elbow-ish posture
    s.q_ref = q_ref
    d0 = float(np.linalg.norm(s.arm_q() - q_ref))
    for _ in range(2000):
        s.servo(hold, dt)
    d1 = float(np.linalg.norm(s.arm_q() - q_ref))
    tcp_moved = float(np.linalg.norm(s.ee_pose().translation
                                     - hold.translation))
    print(f"null-space anchor: posture distance to q_ref {d0:.3f} -> "
          f"{d1:.3f} rad while TCP moved {tcp_moved*1000:.2f} mm")
    assert d1 < d0 - 0.05, "anchor did not move the posture toward q_ref"
    assert tcp_moved < 3e-3, f"anchor disturbed the TCP: {tcp_moved*1000:.1f} mm"

    # SO-101 mode: joints 3 and 5 frozen -> 5 DoF, task-priority servo.
    # Contract: position EXACT always; achievable orientation (wrist
    # pitch/roll) tracked in position's null space; unreachable orientation
    # (yaw) sacrificed without costing position.
    # Measured contract (vs a zero-orientation-effort baseline): position
    # exact everywhere; pitch/roll targets recover the achievable component
    # (19.9->10.5 deg, 19.4->9.2 deg); yaw is unreachable (26 deg = baseline:
    # translating swings the base pan and nothing among the 5 joints can yaw
    # back independently). The ~9 deg floor is that translation-induced yaw.
    for ax, ang_max in ((None, 0.21), ("x", 0.25), ("y", 0.23), ("z", None)):
        fz = CartesianServoIK(frozen=("rev_motor_03", "rev_motor_05"))
        fz.set_q(q0)
        frozen_before = fz.arm_q()[[2, 4]]
        t2 = fz.ee_pose()
        t2.translation = t2.translation + np.array([0.04, -0.04, 0.04])
        if ax:
            t2.rotation = t2.rotation @ pin.utils.rotate(ax, 0.3)
        for _ in range(2000):
            fz.servo(t2, dt)
        err_f = float(np.linalg.norm(fz.ee_pose().translation
                                     - t2.translation))
        ang_f = float(np.linalg.norm(pin.log3(
            fz.ee_pose().rotation.T @ t2.rotation)))
        moved = np.abs(fz.arm_q()[[2, 4]] - frozen_before).max()
        print(f"frozen(3,5) rot={ax or 'none':<4}: pos {err_f*1000:5.1f} mm, "
              f"ang {np.degrees(ang_f):5.1f} deg, frozen moved {moved:.0e}")
        assert moved == 0.0, "frozen joints must not move"
        assert err_f < 5e-3, f"position must stay exact: {err_f*1000:.1f} mm"
        if ang_max is not None:
            assert ang_f < ang_max, \
                f"achievable orientation not tracked: {ang_f:.3f} rad"
    print("SELF-TEST PASSED")
