"""Generate a viewable URDF for rviz2 from the real one.

Two fixes over docs/urdf_Elrobot.urdf, neither of which touches kinematics:

1. Mesh refs like `assets/base_link.stl` become absolute `file://` URIs —
   rviz's resource retriever cannot resolve bare relative paths. Absolute
   paths are machine-specific, which is why this regenerates on every
   `pixi run view` instead of being committed.
2. The jaw links are KEPT, wrong-looking origins and all: the vendor URDF
   is self-consistent — each jaw's visual origin exactly compensates its
   bogus (CAD-world) joint origin, so the rendered mesh lands correctly on
   the gripper and slides correctly with the prismatic joints. Only the TF
   FRAMES are displaced ~38 cm (cosmetic; the rviz config shows no jaw
   axes). Jaw joint states are published by the driver / ik node using the
   vendor's mimic ratio (+/-0.0115 m per rad of rev_motor_08).

Meshes are vendored from https://github.com/norma-core/norma-core (MIT),
see docs/assets/LICENSE.norma-core.

Output: docs/urdf_Elrobot_viz.urdf  (regenerated; do not edit by hand)
"""

import xml.etree.ElementTree as ET
from pathlib import Path

SRC = Path("docs/urdf_Elrobot.urdf")
DST = Path("docs/urdf_Elrobot_viz.urdf")
ASSETS = Path("docs/assets").resolve()


def _stl_center_m(path):
    """Bbox center of a binary STL, mm -> m (vertices are CAD-world mm)."""
    import struct

    import numpy as np
    with open(path, "rb") as f:
        f.seek(80)
        n = struct.unpack("<I", f.read(4))[0]
        vs = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)[
            :, 12:48].copy().view("<f4").reshape(-1, 3)
    return (vs.min(0) + vs.max(0)) / 2 / 1000.0


def fix_jaw_frames(root):
    """Move the jaw LINK FRAMES onto the jaws; keep the pixels identical.

    The vendor's jaw joint origins are CAD-world coords (frame ~38 cm off)
    with visual origins compensating exactly. Rendering composes
    frame . visual, which is invariant under: origin += delta,
    visual -= delta. Delta is chosen to put each frame at its jaw's mesh
    center expressed in the gripper-base frame, so TF lands ON the jaw and
    slides with it. Rotations are all identity here, so it is pure vector
    arithmetic.
    """
    import numpy as np
    import pinocchio as pin

    m = pin.buildModelFromUrdf(str(SRC))
    d = m.createData()
    pin.forwardKinematics(m, d, pin.neutral(m))
    pin.updateFramePlacements(m, d)
    Tgb = d.oMf[m.getFrameId("Gripper_Base_v1_1")]

    links = {l.get("name"): l for l in root.findall("link")}
    for j in root.findall("joint"):
        if j.get("name") not in ("rev_motor_08_1", "rev_motor_08_2"):
            continue
        link = links[j.find("child").get("link")]
        mesh_file = ASSETS / Path(
            link.find("visual/geometry/mesh").get("filename")).name
        c_world = _stl_center_m(mesh_file)
        o = j.find("origin")
        old = np.array([float(x) for x in o.get("xyz").split()])
        new = Tgb.rotation.T @ (c_world - Tgb.translation)
        o.set("xyz", " ".join(f"{v:.6f}" for v in new))
        delta = old - new  # what the frame moved by; visuals move back
        for tag in ("visual", "collision"):
            el = link.find(f"{tag}/origin")
            if el is None:
                continue
            vx = np.array([float(x) for x in el.get("xyz").split()])
            el.set("xyz", " ".join(f"{v:.6f}" for v in vx + delta))


# -- camera (Innomaker U20CAM-1080P on CameraMount_square_27mm) -----------
# Mount pose on the gripper base, TUNE AGAINST THE REAL BRACKET: xyz is the
# bracket base center on the top plate, in the Gripper_Base link frame
# (x right, y toward jaws, z up, meters); yaw 90 deg turns the bracket's
# camera direction onto the arm axis. Nudge xyz until rviz matches the photo.
CAM_MOUNT_XYZ = (0.0, 0.012, 0.0237)
CAM_MOUNT_RPY = (0.0, 0.0, 1.5708)
# Optical frame: derived from the mount STL - camera plate normal
# (0.91, 0, -0.42) in mount-local frame (24.8 deg below horizontal), lens
# center ~10 mm out from the plate face centroid (-0.6, 0, 49.4) mm.
CAM_OPTICAL_XYZ = (0.0085, 0.0, 0.0452)  # in camera_mount frame, meters
# REP-103 optical: z = view direction, x = image right, y = image down.
# Columns [x_opt y_opt z_opt] in mount frame; if the image appears rotated
# 180 deg later, flip the signs of the x_opt and y_opt columns.
CAM_OPTICAL_AXES = ((0, -0.42, 0.91), (-1, 0, 0), (0, -0.91, -0.42))  # rows


def add_camera(root):
    """camera_mount (bracket mesh) + camera_optical_frame, fixed joints.

    Fixed joints need no joint states - robot_state_publisher broadcasts
    their TF from the URDF alone. The U20CAM-1080P is a 120 deg FOV camera;
    FOV lives in the CameraInfo pipeline, not the URDF.
    """
    import numpy as np
    import pinocchio as pin

    def make(tag, parent, **attrs):
        el = ET.SubElement(parent, tag)
        for k, v in attrs.items():
            el.set(k, v)
        return el

    def fixed_joint(name, parent, child, xyz, rpy):
        j = make("joint", root, name=name, type="fixed")
        make("origin", j, xyz=" ".join(f"{v:.6f}" for v in xyz),
             rpy=" ".join(f"{v:.6f}" for v in rpy))
        make("parent", j, link=parent)
        make("child", j, link=child)

    mount = make("link", root, name="camera_mount")
    v = make("visual", mount)
    make("origin", v, xyz="0 0 0", rpy="0 0 0")
    g = make("geometry", v)
    make("mesh", g,
         filename=(ASSETS / "CameraMount_square_27mm.stl").as_uri(),
         scale="0.001 0.001 0.001")
    mat = make("material", v, name="cam_black")
    make("color", mat, rgba="0.15 0.15 0.15 1")
    fixed_joint("camera_mount_joint", "Gripper_Base_v1_1", "camera_mount",
                CAM_MOUNT_XYZ, CAM_MOUNT_RPY)

    make("link", root, name="camera_optical_frame")
    R = np.array(CAM_OPTICAL_AXES, dtype=float)
    U, _, Vt = np.linalg.svd(R)  # orthonormalize the rounded axes
    rpy = pin.rpy.matrixToRpy(U @ Vt)
    fixed_joint("camera_optical_joint", "camera_mount",
                "camera_optical_frame", CAM_OPTICAL_XYZ, rpy)


def main():
    tree = ET.parse(SRC)
    root = tree.getroot()
    fix_jaw_frames(root)
    add_camera(root)

    missing = []
    n_mesh = 0
    for link in root.findall("link"):
        for mesh in link.iter("mesh"):
            fname = Path(mesh.get("filename")).name
            path = ASSETS / fname
            if not path.exists():
                missing.append(fname)
            mesh.set("filename", path.as_uri())
            n_mesh += 1

    if missing:
        raise SystemExit(f"missing meshes in {ASSETS}: {missing}")

    tree.write(DST)
    print(f"wrote {DST} ({n_mesh} meshes -> {ASSETS})")


if __name__ == "__main__":
    main()
