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
CAM_MOUNT_XYZ = (0.001, 0.016, 0.0157)  # field-tuned; (right, toward-jaws, up)
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


REBASED = Path("data/viz_meshes")  # derived, gitignored, regenerated


def _write_dae(dst, tris, rgb=(0.8, 0.8, 0.8)):
    """Minimal COLLADA with explicit Z_UP, meters, indexed vertices.

    tris: (T, 3, 3) float array, link-local meters. Normals are recomputed
    from the geometry (vendor STL normals are unreliable). Vertices are
    deduplicated so the XML stays a fraction of the raw triangle soup.
    """
    import numpy as np

    corners = tris.reshape(-1, 3)
    verts, inverse = np.unique(corners.round(7), axis=0, return_inverse=True)
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    n = np.cross(v1 - v0, v2 - v0)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 1e-12)
    T = len(tris)
    # index stream: [vertex_idx, normal_idx] per corner; normal = face index
    idx = np.empty(T * 6, dtype=np.int64)
    idx[0::2] = inverse
    idx[1::2] = np.repeat(np.arange(T), 3)

    def arr(a):
        return " ".join(f"{x:.7g}" for x in np.asarray(a).ravel())

    dst.write_text(f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_effects><effect id="fx"><profile_COMMON><technique sid="t">
    <phong><diffuse><color>{rgb[0]} {rgb[1]} {rgb[2]} 1</color></diffuse>
    <specular><color>0.2 0.2 0.2 1</color></specular>
    <shininess><float>20</float></shininess></phong>
  </technique></profile_COMMON></effect></library_effects>
  <library_materials><material id="mat"><instance_effect url="#fx"/></material></library_materials>
  <library_geometries><geometry id="g"><mesh>
    <source id="pos">
      <float_array id="pa" count="{len(verts) * 3}">{arr(verts)}</float_array>
      <technique_common><accessor source="#pa" count="{len(verts)}" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/>
        <param name="Z" type="float"/></accessor></technique_common>
    </source>
    <source id="nor">
      <float_array id="na" count="{T * 3}">{arr(n)}</float_array>
      <technique_common><accessor source="#na" count="{T}" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/>
        <param name="Z" type="float"/></accessor></technique_common>
    </source>
    <vertices id="v"><input semantic="POSITION" source="#pos"/></vertices>
    <triangles count="{T}" material="msym">
      <input semantic="VERTEX" source="#v" offset="0"/>
      <input semantic="NORMAL" source="#nor" offset="1"/>
      <p>{" ".join(map(str, idx))}</p>
    </triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="s">
    <node id="n"><instance_geometry url="#g"><bind_material>
      <technique_common><instance_material symbol="msym" target="#mat"/></technique_common>
    </bind_material></instance_geometry></node>
  </visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#s"/></scene>
</COLLADA>
""")


def rebase_meshes(root):
    """Bake visual origin + scale INTO the mesh vertices, per link.

    The vendor meshes are CAD-world millimeters with a compensating visual
    origin per link. rviz composes frame . origin . scale(mesh) correctly;
    Foxglove's URDF layer does not (links render exploded at their CAD-world
    offsets). Rewriting each mesh into its link-local frame in meters -
    identity origin, scale 1 - is the conventional pattern every renderer
    agrees on. Collision elements are stripped: this is a display model.
    """
    import struct

    import numpy as np
    import pinocchio as pin

    REBASED.mkdir(parents=True, exist_ok=True)
    n = 0
    for link in root.findall("link"):
        for col in link.findall("collision"):
            link.remove(col)
        v = link.find("visual")
        if v is None:
            continue
        mesh = v.find("geometry/mesh")
        if mesh is None:
            continue
        o = v.find("origin")
        xyz = np.array([float(x) for x in
                        ((o.get("xyz") if o is not None else None) or "0 0 0").split()])
        rpy = [float(x) for x in
               ((o.get("rpy") if o is not None else None) or "0 0 0").split()]
        R = (pin.utils.rotate("z", rpy[2]) @ pin.utils.rotate("y", rpy[1])
             @ pin.utils.rotate("x", rpy[0]))
        scale = float((mesh.get("scale") or "0.001").split()[0])
        src = ASSETS / Path(mesh.get("filename")).name
        # COLLADA, not STL: STL has no up-axis metadata, and Foxglove's
        # (three.js, Y-up) loader orients STLs differently from rviz -
        # meshes rendered detached/rotated from their correctly-posed
        # frames. DAE declares Z_UP explicitly; both renderers honor it.
        dst = REBASED / (src.stem + ".dae")
        if not dst.exists() or dst.stat().st_mtime < src.stat().st_mtime:
            with open(src, "rb") as f:
                f.seek(80)
                cnt = struct.unpack("<I", f.read(4))[0]
                raw = np.frombuffer(f.read(cnt * 50), dtype=np.uint8
                                    ).reshape(cnt, 50)
            tris = raw[:, 12:48].copy().view("<f4").reshape(-1, 3)
            verts = (tris.astype(np.float64) * scale) @ R.T + xyz  # link, m
            name = link.get("name")
            rgb = ((0.08, 0.08, 0.08) if name.startswith("ST3215")
                   else (0.15, 0.15, 0.15) if name == "camera_mount"
                   else (0.95, 0.45, 0.10))  # printed orange
            _write_dae(dst, verts.reshape(-1, 3, 3), rgb)
        # ALSO declare the color as a URDF material: Foxglove's URDF layer
        # colors links from URDF materials, not from mesh-file materials
        name = link.get("name")
        rgb = ((0.08, 0.08, 0.08) if name.startswith("ST3215")
               else (0.15, 0.15, 0.15) if name == "camera_mount"
               else (0.95, 0.45, 0.10))
        for old_mat in v.findall("material"):
            v.remove(old_mat)
        mat = ET.SubElement(v, "material")
        mat.set("name", f"col_{name}")
        c = ET.SubElement(mat, "color")
        c.set("rgba", f"{rgb[0]} {rgb[1]} {rgb[2]} 1")
        mesh.set("filename", dst.resolve().as_uri())
        if mesh.get("scale"):
            del mesh.attrib["scale"]
        if o is not None:
            o.set("xyz", "0 0 0")
            o.set("rpy", "0 0 0")
        n += 1
    return n


def main():
    tree = ET.parse(SRC)
    root = tree.getroot()
    # sanitize vendor joint names with SPACES ("Rigid 1") - XML-legal but a
    # classic trap for downstream parsers (Foxglove et al.)
    for j in root.findall("joint"):
        j.set("name", j.get("name").replace(" ", "_"))
    fix_jaw_frames(root)
    add_camera(root)

    missing = [Path(m.get("filename")).name for link in root.findall("link")
               for m in link.iter("mesh")
               if not (ASSETS / Path(m.get("filename")).name).exists()]
    if missing:
        raise SystemExit(f"missing meshes in {ASSETS}: {missing}")

    n_mesh = rebase_meshes(root)
    tree.write(DST)
    print(f"wrote {DST} ({n_mesh} link-local meshes -> {REBASED})")


if __name__ == "__main__":
    main()
