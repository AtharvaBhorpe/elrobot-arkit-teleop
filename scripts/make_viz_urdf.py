"""Generate a viewable URDF for rviz2 from the real one.

Two fixes over docs/urdf_Elrobot.urdf, neither of which touches kinematics:

1. Mesh refs like `assets/base_link.stl` become absolute `file://` URIs —
   rviz's resource retriever cannot resolve bare relative paths. Absolute
   paths are machine-specific, which is why this regenerates on every
   `pixi run view` instead of being committed.
2. The jaw joints and links are removed entirely: `rev_motor_08_1/_2`
   origins kept CAD world coordinates (~38 cm off, upstream vendor bug —
   verified identical in norma-core's own elrobot_follower.urdf). They are
   leaves, so the arm chain is untouched; removing them also stops
   robot_state_publisher broadcasting their bogus TF.

Meshes are vendored from https://github.com/norma-core/norma-core (MIT),
see docs/assets/LICENSE.norma-core.

Output: docs/urdf_Elrobot_viz.urdf  (regenerated; do not edit by hand)
"""

import xml.etree.ElementTree as ET
from pathlib import Path

SRC = Path("docs/urdf_Elrobot.urdf")
DST = Path("docs/urdf_Elrobot_viz.urdf")
ASSETS = Path("docs/assets").resolve()
BROKEN_JOINTS = {"rev_motor_08_1", "rev_motor_08_2"}  # CAD-world origins


def main():
    tree = ET.parse(SRC)
    root = tree.getroot()

    # Remove the jaw joints and links outright (leaves, display-only model):
    # keeping them meant robot_state_publisher broadcast their bogus TF, which
    # rendered as frames floating 38 cm from the gripper.
    broken_links = set()
    for j in list(root.findall("joint")):
        if j.get("name") in BROKEN_JOINTS:
            broken_links.add(j.find("child").get("link"))
            root.remove(j)
    for link in list(root.findall("link")):
        if link.get("name") in broken_links:
            root.remove(link)

    missing = []
    for link in root.findall("link"):
        for mesh in link.iter("mesh"):
            fname = Path(mesh.get("filename")).name
            path = ASSETS / fname
            if not path.exists():
                missing.append(fname)
            mesh.set("filename", path.as_uri())

    if missing:
        raise SystemExit(f"missing meshes in {ASSETS}: {missing}")

    tree.write(DST)
    print(f"wrote {DST} (meshes -> {ASSETS}, "
          f"{len(broken_links)} jaw links stripped)")


if __name__ == "__main__":
    main()
