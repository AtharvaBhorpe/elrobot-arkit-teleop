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


def main():
    tree = ET.parse(SRC)
    root = tree.getroot()

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
