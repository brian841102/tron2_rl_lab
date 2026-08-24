#!/usr/bin/env python3
"""Create K1 wheel-foot v3 with the left keep-pitch limit aligned to right."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_URDF = ROOT / "urdf" / "k1_wheelfoot_v2.urdf"
OUTPUT_URDF = ROOT / "urdf" / "k1_wheelfoot_v3.urdf"


def main() -> None:
    tree = ET.parse(SOURCE_URDF)
    robot = tree.getroot()
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

    left = joints["left_leg_keep_pitch_joint"]
    right = joints["right_leg_keep_pitch_joint"]
    left_limit = left.find("limit")
    right_limit = right.find("limit")
    if left_limit is None or right_limit is None:
        raise ValueError("keep-pitch joints must both define limits")

    # The right-side measured limit is the trusted value for this revision.
    left_limit.set("lower", right_limit.get("lower", "-2.00"))
    left_limit.set("upper", right_limit.get("upper", "0.33"))

    # Keep-pitch reuses the standard K1 knee actuator. The wheel-foot travel
    # remains custom, but the motor effort and speed match Knee_Pitch.
    for name in ("left_leg_keep_pitch_joint", "right_leg_keep_pitch_joint"):
        limit = joints[name].find("limit")
        if limit is None:
            raise ValueError(f"joint {name} has no limit element")
        limit.set("effort", "40")
        limit.set("velocity", "12.5")

    ET.indent(tree, space="  ")
    tree.write(OUTPUT_URDF, encoding="utf-8", xml_declaration=True)
    with OUTPUT_URDF.open("ab") as output:
        output.write(b"\n")
    print(f"Wrote {OUTPUT_URDF}")


if __name__ == "__main__":
    main()
