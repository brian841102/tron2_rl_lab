#!/usr/bin/env python3
"""Create the symmetric-limit K1 wheel-foot v2 URDF."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_URDF = ROOT / "urdf" / "k1_wheelfoot.urdf"
OUTPUT_URDF = ROOT / "urdf" / "k1_wheelfoot_v2.urdf"

# Limits are rounded to 0.01 rad and mirrored across each left/right pair.
# The smaller measured magnitude is used when the two sides differ, so the
# normalization does not enlarge the measured motion envelope. Axis signs
# remain those of the current wheel-foot hardware calibration.
JOINT_LIMITS = {
    "ALeft_Shoulder_Pitch": (-1.23, 2.96),
    "ARight_Shoulder_Pitch": (-2.96, 1.23),
    "Left_Shoulder_Roll": (-1.66, 1.66),
    "Right_Shoulder_Roll": (-1.66, 1.66),
    "Left_Elbow_Pitch": (-1.92, 1.92),
    "Right_Elbow_Pitch": (-1.92, 1.92),
    "Left_Elbow_Yaw": (-0.80, 2.28),
    "Right_Elbow_Yaw": (-2.28, 0.80),
    "Left_Hip_Pitch": (-2.99, 2.25),
    "Right_Hip_Pitch": (-2.25, 2.99),
    "Left_Hip_Roll": (-0.41, 1.53),
    "Right_Hip_Roll": (-1.53, 0.41),
    "Left_Hip_Yaw": (-1.00, 1.00),
    "Right_Hip_Yaw": (-1.00, 1.00),
    "left_leg_keep_pitch_joint": (-0.33, 2.00),
    "right_leg_keep_pitch_joint": (-2.00, 0.33),
}

# Restore the official effort values for the four common hip Roll/Yaw motors.
JOINT_EFFORTS = {
    "Left_Hip_Roll": 35.0,
    "Right_Hip_Roll": 35.0,
    "Left_Hip_Yaw": 20.0,
    "Right_Hip_Yaw": 20.0,
}


def main() -> None:
    tree = ET.parse(SOURCE_URDF)
    robot = tree.getroot()
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

    missing = (set(JOINT_LIMITS) | set(JOINT_EFFORTS)) - set(joints)
    if missing:
        raise KeyError(f"source URDF is missing joints: {sorted(missing)}")

    for name, (lower, upper) in JOINT_LIMITS.items():
        limit = joints[name].find("limit")
        if limit is None:
            raise ValueError(f"joint {name} has no limit element")
        limit.set("lower", f"{lower:g}")
        limit.set("upper", f"{upper:g}")

    for name, effort in JOINT_EFFORTS.items():
        limit = joints[name].find("limit")
        if limit is None:
            raise ValueError(f"joint {name} has no limit element")
        limit.set("effort", f"{effort:g}")

    ET.indent(tree, space="  ")
    tree.write(OUTPUT_URDF, encoding="utf-8", xml_declaration=True)
    with OUTPUT_URDF.open("ab") as output:
        output.write(b"\n")
    print(f"Wrote {OUTPUT_URDF}")


if __name__ == "__main__":
    main()
