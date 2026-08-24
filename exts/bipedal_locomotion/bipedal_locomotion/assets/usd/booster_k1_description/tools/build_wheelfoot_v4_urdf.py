#!/usr/bin/env python3
"""Create wheel-foot v4 with official mechanical limits on common joints."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_URDF = ROOT / "urdf" / "k1_wheelfoot_v3.urdf"
OFFICIAL_URDF = ROOT / "urdf" / "k1_22dof.urdf"
OUTPUT_URDF = ROOT / "urdf" / "k1_wheelfoot_v4.urdf"


def vector(value: str) -> tuple[float, float, float]:
    values = tuple(float(part) for part in value.split())
    if len(values) != 3:
        raise ValueError(f"axis must have three values: {value}")
    return values


def number(value: float) -> str:
    if abs(value) < 1e-12:
        value = 0.0
    return f"{value:g}"


def mechanical_limits(official_joint: ET.Element, target_joint: ET.Element) -> tuple[str, str]:
    official_axis = official_joint.find("axis")
    target_axis = target_joint.find("axis")
    official_limit = official_joint.find("limit")
    if official_axis is None or target_axis is None or official_limit is None:
        raise ValueError(f"joint {official_joint.get('name')} is incomplete")

    source_axis = vector(official_axis.get("xyz", ""))
    target_axis_value = vector(target_axis.get("xyz", ""))
    if target_axis_value == source_axis:
        sign = 1.0
    elif target_axis_value == tuple(-value for value in source_axis):
        sign = -1.0
    else:
        raise ValueError(
            f"joint {official_joint.get('name')} has an unrelated axis: "
            f"official={source_axis}, target={target_axis_value}"
        )

    lower = float(official_limit.get("lower", "nan"))
    upper = float(official_limit.get("upper", "nan"))
    if sign < 0.0:
        lower, upper = -upper, -lower
    return number(lower), number(upper)


def main() -> None:
    tree = ET.parse(SOURCE_URDF)
    source_robot = tree.getroot()
    official_robot = ET.parse(OFFICIAL_URDF).getroot()
    target_joints = {joint.attrib["name"]: joint for joint in source_robot.findall("joint")}
    official_joints = {joint.attrib["name"]: joint for joint in official_robot.findall("joint")}

    common_names = sorted(set(target_joints) & set(official_joints))
    if not common_names:
        raise ValueError("no common joints found")

    for name in common_names:
        target = target_joints[name]
        official = official_joints[name]
        target_limit = target.find("limit")
        if target_limit is None:
            raise ValueError(f"target joint {name} has no limit element")
        lower, upper = mechanical_limits(official, target)
        target_limit.set("lower", lower)
        target_limit.set("upper", upper)

    ET.indent(tree, space="  ")
    tree.write(OUTPUT_URDF, encoding="utf-8", xml_declaration=True)
    with OUTPUT_URDF.open("ab") as output:
        output.write(b"\n")
    print(f"Updated official mechanical limits on {len(common_names)} common joints")
    print(f"Wrote {OUTPUT_URDF}")


if __name__ == "__main__":
    main()
