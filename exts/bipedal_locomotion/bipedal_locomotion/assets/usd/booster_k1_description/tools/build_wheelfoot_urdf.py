#!/usr/bin/env python3
"""Build a K1 preview URDF with mirrored wheel-foot branches."""

from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from pathlib import Path

from estimate_trunk_inertia import TRUNK_ESTIMATE
from filter_base_link_small_blocks import LEFT_SIDE_MESH, RIGHT_SIDE_MESH, filter_base_link_mesh
from generate_wheel_collision_mesh import OUTPUT_MESH as WHEEL_COLLISION_MESH
from generate_wheel_collision_mesh import generate_wheel_collision_mesh


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_NAME = "booster_k1_description"
K1_URDF = ROOT / "urdf" / "k1_22dof.urdf"
WHEEL_URDF = ROOT / "cad" / "wheelfoot" / "wheelfoot_reference.urdf"
OUTPUT_URDF = ROOT / "urdf" / "k1_wheelfoot.urdf"
WHEEL_MESH_URI = f"package://{PACKAGE_NAME}/meshes/wheelfoot/"
WHEEL_SOURCE_LINK = "right_leg_anker_pitch_Link"
WHEEL_SOURCE_JOINT = "right_leg_anker_pitch_joint"
WHEEL_COLLISION_Y = "0.032"
WHEEL_CROWN_RADIUS = "0.075"
WHEEL_CROWN_WIDTH = "0.006"
WHEEL_SHOULDER_OFFSET = 0.0165
HALF_TURN_RPY = "3.141592653589793 0 0"
IMU_ORIGIN = (0.03, 0.0, 0.005)
IMU_MASS = 0.01
IMU_SIZE = (0.03, 0.02, 0.005)

# These omissions apply only to the generated wheel-foot variant.  The source
# k1_22dof.urdf remains intact, so Head_2 can be restored by removing these two
# entries after the hardware is reinstalled.
REMOVED_LINKS = {
    "Head_2",
    "Left_Shank",
    "Left_Ankle_Cross",
    "left_foot_link",
    "Right_Shank",
    "Right_Ankle_Cross",
    "right_foot_link",
}
REMOVED_JOINTS = {
    "Head_pitch",
    "Left_Knee_Pitch",
    "Left_Ankle_Pitch",
    "Left_Ankle_Roll",
    "Right_Knee_Pitch",
    "Right_Ankle_Pitch",
    "Right_Ankle_Roll",
}


def format_number(value: float) -> str:
    return f"{value:.12g}"


def format_vector(values: tuple[float, ...]) -> str:
    return " ".join(format_number(value) for value in values)


def apply_trunk_inertial_estimate(robot: ET.Element) -> None:
    trunk = find_named(robot.findall("link"), "Trunk")
    origin = trunk.find("inertial/origin")
    mass = trunk.find("inertial/mass")
    inertia = trunk.find("inertial/inertia")
    if origin is None or mass is None or inertia is None:
        raise ValueError("Trunk has an incomplete inertial element")

    origin.set("xyz", format_vector(TRUNK_ESTIMATE.center))
    origin.set("rpy", "0 0 0")
    mass.set("value", format_number(TRUNK_ESTIMATE.mass))
    inertia.attrib.update(
        dict(
            zip(
                ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"),
                (format_number(value) for value in TRUNK_ESTIMATE.inertia),
            )
        )
    )


def make_imu_link() -> ET.Element:
    link = ET.Element("link", {"name": "imu_link"})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": format_number(IMU_MASS)})
    x, y, z = IMU_SIZE
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": format_number(IMU_MASS * (y * y + z * z) / 12.0),
            "ixy": "0",
            "ixz": "0",
            "iyy": format_number(IMU_MASS * (x * x + z * z) / 12.0),
            "iyz": "0",
            "izz": format_number(IMU_MASS * (x * x + y * y) / 12.0),
        },
    )
    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": format_vector(IMU_SIZE)})
    material = ET.SubElement(visual, "material", {"name": "imu_black"})
    ET.SubElement(material, "color", {"rgba": "0.08 0.08 0.08 1"})
    return link


def make_imu_joint() -> ET.Element:
    joint = ET.Element("joint", {"name": "Trunk_to_imu", "type": "fixed"})
    ET.SubElement(
        joint,
        "origin",
        {"xyz": format_vector(IMU_ORIGIN), "rpy": "0 0 0"},
    )
    ET.SubElement(joint, "parent", {"link": "Trunk"})
    ET.SubElement(joint, "child", {"link": "imu_link"})
    return joint


# The CAD branch named "right" lies at +Y, which is the physical K1 left side.
# Its Y-mirrored branch supplies the physical K1 right side.
LEFT_LINK_SPECS = (
    ("base_link", "left_base_link", RIGHT_SIDE_MESH.name),
    ("right_leg_keep_pitch_Link", "left_leg_keep_pitch_Link", "right_leg_keep_pitch_Link.STL"),
    ("right_leg_anker_pitch_Link", "left_wheel_link", "right_leg_anker_pitch_Link.STL"),
)
RIGHT_LINK_SPECS = (
    ("base_link", "right_base_link", LEFT_SIDE_MESH.name),
    ("right_leg_keep_pitch_Link", "right_leg_keep_pitch_Link", "left_leg_keep_pitch_Link.STL"),
    ("right_leg_anker_pitch_Link", "right_wheel_link", "left_leg_anker_pitch_Link.STL"),
)
RIGHT_LINK_NAMES = {
    "base_link": "right_base_link",
    "right_leg_keep_pitch_Link": "right_leg_keep_pitch_Link",
    "right_leg_anker_pitch_Link": "right_wheel_link",
}
LEFT_LINK_NAMES = {
    "base_link": "left_base_link",
    "right_leg_keep_pitch_Link": "left_leg_keep_pitch_Link",
    "right_leg_anker_pitch_Link": "left_wheel_link",
}

# The SolidWorks export used an approximately 1000 kg/m^3 placeholder density
# for every wheel-foot component. These overrides use measured masses for the
# motor/base, lower leg, and rotating wheel-foot. The CAD inertia tensors are
# scaled linearly from their prior source masses while preserving each COM.
WHEELFOOT_INERTIAL_OVERRIDES = {
    "base_link": {
        "origin": "-2.28776125948645e-06 0.0918110362128847 -6.12430857779765e-06",
        "mass": "0.414",
        "inertia": {
            "ixx": "0.000423687210687687",
            "ixy": "2.51247343560104e-08",
            "ixz": "1.37015571348420e-07",
            "iyy": "0.000539097925330282",
            "iyz": "2.77692372310688e-09",
            "izz": "0.000423088036699582",
        },
    },
    "right_leg_keep_pitch_Link": {
        "origin": "0.00196628298771043 0.0098922281432357 -0.0791439972707094",
        "mass": "0.204",
        "inertia": {
            "ixx": "0.000893653044644746",
            "ixy": "-2.19954526020526e-08",
            "ixz": "-4.26860799781967e-07",
            "iyy": "0.000987780297269728",
            "iyz": "2.26545952402249e-05",
            "izz": "0.000104751237583495",
        },
    },
    "right_leg_anker_pitch_Link": {
        "origin": "0.000141767 -0.021536494 0.000158334",
        "mass": "0.491",
        "inertia": {
            "ixx": "0.000704842235587400",
            "ixy": "0",
            "ixz": "0",
            "iyy": "0.001128650249777336",
            "iyz": "0",
            "izz": "0.000700396681097755",
        },
    },
}

RIGHT_JOINT_NAMES = {
    "right_leg_keep_pitch_joint": "right_leg_keep_pitch_joint",
    "right_leg_anker_pitch_joint": "right_wheel_joint",
}
LEFT_JOINT_NAMES = {
    "right_leg_keep_pitch_joint": "left_leg_keep_pitch_joint",
    "right_leg_anker_pitch_joint": "left_wheel_joint",
}

# Hardware measurements captured on 2026-07-16 established the joint
# directions. Every -1 row is represented here by a negated axis.
# ARight_Shoulder_Pitch and left_leg_keep_pitch_joint were missing their
# direction column; +1 is inferred from the existing model and their paired
# joints.
#
# Left_Hip_Roll, Left_Hip_Yaw, and both keep-pitch joints were re-zeroed and
# remeasured on 2026-07-21. Their hardware zero now matches the URDF joint zero,
# so these four ranges apply directly without encoder offsets.
REAL_ROBOT_JOINT_SPECS = {
    "ALeft_Shoulder_Pitch": ("0 -1 0", -1.23, 2.96),
    "Left_Shoulder_Roll": ("-1 0 0", -1.66, 1.66),
    "Left_Elbow_Pitch": ("0 -1 0", -1.92, 1.92),
    "Left_Elbow_Yaw": ("0 0 -1", -0.82, 2.28),
    "ARight_Shoulder_Pitch": ("0 1 0", -2.96, 1.23),
    "Right_Shoulder_Roll": ("-1 0 0", -1.66, 1.66),
    "Right_Elbow_Pitch": ("0 1 0", -1.92, 1.92),
    "Right_Elbow_Yaw": ("0 0 -1", -2.28, 0.80),
    "Left_Hip_Pitch": ("0 1 0", -2.99, 2.26),
    "Left_Hip_Roll": ("1 0 0", -0.42, 1.53),
    "Left_Hip_Yaw": ("0 0 -1", -1.01, 1.09),
    "left_leg_keep_pitch_joint": ("0 -1 0", -0.35, 2.0),
    "Right_Hip_Pitch": ("0 -1 0", -2.25, 3.00),
    "Right_Hip_Roll": ("1 0 0", -1.58, 0.41),
    "Right_Hip_Yaw": ("0 0 -1", -1.05, 1.05),
    "right_leg_keep_pitch_joint": ("0 -1 0", -2.02, 0.33),
    "left_wheel_joint": ("0 1 0", None, None),
    "right_wheel_joint": ("0 1 0", None, None),
}

# URDF effort limits are unsigned magnitudes, so 30 Nm represents a symmetric
# hardware torque range of [-30, 30] Nm.
REAL_ROBOT_JOINT_EFFORT_LIMITS = {
    "Left_Hip_Pitch": 30.0,
    "Left_Hip_Roll": 30.0,
    "Left_Hip_Yaw": 30.0,
    "left_leg_keep_pitch_joint": 30.0,
    "Right_Hip_Pitch": 30.0,
    "Right_Hip_Roll": 30.0,
    "Right_Hip_Yaw": 30.0,
    "right_leg_keep_pitch_joint": 30.0,
}


def find_named(elements: list[ET.Element], name: str) -> ET.Element:
    for element in elements:
        if element.attrib.get("name") == name:
            return element
    raise KeyError(name)


def set_mesh_path(element: ET.Element, mesh_name: str) -> None:
    for mesh in element.findall(".//mesh"):
        mesh.set("filename", WHEEL_MESH_URI + mesh_name)


def apply_wheelfoot_inertial_override(link: ET.Element, source_name: str) -> None:
    spec = WHEELFOOT_INERTIAL_OVERRIDES.get(source_name)
    if spec is None:
        return

    origin = link.find("inertial/origin")
    mass = link.find("inertial/mass")
    inertia = link.find("inertial/inertia")
    if origin is None or mass is None or inertia is None:
        raise ValueError(f"link {link.attrib['name']} has an incomplete inertial element")

    origin.set("xyz", spec["origin"])
    origin.set("rpy", "0 0 0")
    mass.set("value", spec["mass"])
    inertia.attrib.update(spec["inertia"])


def negate_number(value: str) -> str:
    if float(value) == 0.0:
        return "0"
    if value.startswith("-"):
        return value[1:]
    if value.startswith("+"):
        return "-" + value[1:]
    return "-" + value


def mirror_origin_across_y(origin: ET.Element | None) -> None:
    if origin is None:
        return
    xyz = origin.attrib.get("xyz", "0 0 0").split()
    if len(xyz) != 3:
        raise ValueError(f"origin xyz must have three values: {xyz}")
    xyz[1] = negate_number(xyz[1])
    origin.set("xyz", " ".join(xyz))

    rpy = origin.attrib.get("rpy", "0 0 0").split()
    if len(rpy) != 3:
        raise ValueError(f"origin rpy must have three values: {rpy}")
    rpy[0] = negate_number(rpy[0])
    rpy[2] = negate_number(rpy[2])
    origin.set("rpy", " ".join(rpy))


def mirror_link_dynamics_across_y(link: ET.Element) -> None:
    mirror_origin_across_y(link.find("inertial/origin"))
    for origin in link.findall("visual/origin") + link.findall("collision/origin"):
        mirror_origin_across_y(origin)
    inertia = link.find("inertial/inertia")
    if inertia is None:
        raise ValueError(f"link {link.attrib['name']} has no inertia tensor")
    for cross_term in ("ixy", "iyz"):
        inertia.set(cross_term, negate_number(inertia.attrib[cross_term]))


def replace_wheel_collision(link: ET.Element, mirror_y: bool) -> None:
    collisions = link.findall("collision")
    if len(collisions) != 1:
        raise ValueError(
            f"wheel link {link.attrib['name']} must have exactly one collision element"
        )

    collision = collisions[0]
    collision.clear()
    center_y = float(WHEEL_COLLISION_Y) if mirror_y else -float(WHEEL_COLLISION_Y)
    ET.SubElement(
        collision,
        "origin",
        {"xyz": f"0 {center_y:g} 0", "rpy": "1.5707963267948966 0 0"},
    )
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(
        geometry,
        "cylinder",
        {"radius": WHEEL_CROWN_RADIUS, "length": WHEEL_CROWN_WIDTH},
    )

    for y_offset, rpy in (
        (-WHEEL_SHOULDER_OFFSET, "0 0 0"),
        (WHEEL_SHOULDER_OFFSET, HALF_TURN_RPY),
    ):
        shoulder = ET.SubElement(link, "collision")
        ET.SubElement(
            shoulder,
            "origin",
            {"xyz": f"0 {center_y + y_offset:g} 0", "rpy": rpy},
        )
        shoulder_geometry = ET.SubElement(shoulder, "geometry")
        ET.SubElement(
            shoulder_geometry,
            "mesh",
            {"filename": WHEEL_MESH_URI + WHEEL_COLLISION_MESH.name},
        )


def clone_wheel_link(
    wheel_robot: ET.Element,
    source_name: str,
    target_name: str,
    mesh_name: str,
    mirror_y: bool = False,
) -> ET.Element:
    source = find_named(wheel_robot.findall("link"), source_name)
    link = copy.deepcopy(source)
    link.set("name", target_name)
    set_mesh_path(link, mesh_name)
    apply_wheelfoot_inertial_override(link, source_name)
    if mirror_y:
        mirror_link_dynamics_across_y(link)
    if source_name == WHEEL_SOURCE_LINK:
        replace_wheel_collision(link, mirror_y)
    return link


def order_limit_bounds(joint: ET.Element) -> None:
    limit = joint.find("limit")
    if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
        return
    lower = float(limit.attrib["lower"])
    upper = float(limit.attrib["upper"])
    if lower > upper:
        limit.set("lower", f"{upper:g}")
        limit.set("upper", f"{lower:g}")


def make_wheel_joint_continuous(joint: ET.Element) -> None:
    limit = joint.find("limit")
    if limit is None or "effort" not in limit.attrib or "velocity" not in limit.attrib:
        raise ValueError(
            f"wheel joint {joint.attrib['name']} must define effort and velocity limits"
        )
    joint.set("type", "continuous")
    limit.attrib.pop("lower", None)
    limit.attrib.pop("upper", None)


def clone_wheel_joint(
    wheel_robot: ET.Element,
    source_name: str,
    target_name: str,
    link_names: dict[str, str],
    mirror_y: bool = False,
) -> ET.Element:
    source = find_named(wheel_robot.findall("joint"), source_name)
    joint = copy.deepcopy(source)
    joint.set("name", target_name)
    for relation in ("parent", "child"):
        node = joint.find(relation)
        source_link = node.attrib["link"]
        node.set("link", link_names[source_link])
    if mirror_y:
        # A Y reflection leaves these pitch-joint axes at (0, -1, 0).
        mirror_origin_across_y(joint.find("origin"))
    if source_name == WHEEL_SOURCE_JOINT:
        make_wheel_joint_continuous(joint)
    else:
        order_limit_bounds(joint)
    return joint


def make_fixed_base_joint(name: str, parent: str, child: str, y: str) -> ET.Element:
    joint = ET.Element("joint", {"name": name, "type": "fixed"})
    ET.SubElement(
        joint,
        "origin",
        {
            "xyz": f"-0.014 {y} -0.117",
            "rpy": "0 0 0",
        },
    )
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    return joint


def apply_real_robot_joint_specs(robot: ET.Element) -> None:
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    required_joints = set(REAL_ROBOT_JOINT_SPECS) | set(REAL_ROBOT_JOINT_EFFORT_LIMITS)
    missing = required_joints - set(joints)
    if missing:
        raise KeyError(f"real-robot joint specs reference missing joints: {sorted(missing)}")

    for name, (axis_xyz, lower, upper) in REAL_ROBOT_JOINT_SPECS.items():
        joint = joints[name]
        axis = joint.find("axis")
        limit = joint.find("limit")
        if axis is None or limit is None:
            raise ValueError(f"joint {name} must define axis and limit elements")
        axis.set("xyz", axis_xyz)

        if lower is None or upper is None:
            if joint.attrib.get("type") != "continuous":
                raise ValueError(f"unbounded hardware joint {name} must be continuous")
            limit.attrib.pop("lower", None)
            limit.attrib.pop("upper", None)
            continue

        if lower >= upper:
            raise ValueError(f"joint {name} has invalid hardware limits: {lower}, {upper}")
        limit.set("lower", f"{lower:g}")
        limit.set("upper", f"{upper:g}")

    for name, effort in REAL_ROBOT_JOINT_EFFORT_LIMITS.items():
        limit = joints[name].find("limit")
        if limit is None:
            raise ValueError(f"joint {name} must define a limit element")
        if effort <= 0.0:
            raise ValueError(f"joint {name} has invalid effort limit: {effort}")
        limit.set("effort", f"{effort:g}")


def build() -> None:
    filter_base_link_mesh()
    generate_wheel_collision_mesh()

    k1_tree = ET.parse(K1_URDF)
    k1_robot = k1_tree.getroot()
    k1_robot.set("name", "K1_wheelfoot")

    for child in list(k1_robot):
        if child.tag == "link" and child.attrib.get("name") in REMOVED_LINKS:
            k1_robot.remove(child)
        if child.tag == "joint" and child.attrib.get("name") in REMOVED_JOINTS:
            k1_robot.remove(child)

    apply_trunk_inertial_estimate(k1_robot)
    k1_robot.append(make_imu_link())
    k1_robot.append(make_imu_joint())

    wheel_robot = ET.parse(WHEEL_URDF).getroot()
    for source_name, target_name, mesh_name in LEFT_LINK_SPECS:
        k1_robot.append(clone_wheel_link(wheel_robot, source_name, target_name, mesh_name))
    for source_name, target_name, mesh_name in RIGHT_LINK_SPECS:
        k1_robot.append(
            clone_wheel_link(wheel_robot, source_name, target_name, mesh_name, mirror_y=True)
        )

    k1_robot.append(
        make_fixed_base_joint(
            "Right_Knee_Wheelfoot_Base", "Right_Hip_Yaw", "right_base_link", "0.096"
        )
    )
    k1_robot.append(
        make_fixed_base_joint(
            "Left_Knee_Wheelfoot_Base", "Left_Hip_Yaw", "left_base_link", "-0.096"
        )
    )

    for source_name, target_name in LEFT_JOINT_NAMES.items():
        k1_robot.append(clone_wheel_joint(wheel_robot, source_name, target_name, LEFT_LINK_NAMES))
    for source_name, target_name in RIGHT_JOINT_NAMES.items():
        k1_robot.append(
            clone_wheel_joint(
                wheel_robot, source_name, target_name, RIGHT_LINK_NAMES, mirror_y=True
            )
        )

    apply_real_robot_joint_specs(k1_robot)

    ET.indent(k1_tree, space="  ")
    k1_tree.write(OUTPUT_URDF, encoding="utf-8", xml_declaration=True)
    with OUTPUT_URDF.open("ab") as output:
        output.write(b"\n")
    print(f"Wrote {OUTPUT_URDF}")


if __name__ == "__main__":
    build()
