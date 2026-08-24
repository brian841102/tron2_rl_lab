#!/usr/bin/env python3
"""Verify the mirrored wheel-foot branches in the generated K1 URDF."""

from __future__ import annotations

import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from collections import defaultdict, deque


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_NAME = "booster_k1_description"
PACKAGE_URI = f"package://{PACKAGE_NAME}/"
WHEEL_MESH_URI = PACKAGE_URI + "meshes/wheelfoot/"
URDF = ROOT / "urdf" / "k1_wheelfoot.urdf"
WHEEL_MESH_DIR = ROOT / "meshes" / "wheelfoot"
BASE_LINK_MESH = WHEEL_MESH_DIR / "base_link.STL"
FILTERED_BASE_LINK_MESH = WHEEL_MESH_DIR / "base_link_no_small_blocks.STL"
POSITIVE_Y_BASE_LINK_MESH = WHEEL_MESH_DIR / "base_link_right_side_no_small_blocks.STL"
NEGATIVE_Y_BASE_LINK_MESH = WHEEL_MESH_DIR / "base_link_left_side_no_small_blocks.STL"
LEFT_KEEP_LINK_MESH = WHEEL_MESH_DIR / "right_leg_keep_pitch_Link.STL"
RIGHT_KEEP_LINK_MESH = WHEEL_MESH_DIR / "left_leg_keep_pitch_Link.STL"
LEFT_HIP_YAW_MESH = ROOT / "meshes" / "k1" / "k1_Left_Hip_Yaw.STL"
RIGHT_HIP_YAW_MESH = ROOT / "meshes" / "k1" / "k1_Right_Hip_Yaw.STL"
K1_LOGO_MESH = ROOT / "meshes" / "k1" / "k1_K1logo.STL"
WHEEL_COLLISION_MESH = WHEEL_MESH_DIR / "wheel_tire_collision.STL"
EXPECTED_TRUNK_INERTIAL = (
    (0.0, 0.0, -0.0102225444804),
    2.2254068,
    (0.0172050874479, 0.0, 0.0, 0.0150839433225, 0.0, 0.0064847859208),
)
EXPECTED_IMU_ORIGIN = (0.03, 0.0, 0.005)
EXPECTED_IMU_MASS = 0.01
EXPECTED_RIGHT_WHEELFOOT_INERTIALS = {
    "right_base_link": (
        (-2.28776125948645e-06, -0.0918110362128847, -6.12430857779765e-06),
        0.414,
        (
            0.000423687210687687,
            -2.51247343560104e-08,
            1.37015571348420e-07,
            0.000539097925330282,
            -2.77692372310688e-09,
            0.000423088036699582,
        ),
    ),
    "right_leg_keep_pitch_Link": (
        (0.00196628298771043, -0.0098922281432357, -0.0791439972707094),
        0.204,
        (
            0.000893653044644746,
            2.19954526020526e-08,
            -4.26860799781967e-07,
            0.000987780297269728,
            -2.26545952402249e-05,
            0.000104751237583495,
        ),
    ),
    "right_wheel_link": (
        (0.000141767, 0.021536494, 0.000158334),
        0.491,
        (0.000704842235587400, 0.0, 0.0, 0.001128650249777336, 0.0, 0.000700396681097755),
    ),
}
EXPECTED_TOTAL_MASS = 12.8534068
RIGHT_WHEEL_ORIGIN = (-0.000137649722258528, -0.00599999999995582, -0.174999945864445)
WHEEL_COLLISION_URI = WHEEL_MESH_URI + WHEEL_COLLISION_MESH.name
WHEEL_COLLISION_TRIANGLES = 116
WHEEL_CROWN_RADIUS = 0.075
WHEEL_CROWN_LENGTH = 0.006
WHEEL_SHOULDER_OFFSET = 0.0165
KNEE_CENTER_X = -0.014
KNEE_CENTER_Z = -0.117
GROOVE_INNER_RADIUS = 0.030
GROOVE_OUTER_RADIUS = 0.046
MAX_GROOVE_FIT_GAP = 0.0003
EXPECTED_REAL_ROBOT_JOINT_SPECS = {
    "ALeft_Shoulder_Pitch": ((0.0, -1.0, 0.0), -1.23, 2.96),
    "Left_Shoulder_Roll": ((-1.0, 0.0, 0.0), -1.66, 1.66),
    "Left_Elbow_Pitch": ((0.0, -1.0, 0.0), -1.92, 1.92),
    "Left_Elbow_Yaw": ((0.0, 0.0, -1.0), -0.82, 2.28),
    "ARight_Shoulder_Pitch": ((0.0, 1.0, 0.0), -2.96, 1.23),
    "Right_Shoulder_Roll": ((-1.0, 0.0, 0.0), -1.66, 1.66),
    "Right_Elbow_Pitch": ((0.0, 1.0, 0.0), -1.92, 1.92),
    "Right_Elbow_Yaw": ((0.0, 0.0, -1.0), -2.28, 0.80),
    "Left_Hip_Pitch": ((0.0, 1.0, 0.0), -2.99, 2.26),
    "Left_Hip_Roll": ((1.0, 0.0, 0.0), -0.42, 1.53),
    "Left_Hip_Yaw": ((0.0, 0.0, -1.0), -1.01, 1.09),
    "left_leg_keep_pitch_joint": ((0.0, -1.0, 0.0), -0.35, 2.0),
    "Right_Hip_Pitch": ((0.0, -1.0, 0.0), -2.25, 3.00),
    "Right_Hip_Roll": ((1.0, 0.0, 0.0), -1.58, 0.41),
    "Right_Hip_Yaw": ((0.0, 0.0, -1.0), -1.05, 1.05),
    "right_leg_keep_pitch_joint": ((0.0, -1.0, 0.0), -2.02, 0.33),
    "left_wheel_joint": ((0.0, 1.0, 0.0), None, None),
    "right_wheel_joint": ((0.0, 1.0, 0.0), None, None),
}
EXPECTED_REAL_ROBOT_JOINT_EFFORT_LIMITS = {
    "Left_Hip_Pitch": 30.0,
    "Left_Hip_Roll": 30.0,
    "Left_Hip_Yaw": 30.0,
    "left_leg_keep_pitch_joint": 30.0,
    "Right_Hip_Pitch": 30.0,
    "Right_Hip_Roll": 30.0,
    "Right_Hip_Yaw": 30.0,
    "right_leg_keep_pitch_joint": 30.0,
}


def numbers(value: str) -> tuple[float, ...]:
    return tuple(float(part) for part in value.split())


def close_tuple(actual: tuple[float, ...], expected: tuple[float, ...], tol: float = 1e-9) -> bool:
    return len(actual) == len(expected) and all(abs(a - e) <= tol for a, e in zip(actual, expected))


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def binary_stl_bounds(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    data = path.read_bytes()
    require(len(data) >= 84, f"not a binary STL: {path}")
    triangles = struct.unpack_from("<I", data, 80)[0]
    require(84 + triangles * 50 <= len(data), f"binary STL triangle count is invalid: {path}")

    mins = [float("inf"), float("inf"), float("inf")]
    maxs = [float("-inf"), float("-inf"), float("-inf")]
    for triangle_index in range(triangles):
        values = struct.unpack_from("<9f", data, 84 + triangle_index * 50 + 12)
        for vertex_offset in range(0, 9, 3):
            for axis in range(3):
                value = values[vertex_offset + axis]
                mins[axis] = min(mins[axis], value)
                maxs[axis] = max(maxs[axis], value)
    return tuple(mins), tuple(maxs)


def binary_stl_triangle_count(path: Path) -> int:
    data = path.read_bytes()
    require(len(data) >= 84, f"not a binary STL: {path}")
    triangles = struct.unpack_from("<I", data, 80)[0]
    require(84 + triangles * 50 == len(data), f"binary STL size is invalid: {path}")
    return triangles


def hip_yaw_groove_floor_depth(path: Path, outward_sign: int) -> float:
    data = path.read_bytes()
    triangle_count = binary_stl_triangle_count(path)
    vertices: set[tuple[float, float, float]] = set()
    for triangle_index in range(triangle_count):
        values = struct.unpack_from("<9f", data, 84 + triangle_index * 50 + 12)
        vertices.update(
            tuple(values[offset + axis] for axis in range(3))
            for offset in range(0, 9, 3)
        )

    def radial_distance(vertex: tuple[float, float, float]) -> float:
        return (
            (vertex[0] - KNEE_CENTER_X) ** 2
            + (vertex[2] - KNEE_CENTER_Z) ** 2
        ) ** 0.5

    floor_candidates = [
        outward_sign * vertex[1]
        for vertex in vertices
        if abs(radial_distance(vertex) - GROOVE_INNER_RADIUS) <= 5e-5
        and outward_sign * vertex[1] > 0.020
    ]
    require(len(floor_candidates) >= 7, f"could not identify the Hip_Yaw groove floor in {path}")
    floor_depth = min(floor_candidates)
    outer_ring_vertices = sum(
        abs(outward_sign * vertex[1] - floor_depth) <= 1e-7
        and abs(radial_distance(vertex) - GROOVE_OUTER_RADIUS) <= 1e-4
        for vertex in vertices
    )
    require(outer_ring_vertices >= 15, f"Hip_Yaw groove floor ring is incomplete in {path}")
    return floor_depth


def require_valid_convex_stl(path: Path) -> None:
    data = path.read_bytes()
    triangle_count = binary_stl_triangle_count(path)
    triangles = []
    stored_normals = []
    vertices = set()
    edge_counts: dict[tuple[tuple[float, ...], tuple[float, ...]], int] = defaultdict(int)

    for triangle_index in range(triangle_count):
        values = struct.unpack_from("<12f", data, 84 + triangle_index * 50)
        normal = values[0:3]
        triangle = (values[3:6], values[6:9], values[9:12])
        stored_normals.append(normal)
        triangles.append(triangle)
        vertices.update(triangle)
        for start, end in zip(triangle, triangle[1:] + triangle[:1]):
            edge_counts[tuple(sorted((start, end)))] += 1

    require(all(count == 2 for count in edge_counts.values()), f"collision mesh is not watertight: {path}")
    require(len(vertices) - len(edge_counts) + triangle_count == 2, f"collision mesh Euler characteristic is wrong: {path}")

    signed_volume_times_six = 0.0
    unit_normals = []
    for triangle, stored_normal in zip(triangles, stored_normals):
        a, b, c = triangle
        edge_a = tuple(b[axis] - a[axis] for axis in range(3))
        edge_b = tuple(c[axis] - a[axis] for axis in range(3))
        cross = (
            edge_a[1] * edge_b[2] - edge_a[2] * edge_b[1],
            edge_a[2] * edge_b[0] - edge_a[0] * edge_b[2],
            edge_a[0] * edge_b[1] - edge_a[1] * edge_b[0],
        )
        magnitude = sum(value * value for value in cross) ** 0.5
        require(magnitude > 1e-12, f"collision mesh contains a degenerate triangle: {path}")
        unit_normal = tuple(value / magnitude for value in cross)
        unit_normals.append(unit_normal)
        alignment = sum(actual * stored for actual, stored in zip(unit_normal, stored_normal))
        require(alignment >= 1.0 - 1e-5, f"collision mesh has an inconsistent stored normal: {path}")
        signed_volume_times_six += (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        )

    require(signed_volume_times_six > 0.0, f"collision mesh winding points inward: {path}")
    for triangle, normal in zip(triangles, unit_normals):
        origin = triangle[0]
        for vertex in vertices:
            plane_distance = sum(
                normal[axis] * (vertex[axis] - origin[axis]) for axis in range(3)
            )
            require(plane_distance <= 1e-7, f"collision mesh is not convex: {path}")


def small_block_component_count(path: Path) -> int:
    data = path.read_bytes()
    require(len(data) >= 84, f"not a binary STL: {path}")
    triangles = struct.unpack_from("<I", data, 80)[0]
    require(84 + triangles * 50 <= len(data), f"binary STL triangle count is invalid: {path}")

    scale = 1_000_000
    triangle_vertices = []
    vertex_to_triangles: dict[tuple[int, int, int], list[int]] = defaultdict(list)
    for triangle_index in range(triangles):
        values = struct.unpack_from("<9f", data, 84 + triangle_index * 50 + 12)
        vertices = []
        for vertex_offset in range(0, 9, 3):
            vertex = tuple(round(values[vertex_offset + axis] * scale) for axis in range(3))
            vertices.append(vertex)
            vertex_to_triangles[vertex].append(triangle_index)
        triangle_vertices.append(vertices)

    seen = [False] * triangles
    small_blocks = 0
    for triangle_index in range(triangles):
        if seen[triangle_index]:
            continue

        queue = deque([triangle_index])
        seen[triangle_index] = True
        mins = [float("inf"), float("inf"), float("inf")]
        maxs = [float("-inf"), float("-inf"), float("-inf")]
        while queue:
            current = queue.popleft()
            for vertex in triangle_vertices[current]:
                for axis in range(3):
                    value = vertex[axis] / scale
                    mins[axis] = min(mins[axis], value)
                    maxs[axis] = max(maxs[axis], value)
                for neighbor in vertex_to_triangles[vertex]:
                    if not seen[neighbor]:
                        seen[neighbor] = True
                        queue.append(neighbor)

        extents = tuple(maxs[axis] - mins[axis] for axis in range(3))
        if (
            0.0035 <= extents[0] <= 0.0045
            and 0.0075 <= extents[1] <= 0.0085
            and 0.0035 <= extents[2] <= 0.0045
        ) or (
            0.0095 <= extents[0] <= 0.0105
            and 0.0095 <= extents[1] <= 0.0105
            and 0.0095 <= extents[2] <= 0.0105
        ):
            small_blocks += 1

    return small_blocks


def component_y_centers(path: Path) -> list[float]:
    data = path.read_bytes()
    require(len(data) >= 84, f"not a binary STL: {path}")
    triangles = struct.unpack_from("<I", data, 80)[0]
    require(84 + triangles * 50 <= len(data), f"binary STL triangle count is invalid: {path}")

    scale = 1_000_000
    triangle_vertices = []
    vertex_to_triangles: dict[tuple[int, int, int], list[int]] = defaultdict(list)
    for triangle_index in range(triangles):
        values = struct.unpack_from("<9f", data, 84 + triangle_index * 50 + 12)
        vertices = []
        for vertex_offset in range(0, 9, 3):
            vertex = tuple(round(values[vertex_offset + axis] * scale) for axis in range(3))
            vertices.append(vertex)
            vertex_to_triangles[vertex].append(triangle_index)
        triangle_vertices.append(vertices)

    seen = [False] * triangles
    centers = []
    for triangle_index in range(triangles):
        if seen[triangle_index]:
            continue

        queue = deque([triangle_index])
        seen[triangle_index] = True
        min_y = float("inf")
        max_y = float("-inf")
        while queue:
            current = queue.popleft()
            for vertex in triangle_vertices[current]:
                y = vertex[1] / scale
                min_y = min(min_y, y)
                max_y = max(max_y, y)
                for neighbor in vertex_to_triangles[vertex]:
                    if not seen[neighbor]:
                        seen[neighbor] = True
                        queue.append(neighbor)

        centers.append((min_y + max_y) / 2)

    return centers


def mirror_y(values: tuple[float, ...]) -> tuple[float, ...]:
    require(len(values) == 3, f"expected xyz triple, got {values}")
    return values[0], -values[1], values[2]


def inertia_values(link: ET.Element) -> tuple[float, ...]:
    inertia = link.find("inertial/inertia")
    require(inertia is not None, f"link {link.attrib['name']} has no inertia tensor")
    return tuple(float(inertia.attrib[name]) for name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"))


def require_mirrored_link_dynamics(right: ET.Element, left: ET.Element) -> None:
    right_mass = float(right.find("inertial/mass").attrib["value"])
    left_mass = float(left.find("inertial/mass").attrib["value"])
    require(abs(right_mass - left_mass) <= 1e-12, f"{left.attrib['name']} mass is not mirrored")

    right_origin = numbers(right.find("inertial/origin").attrib["xyz"])
    left_origin = numbers(left.find("inertial/origin").attrib["xyz"])
    require(close_tuple(left_origin, mirror_y(right_origin), 1e-12), f"{left.attrib['name']} inertial origin is not mirrored")
    require(close_tuple(numbers(right.find("inertial/origin").attrib["rpy"]), (0.0, 0.0, 0.0)), f"{right.attrib['name']} inertial rotation is not zero")
    require(close_tuple(numbers(left.find("inertial/origin").attrib["rpy"]), (0.0, 0.0, 0.0)), f"{left.attrib['name']} inertial rotation is not zero")

    right_inertia = inertia_values(right)
    left_inertia = inertia_values(left)
    expected_left = (
        right_inertia[0],
        -right_inertia[1],
        right_inertia[2],
        right_inertia[3],
        -right_inertia[4],
        right_inertia[5],
    )
    require(close_tuple(left_inertia, expected_left, 1e-12), f"{left.attrib['name']} inertia tensor is not mirrored")


def require_physical_inertia(link: ET.Element) -> None:
    ixx, ixy, ixz, iyy, iyz, izz = inertia_values(link)
    leading_minor_2 = ixx * iyy - ixy * ixy
    determinant = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - ixz * iyz)
        + ixz * (ixy * iyz - ixz * iyy)
    )
    require(ixx > 0.0 and leading_minor_2 > 0.0 and determinant > 0.0, f"link {link.attrib['name']} inertia is not positive definite")
    require(ixx + iyy >= izz, f"link {link.attrib['name']} inertia violates Ixx + Iyy >= Izz")
    require(ixx + izz >= iyy, f"link {link.attrib['name']} inertia violates Ixx + Izz >= Iyy")
    require(iyy + izz >= ixx, f"link {link.attrib['name']} inertia violates Iyy + Izz >= Ixx")


def require_zero_geometry_origins(link: ET.Element) -> None:
    origins = link.findall("visual/origin") + link.findall("collision/origin")
    for origin in origins:
        require(
            close_tuple(numbers(origin.attrib["xyz"]), (0.0, 0.0, 0.0), 1e-12),
            f"link {link.attrib['name']} geometry origin is not zero",
        )
        require(
            close_tuple(numbers(origin.attrib["rpy"]), (0.0, 0.0, 0.0), 1e-12),
            f"link {link.attrib['name']} geometry rotation is not zero",
        )


def require_matching_joint_dynamics(right: ET.Element, left: ET.Element, expected_type: str) -> None:
    require(
        right.attrib["type"] == left.attrib["type"] == expected_type,
        f"wheel-foot joints must be {expected_type}",
    )
    right_origin = numbers(right.find("origin").attrib["xyz"])
    left_origin = numbers(left.find("origin").attrib["xyz"])
    require(close_tuple(left_origin, mirror_y(right_origin), 1e-12), f"joint {left.attrib['name']} origin is not mirrored")
    require(close_tuple(numbers(right.find("origin").attrib["rpy"]), (0.0, 0.0, 0.0)), f"joint {right.attrib['name']} rotation is not zero")
    require(close_tuple(numbers(left.find("origin").attrib["rpy"]), (0.0, 0.0, 0.0)), f"joint {left.attrib['name']} rotation is not zero")
    expected_axis = (0.0, 1.0, 0.0) if expected_type == "continuous" else (0.0, -1.0, 0.0)
    require(close_tuple(numbers(right.find("axis").attrib["xyz"]), expected_axis), f"joint {right.attrib['name']} axis is wrong")
    require(close_tuple(numbers(left.find("axis").attrib["xyz"]), expected_axis), f"joint {left.attrib['name']} axis is wrong")
    right_limit = right.find("limit").attrib
    left_limit = left.find("limit").attrib
    compared_attributes = ("effort", "velocity")
    for attribute in compared_attributes:
        require(abs(float(right_limit[attribute]) - float(left_limit[attribute])) <= 1e-12, f"joint {left.attrib['name']} {attribute} is not symmetric")
    if expected_type == "continuous":
        require("lower" not in right_limit and "upper" not in right_limit, f"joint {right.attrib['name']} must not have position limits")
        require("lower" not in left_limit and "upper" not in left_limit, f"joint {left.attrib['name']} must not have position limits")
    else:
        require(float(right_limit["lower"]) < float(right_limit["upper"]), f"joint {right.attrib['name']} limits must be ordered")
        require(float(left_limit["lower"]) < float(left_limit["upper"]), f"joint {left.attrib['name']} limits must be ordered")


def require_link_mesh(link: ET.Element, expected: str) -> None:
    visual = [mesh.attrib["filename"] for mesh in link.findall("visual/geometry/mesh")]
    collision = [mesh.attrib["filename"] for mesh in link.findall("collision/geometry/mesh")]
    require(visual == [expected], f"link {link.attrib['name']} visual mesh is wrong: {visual}")
    require(collision == [expected], f"link {link.attrib['name']} collision mesh is wrong: {collision}")


def require_wheel_geometry(link: ET.Element, visual_mesh: str, center_y: float) -> None:
    visual = [mesh.attrib["filename"] for mesh in link.findall("visual/geometry/mesh")]
    require(visual == [visual_mesh], f"link {link.attrib['name']} visual mesh is wrong: {visual}")

    collisions = link.findall("collision")
    require(len(collisions) == 3, f"link {link.attrib['name']} must have one crown and two shoulder colliders")

    crown = collisions[0]
    crown_origin = crown.find("origin")
    cylinder = crown.find("geometry/cylinder")
    require(crown_origin is not None and cylinder is not None, f"link {link.attrib['name']} crown collider is wrong")
    require(close_tuple(numbers(crown_origin.attrib["xyz"]), (0.0, center_y, 0.0), 1e-9), f"link {link.attrib['name']} crown origin is wrong")
    require(close_tuple(numbers(crown_origin.attrib["rpy"]), (1.5707963267948966, 0.0, 0.0), 1e-9), f"link {link.attrib['name']} crown rotation is wrong")
    require(abs(float(cylinder.attrib["radius"]) - WHEEL_CROWN_RADIUS) <= 1e-12, f"link {link.attrib['name']} crown radius is wrong")
    require(abs(float(cylinder.attrib["length"]) - WHEEL_CROWN_LENGTH) <= 1e-12, f"link {link.attrib['name']} crown length is wrong")

    expected_shoulders = (
        ((0.0, center_y - WHEEL_SHOULDER_OFFSET, 0.0), (0.0, 0.0, 0.0)),
        ((0.0, center_y + WHEEL_SHOULDER_OFFSET, 0.0), (3.141592653589793, 0.0, 0.0)),
    )
    for shoulder, (expected_xyz, expected_rpy) in zip(collisions[1:], expected_shoulders):
        origin = shoulder.find("origin")
        collision_mesh = shoulder.find("geometry/mesh")
        require(origin is not None and collision_mesh is not None, f"link {link.attrib['name']} shoulder collider is wrong")
        require(collision_mesh.attrib["filename"] == WHEEL_COLLISION_URI, f"link {link.attrib['name']} shoulder mesh is wrong")
        require(close_tuple(numbers(origin.attrib["xyz"]), expected_xyz, 1e-9), f"link {link.attrib['name']} shoulder origin is wrong")
        require(close_tuple(numbers(origin.attrib["rpy"]), expected_rpy, 1e-9), f"link {link.attrib['name']} shoulder rotation is wrong")


def main() -> int:
    require(URDF.exists(), f"missing generated URDF: {URDF}")
    require(FILTERED_BASE_LINK_MESH.exists(), f"missing filtered base link mesh: {FILTERED_BASE_LINK_MESH}")
    require(POSITIVE_Y_BASE_LINK_MESH.exists(), f"missing positive-Y base link mesh: {POSITIVE_Y_BASE_LINK_MESH}")
    require(NEGATIVE_Y_BASE_LINK_MESH.exists(), f"missing negative-Y base link mesh: {NEGATIVE_Y_BASE_LINK_MESH}")
    require(LEFT_KEEP_LINK_MESH.exists(), f"missing left keep-pitch mesh: {LEFT_KEEP_LINK_MESH}")
    require(RIGHT_KEEP_LINK_MESH.exists(), f"missing right keep-pitch mesh: {RIGHT_KEEP_LINK_MESH}")
    require(LEFT_HIP_YAW_MESH.exists(), f"missing left Hip_Yaw mesh: {LEFT_HIP_YAW_MESH}")
    require(RIGHT_HIP_YAW_MESH.exists(), f"missing right Hip_Yaw mesh: {RIGHT_HIP_YAW_MESH}")
    require(K1_LOGO_MESH.exists(), f"missing K1 logo mesh: {K1_LOGO_MESH}")
    require(WHEEL_COLLISION_MESH.exists(), f"missing wheel collision mesh: {WHEEL_COLLISION_MESH}")
    require(binary_stl_triangle_count(WHEEL_COLLISION_MESH) == WHEEL_COLLISION_TRIANGLES, "wheel collision mesh triangle count is wrong")
    require_valid_convex_stl(WHEEL_COLLISION_MESH)
    collision_min, collision_max = binary_stl_bounds(WHEEL_COLLISION_MESH)
    require(close_tuple(collision_min, (-0.07495, -0.0135, -0.07495), 1e-7), "wheel collision mesh minimum bounds are wrong")
    require(close_tuple(collision_max, (0.07495, 0.0135, 0.07495), 1e-7), "wheel collision mesh maximum bounds are wrong")
    require(small_block_component_count(BASE_LINK_MESH) > 0, "original base_link mesh should contain small blocks")
    require(small_block_component_count(FILTERED_BASE_LINK_MESH) == 0, "filtered base_link mesh still contains 4x8x4mm blocks")
    require(small_block_component_count(POSITIVE_Y_BASE_LINK_MESH) == 0, "positive-Y base_link mesh still contains small blocks")
    require(small_block_component_count(NEGATIVE_Y_BASE_LINK_MESH) == 0, "negative-Y base_link mesh still contains small blocks")

    tree = ET.parse(URDF)
    robot = tree.getroot()

    links = {link.attrib["name"]: link for link in robot.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    require(
        set(EXPECTED_REAL_ROBOT_JOINT_SPECS).issubset(joints),
        f"missing measured hardware joints: {set(EXPECTED_REAL_ROBOT_JOINT_SPECS) - set(joints)}",
    )
    for name, (expected_axis, expected_lower, expected_upper) in EXPECTED_REAL_ROBOT_JOINT_SPECS.items():
        joint = joints[name]
        axis = joint.find("axis")
        limit = joint.find("limit")
        require(axis is not None, f"joint {name} has no axis")
        require(limit is not None, f"joint {name} has no limit")
        require(
            close_tuple(numbers(axis.attrib["xyz"]), expected_axis),
            f"joint {name} axis does not match the real robot",
        )
        if expected_lower is None or expected_upper is None:
            require(
                "lower" not in limit.attrib and "upper" not in limit.attrib,
                f"continuous joint {name} must not have position limits",
            )
        else:
            require(
                abs(float(limit.attrib["lower"]) - expected_lower) <= 1e-12,
                f"joint {name} lower limit does not match the real robot",
            )
            require(
                abs(float(limit.attrib["upper"]) - expected_upper) <= 1e-12,
                f"joint {name} upper limit does not match the real robot",
            )
    require(
        set(EXPECTED_REAL_ROBOT_JOINT_EFFORT_LIMITS).issubset(joints),
        "missing joints with measured effort limits: "
        f"{set(EXPECTED_REAL_ROBOT_JOINT_EFFORT_LIMITS) - set(joints)}",
    )
    for name, expected_effort in EXPECTED_REAL_ROBOT_JOINT_EFFORT_LIMITS.items():
        limit = joints[name].find("limit")
        require(limit is not None, f"joint {name} has no limit")
        require(
            abs(float(limit.attrib["effort"]) - expected_effort) <= 1e-12,
            f"joint {name} effort limit does not match the real robot",
        )
    positive_base_min, positive_base_max = binary_stl_bounds(POSITIVE_Y_BASE_LINK_MESH)
    negative_base_min, negative_base_max = binary_stl_bounds(NEGATIVE_Y_BASE_LINK_MESH)
    logo_min, logo_max = binary_stl_bounds(K1_LOGO_MESH)
    require(logo_min[0] > 0.06, f"K1 logo must identify the +X front face: {logo_min}, {logo_max}")

    removed_links = {
        "Head_2",
        "Left_Shank",
        "Left_Ankle_Cross",
        "left_foot_link",
        "Right_Shank",
        "Right_Ankle_Cross",
        "right_foot_link",
        "base_link",
    }
    removed_joints = {
        "Head_pitch",
        "Left_Knee_Pitch",
        "Left_Ankle_Pitch",
        "Left_Ankle_Roll",
        "Right_Knee_Pitch",
        "Right_Ankle_Pitch",
        "Right_Ankle_Roll",
    }
    added_links = {
        "imu_link",
        "right_base_link",
        "right_leg_keep_pitch_Link",
        "right_wheel_link",
        "left_base_link",
        "left_leg_keep_pitch_Link",
        "left_wheel_link",
    }
    added_joints = {
        "Trunk_to_imu",
        "Right_Knee_Wheelfoot_Base",
        "right_leg_keep_pitch_joint",
        "right_wheel_joint",
        "Left_Knee_Wheelfoot_Base",
        "left_leg_keep_pitch_joint",
        "left_wheel_joint",
    }

    require(removed_links.isdisjoint(links), f"old lower-leg links still present: {removed_links & links.keys()}")
    require(removed_joints.isdisjoint(joints), f"old lower-leg joints still present: {removed_joints & joints.keys()}")
    require(added_links.issubset(links), f"missing wheel-foot links: {added_links - links.keys()}")
    require(added_joints.issubset(joints), f"missing wheel-foot joints: {added_joints - joints.keys()}")

    require(len(links) == len(robot.findall("link")), "duplicate link names detected")
    require(len(joints) == len(robot.findall("joint")), "duplicate joint names detected")
    require(len(links) == 23, f"expected 23 links, found {len(links)}")
    require(len(joints) == 22, f"expected 22 joints, found {len(joints)}")

    trunk = links["Trunk"]
    imu_link = links["imu_link"]
    imu_joint = joints["Trunk_to_imu"]
    expected_trunk_origin, expected_trunk_mass, expected_trunk_inertia = EXPECTED_TRUNK_INERTIAL
    require(
        close_tuple(numbers(trunk.find("inertial/origin").attrib["xyz"]), expected_trunk_origin, 1e-12),
        "Trunk COM does not match the stripped-hardware estimate",
    )
    require(
        abs(float(trunk.find("inertial/mass").attrib["value"]) - expected_trunk_mass) <= 1e-12,
        "Trunk mass does not match the stripped-hardware estimate",
    )
    require(
        close_tuple(inertia_values(trunk), expected_trunk_inertia, 1e-12),
        "Trunk inertia does not match the stripped-hardware estimate",
    )
    require_physical_inertia(trunk)

    require(imu_joint.attrib["type"] == "fixed", "Trunk_to_imu must be fixed")
    require(imu_joint.find("parent").attrib["link"] == "Trunk", "IMU parent must be Trunk")
    require(imu_joint.find("child").attrib["link"] == "imu_link", "IMU joint child is wrong")
    imu_origin = numbers(imu_joint.find("origin").attrib["xyz"])
    require(close_tuple(imu_origin, EXPECTED_IMU_ORIGIN, 1e-12), "IMU mount xyz is wrong")
    require(
        close_tuple(numbers(imu_joint.find("origin").attrib["rpy"]), (0.0, 0.0, 0.0), 1e-12),
        "IMU axes must match the Trunk axes",
    )
    require(abs(0.06 - imu_origin[0] - 0.03) <= 1e-12, "IMU must be 3 cm behind the +X front face")
    require(abs(float(imu_link.find("inertial/mass").attrib["value"]) - EXPECTED_IMU_MASS) <= 1e-12, "IMU mass is wrong")
    imu_visuals = imu_link.findall("visual")
    require(len(imu_visuals) == 1, "IMU link must have one visual package")
    require(
        close_tuple(numbers(imu_visuals[0].find("geometry/box").attrib["size"]), (0.03, 0.02, 0.005), 1e-12),
        "IMU visual package size is wrong",
    )
    require(not imu_link.findall("collision"), "IMU frame must not add collision geometry")
    require_physical_inertia(imu_link)

    right_fixed = joints["Right_Knee_Wheelfoot_Base"]
    require(right_fixed.attrib["type"] == "fixed", "Right_Knee_Wheelfoot_Base must be fixed")
    require(right_fixed.find("parent").attrib["link"] == "Right_Hip_Yaw", "right wheel-foot base must attach to Right_Hip_Yaw")
    require(right_fixed.find("child").attrib["link"] == "right_base_link", "right fixed joint child must be right_base_link")
    require(close_tuple(numbers(right_fixed.find("origin").attrib["xyz"]), (-0.014, 0.096, -0.117), 1e-7), "right fixed joint xyz is wrong")
    require(close_tuple(numbers(right_fixed.find("origin").attrib["rpy"]), (0.0, 0.0, 0.0)), "right fixed joint rpy is wrong")

    left_fixed = joints["Left_Knee_Wheelfoot_Base"]
    require(left_fixed.attrib["type"] == "fixed", "Left_Knee_Wheelfoot_Base must be fixed")
    require(left_fixed.find("parent").attrib["link"] == "Left_Hip_Yaw", "left wheel-foot base must attach to Left_Hip_Yaw")
    require(left_fixed.find("child").attrib["link"] == "left_base_link", "left fixed joint child must be left_base_link")
    require(close_tuple(numbers(left_fixed.find("origin").attrib["xyz"]), (-0.014, -0.096, -0.117), 1e-7), "left fixed joint xyz is wrong")
    require(close_tuple(numbers(left_fixed.find("origin").attrib["rpy"]), (0.0, 0.0, 0.0)), "left fixed joint rpy is wrong")

    right_keep = joints["right_leg_keep_pitch_joint"]
    left_keep = joints["left_leg_keep_pitch_joint"]
    require(right_keep.find("parent").attrib["link"] == "right_base_link", "right keep-pitch parent is wrong")
    require(right_keep.find("child").attrib["link"] == "right_leg_keep_pitch_Link", "right keep-pitch child is wrong")
    require(left_keep.find("parent").attrib["link"] == "left_base_link", "left keep-pitch parent is wrong")
    require(left_keep.find("child").attrib["link"] == "left_leg_keep_pitch_Link", "left keep-pitch child is wrong")
    require(close_tuple(numbers(right_keep.find("origin").attrib["xyz"]), (0.0, -0.122, 0.0)), "right keep-pitch origin is wrong")
    require_matching_joint_dynamics(right_keep, left_keep, "revolute")

    right_wheel = joints["right_wheel_joint"]
    left_wheel = joints["left_wheel_joint"]
    require(right_wheel.find("parent").attrib["link"] == "right_leg_keep_pitch_Link", "right wheel parent is wrong")
    require(right_wheel.find("child").attrib["link"] == "right_wheel_link", "right wheel child is wrong")
    require(left_wheel.find("parent").attrib["link"] == "left_leg_keep_pitch_Link", "left wheel parent is wrong")
    require(left_wheel.find("child").attrib["link"] == "left_wheel_link", "left wheel child is wrong")
    require(close_tuple(numbers(right_wheel.find("origin").attrib["xyz"]), RIGHT_WHEEL_ORIGIN, 1e-12), "right wheel origin is wrong")
    require_matching_joint_dynamics(right_wheel, left_wheel, "continuous")

    right_base = links["right_base_link"]
    left_base = links["left_base_link"]
    right_keep_link = links["right_leg_keep_pitch_Link"]
    left_keep_link = links["left_leg_keep_pitch_Link"]
    right_wheel_link = links["right_wheel_link"]
    left_wheel_link = links["left_wheel_link"]

    total_mass = sum(float(link.find("inertial/mass").attrib["value"]) for link in links.values())
    require(abs(total_mass - EXPECTED_TOTAL_MASS) <= 1e-12, f"unexpected total mass: {total_mass}")

    for name, (expected_origin, expected_mass, expected_inertia) in EXPECTED_RIGHT_WHEELFOOT_INERTIALS.items():
        link = links[name]
        require(
            close_tuple(numbers(link.find("inertial/origin").attrib["xyz"]), expected_origin, 1e-12),
            f"link {name} inertial origin is wrong",
        )
        require(
            abs(float(link.find("inertial/mass").attrib["value"]) - expected_mass) <= 1e-12,
            f"link {name} mass is wrong",
        )
        require(
            close_tuple(inertia_values(link), expected_inertia, 1e-15),
            f"link {name} inertia is wrong",
        )

    for right_link, left_link in (
        (right_base, left_base),
        (right_keep_link, left_keep_link),
        (right_wheel_link, left_wheel_link),
    ):
        require_mirrored_link_dynamics(right_link, left_link)
        require_physical_inertia(right_link)
        require_physical_inertia(left_link)

    for link in (right_base, left_base, right_keep_link, left_keep_link):
        require_zero_geometry_origins(link)

    require_wheel_geometry(
        right_wheel_link,
        WHEEL_MESH_URI + "left_leg_anker_pitch_Link.STL",
        0.032,
    )
    require_wheel_geometry(
        left_wheel_link,
        WHEEL_MESH_URI + "right_leg_anker_pitch_Link.STL",
        -0.032,
    )

    expected_negative_min = (positive_base_min[0], -positive_base_max[1], positive_base_min[2])
    expected_negative_max = (positive_base_max[0], -positive_base_min[1], positive_base_max[2])
    require(close_tuple(negative_base_min, expected_negative_min, 1e-12), "negative-Y base mesh minimum bounds are not mirrored")
    require(close_tuple(negative_base_max, expected_negative_max, 1e-12), "negative-Y base mesh maximum bounds are not mirrored")
    require(positive_base_min[1] > 0.0, "positive-Y base mesh includes negative-Y geometry")
    require(negative_base_max[1] < 0.0, "negative-Y base mesh includes positive-Y geometry")
    require(all(center > 0.0 for center in component_y_centers(POSITIVE_Y_BASE_LINK_MESH)), "positive-Y base mesh includes negative-Y components")
    require(all(center < 0.0 for center in component_y_centers(NEGATIVE_Y_BASE_LINK_MESH)), "negative-Y base mesh includes positive-Y components")

    right_fixed_xyz = numbers(right_fixed.find("origin").attrib["xyz"])
    left_fixed_xyz = numbers(left_fixed.find("origin").attrib["xyz"])
    right_keep_xyz = numbers(right_keep.find("origin").attrib["xyz"])
    left_keep_xyz = numbers(left_keep.find("origin").attrib["xyz"])
    right_wheel_xyz = numbers(right_wheel.find("origin").attrib["xyz"])
    left_wheel_xyz = numbers(left_wheel.find("origin").attrib["xyz"])

    right_wheel_center_y = right_fixed_xyz[1] + right_keep_xyz[1] + right_wheel_xyz[1] + 0.032
    left_wheel_center_y = left_fixed_xyz[1] + left_keep_xyz[1] + left_wheel_xyz[1] - 0.032
    require(abs(right_wheel_center_y) <= 1e-9, f"right tire center misses the Hip_Yaw plane: {right_wheel_center_y}")
    require(abs(left_wheel_center_y) <= 1e-9, f"left tire center misses the Hip_Yaw plane: {left_wheel_center_y}")

    right_keep_mesh_min, right_keep_mesh_max = binary_stl_bounds(RIGHT_KEEP_LINK_MESH)
    left_keep_mesh_min, left_keep_mesh_max = binary_stl_bounds(LEFT_KEEP_LINK_MESH)
    right_keep_bounds_y = (
        right_fixed_xyz[1] + right_keep_xyz[1] + right_keep_mesh_min[1],
        right_fixed_xyz[1] + right_keep_xyz[1] + right_keep_mesh_max[1],
    )
    left_keep_bounds_y = (
        left_fixed_xyz[1] + left_keep_xyz[1] + left_keep_mesh_min[1],
        left_fixed_xyz[1] + left_keep_xyz[1] + left_keep_mesh_max[1],
    )
    require(close_tuple(right_keep_bounds_y, (-0.044, -0.024), 2e-6), f"right keep-pitch bracket is not in the outer groove: {right_keep_bounds_y}")
    require(close_tuple(left_keep_bounds_y, (0.024, 0.044), 2e-6), f"left keep-pitch bracket is not in the outer groove: {left_keep_bounds_y}")
    require(close_tuple(left_keep_bounds_y, tuple(-value for value in reversed(right_keep_bounds_y)), 1e-9), "keep-pitch bracket envelopes are not mirrored")

    right_hip_min, right_hip_max = binary_stl_bounds(RIGHT_HIP_YAW_MESH)
    left_hip_min, left_hip_max = binary_stl_bounds(LEFT_HIP_YAW_MESH)
    require(right_keep_bounds_y[0] <= right_hip_min[1] <= right_keep_bounds_y[1], "right keep-pitch bracket does not cover the outer Hip_Yaw groove lip")
    require(left_keep_bounds_y[0] <= left_hip_max[1] <= left_keep_bounds_y[1], "left keep-pitch bracket does not cover the outer Hip_Yaw groove lip")
    right_groove_depth = hip_yaw_groove_floor_depth(RIGHT_HIP_YAW_MESH, -1)
    left_groove_depth = hip_yaw_groove_floor_depth(LEFT_HIP_YAW_MESH, 1)
    require(abs(right_groove_depth - left_groove_depth) <= 1e-7, "Hip_Yaw groove floors are not mirrored")
    right_inner_face_depth = -right_keep_bounds_y[1]
    left_inner_face_depth = left_keep_bounds_y[0]
    require(abs(right_inner_face_depth - right_groove_depth) <= MAX_GROOVE_FIT_GAP, f"right keep-pitch bracket is not tight to the groove floor: {right_inner_face_depth} vs {right_groove_depth}")
    require(abs(left_inner_face_depth - left_groove_depth) <= MAX_GROOVE_FIT_GAP, f"left keep-pitch bracket is not tight to the groove floor: {left_inner_face_depth} vs {left_groove_depth}")

    parent_counts: dict[str, int] = {}
    children_by_parent: dict[str, list[str]] = {name: [] for name in links}
    for joint in robot.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        require(parent in links, f"joint {joint.attrib['name']} references missing parent link {parent}")
        require(child in links, f"joint {joint.attrib['name']} references missing child link {child}")
        parent_counts[child] = parent_counts.get(child, 0) + 1
        children_by_parent[parent].append(child)
    repeated = {child: count for child, count in parent_counts.items() if count > 1}
    require(not repeated, f"URDF tree has links with multiple parents: {repeated}")
    roots = set(links) - set(parent_counts)
    require(roots == {"Trunk"}, f"URDF must have only Trunk as root, found {roots}")

    reachable: set[str] = set()
    stack = ["Trunk"]
    while stack:
        current = stack.pop()
        require(current not in reachable, f"URDF contains a cycle through {current}")
        reachable.add(current)
        stack.extend(children_by_parent[current])
    require(reachable == set(links), f"URDF contains unreachable links: {set(links) - reachable}")
    require(sum(joint.attrib["type"] == "fixed" for joint in joints.values()) == 3, "expected two wheel-base joints and one IMU fixed joint")
    require(sum(joint.attrib["type"] == "revolute" for joint in joints.values()) == 17, "expected 17 revolute joints after removing Head_pitch")
    require(sum(joint.attrib["type"] == "continuous" for joint in joints.values()) == 2, "expected two continuous wheel joints")
    require("base_link_left_side_fixed_joint" not in joints, "source base-to-base fixed joint must not be imported")

    mesh_filenames = [
        mesh.attrib["filename"]
        for mesh in robot.findall(".//mesh")
        if "filename" in mesh.attrib
    ]
    expected_wheel_meshes = {
        "right_base_link": WHEEL_MESH_URI + "base_link_left_side_no_small_blocks.STL",
        "left_base_link": WHEEL_MESH_URI + "base_link_right_side_no_small_blocks.STL",
        "right_leg_keep_pitch_Link": WHEEL_MESH_URI + "left_leg_keep_pitch_Link.STL",
        "left_leg_keep_pitch_Link": WHEEL_MESH_URI + "right_leg_keep_pitch_Link.STL",
        "right_wheel_link": WHEEL_MESH_URI + "left_leg_anker_pitch_Link.STL",
        "left_wheel_link": WHEEL_MESH_URI + "right_leg_anker_pitch_Link.STL",
    }
    for link_name, mesh_name in expected_wheel_meshes.items():
        if link_name not in {"right_wheel_link", "left_wheel_link"}:
            require_link_mesh(links[link_name], mesh_name)
        relative_path = mesh_name.removeprefix(PACKAGE_URI)
        require((ROOT / relative_path).exists(), f"missing wheel-foot mesh: {mesh_name}")

    require(
        WHEEL_MESH_URI + "base_link.STL" not in mesh_filenames,
        "generated preview URDF should not reference the unfiltered base_link mesh",
    )
    require(
        WHEEL_MESH_URI + "base_link_no_small_blocks.STL" not in mesh_filenames,
        "generated preview URDF should not use the full pair base_link mesh",
    )
    require(
        all(name.startswith(PACKAGE_URI) for name in mesh_filenames),
        f"all meshes must use {PACKAGE_URI} URIs",
    )

    print(f"OK: {URDF.name} mirrored wheel-foot structure is valid")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
