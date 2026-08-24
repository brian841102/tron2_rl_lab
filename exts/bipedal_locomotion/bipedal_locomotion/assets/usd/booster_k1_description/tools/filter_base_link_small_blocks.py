#!/usr/bin/env python3
"""Create filtered full and side-specific base_link STL meshes."""

from __future__ import annotations

import struct
from collections import defaultdict, deque
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "meshes" / "wheelfoot"
SOURCE_MESH = MESH_DIR / "base_link.STL"
FILTERED_MESH = MESH_DIR / "base_link_no_small_blocks.STL"
RIGHT_SIDE_MESH = MESH_DIR / "base_link_right_side_no_small_blocks.STL"
LEFT_SIDE_MESH = MESH_DIR / "base_link_left_side_no_small_blocks.STL"
VERTEX_SCALE = 1_000_000


def read_binary_stl(path: Path) -> tuple[bytes, list[bytes], list[list[tuple[int, int, int]]]]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"not a binary STL: {path}")

    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * 50
    if expected_size > len(data):
        raise ValueError(f"binary STL triangle count is invalid: {path}")

    raw_triangles: list[bytes] = []
    triangle_vertices: list[list[tuple[int, int, int]]] = []
    for triangle_index in range(triangle_count):
        offset = 84 + triangle_index * 50
        raw = data[offset : offset + 50]
        values = struct.unpack_from("<9f", raw, 12)
        vertices = []
        for vertex_offset in range(0, 9, 3):
            vertices.append(
                tuple(round(values[vertex_offset + axis] * VERTEX_SCALE) for axis in range(3))
            )
        raw_triangles.append(raw)
        triangle_vertices.append(vertices)

    return data[:80], raw_triangles, triangle_vertices


def is_small_block(extents: tuple[float, float, float]) -> bool:
    four_by_eight_by_four = (
        0.0035 <= extents[0] <= 0.0045
        and 0.0075 <= extents[1] <= 0.0085
        and 0.0035 <= extents[2] <= 0.0045
    )
    ten_millimeter_origin_cube = (
        0.0095 <= extents[0] <= 0.0105
        and 0.0095 <= extents[1] <= 0.0105
        and 0.0095 <= extents[2] <= 0.0105
    )
    return four_by_eight_by_four or ten_millimeter_origin_cube


def connected_components(
    triangle_vertices: list[list[tuple[int, int, int]]],
) -> list[tuple[list[int], tuple[float, float, float], tuple[float, float, float]]]:
    vertex_to_triangles: dict[tuple[int, int, int], list[int]] = defaultdict(list)
    for triangle_index, vertices in enumerate(triangle_vertices):
        for vertex in vertices:
            vertex_to_triangles[vertex].append(triangle_index)

    seen = [False] * len(triangle_vertices)
    components = []
    for triangle_index in range(len(triangle_vertices)):
        if seen[triangle_index]:
            continue

        queue = deque([triangle_index])
        seen[triangle_index] = True
        component: list[int] = []
        mins = [float("inf"), float("inf"), float("inf")]
        maxs = [float("-inf"), float("-inf"), float("-inf")]

        while queue:
            current = queue.popleft()
            component.append(current)
            for vertex in triangle_vertices[current]:
                for axis in range(3):
                    value = vertex[axis] / VERTEX_SCALE
                    mins[axis] = min(mins[axis], value)
                    maxs[axis] = max(maxs[axis], value)

                for neighbor in vertex_to_triangles[vertex]:
                    if not seen[neighbor]:
                        seen[neighbor] = True
                        queue.append(neighbor)

        extents = tuple(maxs[axis] - mins[axis] for axis in range(3))
        center = tuple((mins[axis] + maxs[axis]) / 2 for axis in range(3))
        components.append((component, extents, center))

    return components


def find_small_block_triangles(triangle_vertices: list[list[tuple[int, int, int]]]) -> set[int]:
    removed: set[int] = set()
    for component, extents, _center in connected_components(triangle_vertices):
        if is_small_block(extents):
            removed.update(component)

    return removed


def find_right_side_base_triangles(triangle_vertices: list[list[tuple[int, int, int]]]) -> set[int]:
    kept: set[int] = set()
    for component, extents, center in connected_components(triangle_vertices):
        if center[1] > 0.0 and not is_small_block(extents):
            kept.update(component)

    if not kept:
        raise RuntimeError("no right-side base_link components found")
    return kept


def write_binary_stl(path: Path, header: bytes, triangles: list[bytes]) -> None:
    with path.open("wb") as file:
        file.write(header[:80].ljust(80, b" "))
        file.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            file.write(triangle)


def mirror_triangle_across_y(triangle: bytes) -> bytes:
    """Mirror one STL triangle across Y=0 while preserving outward winding."""
    if len(triangle) != 50:
        raise ValueError("binary STL triangle records must be 50 bytes")

    values = list(struct.unpack_from("<12f", triangle))
    for y_offset in (1, 4, 7, 10):
        values[y_offset] = -values[y_offset]

    # A reflection reverses orientation, so swap two vertices to restore it.
    values[3:6], values[6:9] = values[6:9], values[3:6]
    return struct.pack("<12f", *values) + triangle[48:50]


def filter_base_link_mesh() -> int:
    header, raw_triangles, triangle_vertices = read_binary_stl(SOURCE_MESH)
    removed = find_small_block_triangles(triangle_vertices)
    if not removed:
        raise RuntimeError("no isolated small-block components found in base_link.STL")

    kept_triangles = [
        triangle for triangle_index, triangle in enumerate(raw_triangles) if triangle_index not in removed
    ]
    write_binary_stl(FILTERED_MESH, header, kept_triangles)
    right_side = find_right_side_base_triangles(triangle_vertices)
    right_side_triangles = [
        triangle for triangle_index, triangle in enumerate(raw_triangles) if triangle_index in right_side
    ]
    write_binary_stl(RIGHT_SIDE_MESH, header, right_side_triangles)
    left_side_triangles = [mirror_triangle_across_y(triangle) for triangle in right_side_triangles]
    write_binary_stl(LEFT_SIDE_MESH, header, left_side_triangles)
    return len(removed)


if __name__ == "__main__":
    removed_triangles = filter_base_link_mesh()
    print(f"Wrote {FILTERED_MESH} after removing {removed_triangles} triangles")
    print(f"Wrote mirrored side meshes: {RIGHT_SIDE_MESH} and {LEFT_SIDE_MESH}")
