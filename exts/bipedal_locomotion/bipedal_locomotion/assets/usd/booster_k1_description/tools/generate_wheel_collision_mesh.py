#!/usr/bin/env python3
"""Generate one GPU-compatible convex shoulder mesh for the wheel tire."""

from __future__ import annotations

import math
import struct
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_MESH = ROOT / "meshes" / "wheelfoot" / "wheel_tire_collision.STL"
ANGULAR_SEGMENTS = 20

# Axial position and outer radius in metres. The mesh covers one shoulder from
# the outer edge toward the crown. A mirrored instance covers the other side.
AXIAL_PROFILE = (
    (-0.0135, 0.06875),
    (-0.0015, 0.07270),
    (0.0135, 0.07495),
)

Vector = tuple[float, float, float]
Triangle = tuple[Vector, Vector, Vector]


def subtract(left: Vector, right: Vector) -> Vector:
    return tuple(a - b for a, b in zip(left, right))


def triangle_normal(triangle: Triangle) -> Vector:
    edge_a = subtract(triangle[1], triangle[0])
    edge_b = subtract(triangle[2], triangle[0])
    cross = (
        edge_a[1] * edge_b[2] - edge_a[2] * edge_b[1],
        edge_a[2] * edge_b[0] - edge_a[0] * edge_b[2],
        edge_a[0] * edge_b[1] - edge_a[1] * edge_b[0],
    )
    length = math.sqrt(sum(value * value for value in cross))
    if length == 0.0:
        raise ValueError(f"degenerate collision triangle: {triangle}")
    return tuple(value / length for value in cross)


def make_rings() -> list[list[Vector]]:
    rings = []
    for y, radius in AXIAL_PROFILE:
        ring = []
        for index in range(ANGULAR_SEGMENTS):
            angle = math.tau * index / ANGULAR_SEGMENTS
            ring.append((radius * math.cos(angle), y, radius * math.sin(angle)))
        rings.append(ring)
    return rings


def make_triangles() -> list[Triangle]:
    rings = make_rings()
    triangles = []
    for ring_index in range(len(rings) - 1):
        lower = rings[ring_index]
        upper = rings[ring_index + 1]
        for index in range(ANGULAR_SEGMENTS):
            next_index = (index + 1) % ANGULAR_SEGMENTS
            triangles.append((lower[index], upper[index], upper[next_index]))
            triangles.append((lower[index], upper[next_index], lower[next_index]))

    for index in range(1, ANGULAR_SEGMENTS - 1):
        triangles.append((rings[0][0], rings[0][index], rings[0][index + 1]))
        triangles.append(
            (
                rings[-1][0],
                rings[-1][index + 1],
                rings[-1][index],
            )
        )
    return triangles


def generate_wheel_collision_mesh() -> int:
    triangles = make_triangles()
    header = b"Booster K1 convex wheel shoulder collision"
    with OUTPUT_MESH.open("wb") as output:
        output.write(header.ljust(80, b" "))
        output.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            values = triangle_normal(triangle) + tuple(
                coordinate for vertex in triangle for coordinate in vertex
            )
            output.write(struct.pack("<12fH", *values, 0))
    return len(triangles)


if __name__ == "__main__":
    triangle_count = generate_wheel_collision_mesh()
    print(f"Wrote {OUTPUT_MESH} with {triangle_count} triangles")
