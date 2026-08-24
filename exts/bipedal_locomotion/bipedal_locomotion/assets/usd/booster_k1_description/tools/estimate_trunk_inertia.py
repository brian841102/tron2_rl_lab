#!/usr/bin/env python3
"""Estimate the stripped K1 wheel-foot trunk inertia from simple components.

The production robot described by ``k1_22dof.urdf`` has a 6.50 kg trunk.
That value represents the populated torso and cannot be scaled from the STL:
the closed trunk mesh has a volume of about 0.006385 m^3, so its old mass is
effectively a 1000 kg/m^3 solid-volume estimate.  The wheel-foot hardware has
only the outer shell, the two lower hip motors, and a small electronics set.

Until the assembled hardware can be weighed and pendulum-tested, this module
keeps the provisional estimate explicit and reproducible.  All primitives are
axis-aligned in the Trunk frame.  Dimensions and positions are in metres,
masses are in kilograms, and inertias are in kg m^2.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import prod


Vector3 = tuple[float, float, float]
Inertia6 = tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class Component:
    """A component with inertia about its own center of mass."""

    name: str
    mass: float
    center: Vector3
    inertia: Inertia6
    basis: str


@dataclass(frozen=True)
class CompositeInertia:
    mass: float
    center: Vector3
    inertia: Inertia6


def box_inertia(mass: float, size: Vector3) -> Inertia6:
    x, y, z = size
    return (
        mass * (y * y + z * z) / 12.0,
        0.0,
        0.0,
        mass * (x * x + z * z) / 12.0,
        0.0,
        mass * (x * x + y * y) / 12.0,
    )


def cylinder_y_inertia(mass: float, radius: float, length: float) -> Inertia6:
    transverse = mass * (3.0 * radius * radius + length * length) / 12.0
    axial = mass * radius * radius / 2.0
    return transverse, 0.0, 0.0, axial, 0.0, transverse


def hollow_box_component(
    name: str,
    outer_size: Vector3,
    thickness: float,
    density: float,
    center: Vector3,
    basis: str,
) -> Component:
    inner_size = tuple(dimension - 2.0 * thickness for dimension in outer_size)
    if min(inner_size) <= 0.0:
        raise ValueError("shell thickness must leave positive inner dimensions")

    outer_mass = density * prod(outer_size)
    inner_mass = density * prod(inner_size)
    outer_inertia = box_inertia(outer_mass, outer_size)
    inner_inertia = box_inertia(inner_mass, inner_size)
    inertia = tuple(outer - inner for outer, inner in zip(outer_inertia, inner_inertia))
    return Component(name, outer_mass - inner_mass, center, inertia, basis)


def combine(components: tuple[Component, ...]) -> CompositeInertia:
    mass = sum(component.mass for component in components)
    if mass <= 0.0:
        raise ValueError("composite mass must be positive")

    center = tuple(
        sum(component.mass * component.center[axis] for component in components) / mass
        for axis in range(3)
    )
    ixx = ixy = ixz = iyy = iyz = izz = 0.0
    for component in components:
        dx, dy, dz = tuple(
            component.center[axis] - center[axis] for axis in range(3)
        )
        c_ixx, c_ixy, c_ixz, c_iyy, c_iyz, c_izz = component.inertia
        ixx += c_ixx + component.mass * (dy * dy + dz * dz)
        ixy += c_ixy - component.mass * dx * dy
        ixz += c_ixz - component.mass * dx * dz
        iyy += c_iyy + component.mass * (dx * dx + dz * dz)
        iyz += c_iyz - component.mass * dy * dz
        izz += c_izz + component.mass * (dx * dx + dy * dy)

    return CompositeInertia(mass, center, (ixx, ixy, ixz, iyy, iyz, izz))


# The main collision box spans x=[-0.06, 0.06], y=[-0.09, 0.09], and
# z=[0.00, 0.20].  A 3 mm ABS shell at 1050 kg/m^3 is used as a provisional
# approximation for the retained outer case.
SHELL = hollow_box_component(
    "3 mm ABS outer shell",
    outer_size=(0.12, 0.18, 0.20),
    thickness=0.003,
    density=1050.0,
    center=(0.0, 0.0, 0.10),
    basis="main Trunk collision box and nominal ABS density",
)

# The lower connected component of k1_Trunk.STL spans roughly x=+/-0.0503,
# y=+/-0.0595, z=[-0.1095, -0.0065].  Two adjacent Y-axis cylinders reproduce
# that envelope.  Each 0.69 kg value is a conservative Trunk-side motor-stator
# allowance tied to the existing Left/Right_Hip_Pitch link mass.  The child
# links still retain their own 0.69 kg values, so this may double-count a motor
# if those values already describe the complete actuator.  In that case the
# measured actuator mass must be redistributed between parent and child links
# instead of being added to both.
HIP_MOTOR_RADIUS = 0.050
HIP_MOTOR_LENGTH = 0.0595
HIP_MOTOR_Z = -0.058
HIP_MOTOR_MASS = 0.69
LEFT_HIP_MOTOR = Component(
    "left hip stator allowance",
    HIP_MOTOR_MASS,
    (0.0, HIP_MOTOR_LENGTH / 2.0, HIP_MOTOR_Z),
    cylinder_y_inertia(HIP_MOTOR_MASS, HIP_MOTOR_RADIUS, HIP_MOTOR_LENGTH),
    "conservative Trunk-side allowance; calibrate parent/child mass split",
)
RIGHT_HIP_MOTOR = Component(
    "right hip stator allowance",
    HIP_MOTOR_MASS,
    (0.0, -HIP_MOTOR_LENGTH / 2.0, HIP_MOTOR_Z),
    cylinder_y_inertia(HIP_MOTOR_MASS, HIP_MOTOR_RADIUS, HIP_MOTOR_LENGTH),
    "conservative Trunk-side allowance; calibrate parent/child mass split",
)

# These two allowances represent the user's retained chips, wiring, and
# fasteners.  Their bottom-of-torso placement is intentionally conservative;
# replace the values with measured masses and positions when available.
ELECTRONICS = Component(
    "electronics",
    0.25,
    (0.0, 0.0, 0.015),
    box_inertia(0.25, (0.09, 0.14, 0.015)),
    "provisional retained-board allowance near the torso floor",
)
WIRING_AND_FASTENERS = Component(
    "wiring and fasteners",
    0.10,
    (0.0, 0.0, 0.040),
    box_inertia(0.10, (0.10, 0.16, 0.030)),
    "provisional distributed allowance",
)

TRUNK_COMPONENTS = (
    SHELL,
    LEFT_HIP_MOTOR,
    RIGHT_HIP_MOTOR,
    ELECTRONICS,
    WIRING_AND_FASTENERS,
)
TRUNK_ESTIMATE = combine(TRUNK_COMPONENTS)


def main() -> None:
    print("Provisional stripped Trunk component estimate")
    print("name                         mass (kg)       xyz in Trunk (m)")
    for component in TRUNK_COMPONENTS:
        x, y, z = component.center
        print(f"{component.name:28s} {component.mass:9.6f}  {x: .6f} {y: .6f} {z: .6f}")
    print(f"total                        {TRUNK_ESTIMATE.mass:9.6f}")
    print("COM xyz:", " ".join(f"{value:.12g}" for value in TRUNK_ESTIMATE.center))
    print(
        "inertia ixx ixy ixz iyy iyz izz:",
        " ".join(f"{value:.12g}" for value in TRUNK_ESTIMATE.inertia),
    )
    print("Status: provisional; replace with assembled mass/COM/pendulum measurements.")


if __name__ == "__main__":
    main()
