#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import numpy as np
import trimesh


def round_to_unit_cells(
    requested_length: float,
    unit_cell_length: float,
) -> tuple[int, float]:
    """
    Round a requested dimension to the nearest integer number of unit cells.

    Python's built-in round() uses banker's rounding, so floor(x + 0.5)
    is used here for conventional nearest-integer rounding.
    """
    number_of_cells = max(
        1,
        math.floor(requested_length / unit_cell_length + 0.5),
    )

    actual_length = number_of_cells * unit_cell_length
    return number_of_cells, actual_length


def cylinder_between_points(
    start: np.ndarray,
    end: np.ndarray,
    diameter: float,
    sections: int = 16,
) -> trimesh.Trimesh:
    """
    Create a cylindrical beam between two 3D points.
    """
    direction = end - start
    length = np.linalg.norm(direction)

    if length <= 1e-12:
        raise ValueError("Beam start and end points must be different.")

    midpoint = (start + end) / 2.0

    beam = trimesh.creation.cylinder(
        radius=diameter / 2.0,
        height=length,
        sections=sections,
    )

    # trimesh cylinders are initially aligned with the local Z axis.
    transform = trimesh.geometry.align_vectors(
        np.array([0.0, 0.0, 1.0]),
        direction / length,
    )

    transform[:3, 3] = midpoint
    beam.apply_transform(transform)

    return beam


def create_bcc_lattice(
    requested_dimensions: tuple[float, float, float],
    unit_cell_length: float,
    beam_diameter: float,
    cylinder_sections: int = 16,
    add_node_spheres: bool = True,
    node_diameter: float | None = None,
) -> tuple[trimesh.Trimesh, tuple[float, float, float], tuple[int, int, int]]:
    """
    Generate a repeated BCC lattice.

    Each cell contains eight beams connecting the cell center to its
    eight corners.

    All dimensions are in millimetres.
    """
    if unit_cell_length <= 0:
        raise ValueError("Unit-cell length must be greater than zero.")

    if beam_diameter <= 0:
        raise ValueError("Beam diameter must be greater than zero.")

    if beam_diameter >= unit_cell_length:
        raise ValueError(
            "Beam diameter should be smaller than the unit-cell length."
        )

    cell_counts = []
    actual_dimensions = []

    for requested_length in requested_dimensions:
        if requested_length <= 0:
            raise ValueError("All requested dimensions must be positive.")

        count, actual_length = round_to_unit_cells(
            requested_length,
            unit_cell_length,
        )

        cell_counts.append(count)
        actual_dimensions.append(actual_length)

    nx, ny, nz = cell_counts

    meshes: list[trimesh.Trimesh] = []

    corner_offsets = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [0.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
        ],
        dtype=float,
    )

    corner_offsets *= unit_cell_length

    node_positions: set[tuple[float, float, float]] = set()

    for ix in range(nx):
        for iy in range(ny):
            for iz in range(nz):
                cell_origin = np.array(
                    [
                        ix * unit_cell_length,
                        iy * unit_cell_length,
                        iz * unit_cell_length,
                    ],
                    dtype=float,
                )

                cell_center = cell_origin + unit_cell_length / 2.0

                for corner_offset in corner_offsets:
                    corner = cell_origin + corner_offset

                    beam = cylinder_between_points(
                        start=cell_center,
                        end=corner,
                        diameter=beam_diameter,
                        sections=cylinder_sections,
                    )

                    meshes.append(beam)

                    if add_node_spheres:
                        node_positions.add(tuple(cell_center))
                        node_positions.add(tuple(corner))

    if add_node_spheres:
        if node_diameter is None:
            node_diameter = beam_diameter

        if node_diameter <= 0:
            raise ValueError("Node diameter must be greater than zero.")

        for position in node_positions:
            sphere = trimesh.creation.icosphere(
                subdivisions=2,
                radius=node_diameter / 2.0,
            )

            sphere.apply_translation(np.asarray(position))
            meshes.append(sphere)

    lattice = trimesh.util.concatenate(meshes)

    # Move the complete lattice so that it is centered at the origin.
    actual_dimensions_array = np.asarray(actual_dimensions)
    lattice.apply_translation(-actual_dimensions_array / 2.0)

    # Remove duplicate and unused mesh data where possible.
    lattice.merge_vertices()
    lattice.remove_unreferenced_vertices()

    return (
        lattice,
        tuple(actual_dimensions),
        tuple(cell_counts),
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate a body-centered cubic lattice STL."
    )

    parser.add_argument(
        "--length-x",
        type=float,
        default=30.0,
        help="Requested X dimension in mm. Default: 30",
    )

    parser.add_argument(
        "--length-y",
        type=float,
        default=30.0,
        help="Requested Y dimension in mm. Default: 30",
    )

    parser.add_argument(
        "--length-z",
        type=float,
        default=30.0,
        help="Requested Z dimension in mm. Default: 30",
    )

    parser.add_argument(
        "--cell-size",
        type=float,
        default=7.0,
        help="Cubic unit-cell side length in mm. Default: 7",
    )

    parser.add_argument(
        "--beam-diameter",
        type=float,
        default=1.0,
        help="Beam diameter in mm. Default: 1",
    )

    parser.add_argument(
        "--sections",
        type=int,
        default=16,
        help="Number of cylinder radial sections. Default: 16",
    )

    parser.add_argument(
        "--node-diameter",
        type=float,
        default=None,
        help=(
            "Diameter of spherical connection nodes in mm. "
            "Default: same as beam diameter"
        ),
    )

    parser.add_argument(
        "--no-node-spheres",
        action="store_true",
        help="Do not add spherical nodes at beam connections.",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("bcc_lattice.stl"),
        help="Output STL filename. Default: bcc_lattice.stl",
    )

    args = parser.parse_args()

    if args.sections < 3:
        raise ValueError("--sections must be at least 3.")

    requested_dimensions = (
        args.length_x,
        args.length_y,
        args.length_z,
    )

    lattice, actual_dimensions, cell_counts = create_bcc_lattice(
        requested_dimensions=requested_dimensions,
        unit_cell_length=args.cell_size,
        beam_diameter=args.beam_diameter,
        cylinder_sections=args.sections,
        add_node_spheres=not args.no_node_spheres,
        node_diameter=args.node_diameter,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    lattice.export(args.output)

    print(f"STL written to: {args.output}")
    print(
        "Requested dimensions: "
        f"{requested_dimensions[0]:.3f} × "
        f"{requested_dimensions[1]:.3f} × "
        f"{requested_dimensions[2]:.3f} mm"
    )
    print(
        "Actual dimensions:    "
        f"{actual_dimensions[0]:.3f} × "
        f"{actual_dimensions[1]:.3f} × "
        f"{actual_dimensions[2]:.3f} mm"
    )
    print(
        "Unit-cell counts:     "
        f"{cell_counts[0]} × "
        f"{cell_counts[1]} × "
        f"{cell_counts[2]}"
    )
    print(f"Beam diameter:       {args.beam_diameter:.3f} mm")
    print(f"Vertices:            {len(lattice.vertices)}")
    print(f"Faces:               {len(lattice.faces)}")


if __name__ == "__main__":
    main()