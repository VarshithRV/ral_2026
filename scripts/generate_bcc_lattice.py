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

    # Trimesh cylinders are initially aligned with the local Z axis.
    transform = trimesh.geometry.align_vectors(
        np.array([0.0, 0.0, 1.0]),
        direction / length,
    )

    transform[:3, 3] = midpoint
    beam.apply_transform(transform)

    return beam


def create_bcc_lattice_with_sheets(
    requested_dimensions: tuple[float, float, float],
    unit_cell_length: float,
    beam_diameter: float,
    sheet_thickness: float,
    sheet_extra_size: float,
    cylinder_sections: int = 16,
    add_node_spheres: bool = True,
    node_diameter: float | None = None,
) -> tuple[
    trimesh.Trimesh,
    tuple[float, float, float],
    tuple[int, int, int],
]:
    """
    Generate a repeated BCC lattice with top and bottom sheets.

    Each BCC cell contains eight beams connecting the center of the cell
    to its eight corners.

    The sheets are centered relative to the lattice and extend beyond the
    lattice by sheet_extra_size overall in both X and Y.

    All dimensions are in millimetres.
    """
    if unit_cell_length <= 0:
        raise ValueError("Unit-cell length must be greater than zero.")

    if beam_diameter <= 0:
        raise ValueError("Beam diameter must be greater than zero.")

    if beam_diameter >= unit_cell_length:
        raise ValueError(
            "Beam diameter must be smaller than the unit-cell length."
        )

    if sheet_thickness <= 0:
        raise ValueError("Sheet thickness must be greater than zero.")

    if sheet_extra_size < 0:
        raise ValueError("Sheet extra size cannot be negative.")

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
    lattice_x, lattice_y, lattice_z = actual_dimensions

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

    # Sheet dimensions are larger than the lattice in X and Y.
    sheet_x = lattice_x + sheet_extra_size
    sheet_y = lattice_y + sheet_extra_size

    bottom_sheet = trimesh.creation.box(
        extents=[sheet_x, sheet_y, sheet_thickness]
    )

    top_sheet = trimesh.creation.box(
        extents=[sheet_x, sheet_y, sheet_thickness]
    )

    # The lattice currently occupies:
    # X: 0 to lattice_x
    # Y: 0 to lattice_y
    # Z: 0 to lattice_z
    #
    # Place the sheets directly below and above the lattice.
    bottom_sheet.apply_translation(
        [
            lattice_x / 2.0,
            lattice_y / 2.0,
            -sheet_thickness / 2.0,
        ]
    )

    top_sheet.apply_translation(
        [
            lattice_x / 2.0,
            lattice_y / 2.0,
            lattice_z + sheet_thickness / 2.0,
        ]
    )

    meshes.append(bottom_sheet)
    meshes.append(top_sheet)

    complete_part = trimesh.util.concatenate(meshes)

    # Center the complete part, including sheets, at the origin.
    total_z = lattice_z + 2.0 * sheet_thickness

    complete_part.apply_translation(
        [
            -lattice_x / 2.0,
            -lattice_y / 2.0,
            -lattice_z / 2.0,
        ]
    )

    complete_part.merge_vertices()
    complete_part.remove_unreferenced_vertices()

    return (
        complete_part,
        tuple(actual_dimensions),
        tuple(cell_counts),
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a BCC lattice STL with top and bottom sheets."
        )
    )

    parser.add_argument(
        "--length-x",
        type=float,
        default=30.0,
        help="Requested lattice X dimension in mm. Default: 30",
    )

    parser.add_argument(
        "--length-y",
        type=float,
        default=30.0,
        help="Requested lattice Y dimension in mm. Default: 30",
    )

    parser.add_argument(
        "--length-z",
        type=float,
        default=30.0,
        help="Requested lattice Z dimension in mm. Default: 30",
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
        "--sheet-thickness",
        type=float,
        default=0.5,
        help="Top and bottom sheet thickness in mm. Default: 0.5",
    )

    parser.add_argument(
        "--sheet-extra-size",
        type=float,
        default=1.0,
        help=(
            "Amount added to the total sheet length and width in mm. "
            "A value of 1 gives a 0.5 mm overhang on each side. "
            "Default: 1"
        ),
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
        default=Path("bcc_lattice_with_sheets.stl"),
        help=(
            "Output STL filename. "
            "Default: bcc_lattice_with_sheets.stl"
        ),
    )

    args = parser.parse_args()

    if args.sections < 3:
        raise ValueError("--sections must be at least 3.")

    requested_dimensions = (
        args.length_x,
        args.length_y,
        args.length_z,
    )

    complete_part, actual_dimensions, cell_counts = (
        create_bcc_lattice_with_sheets(
            requested_dimensions=requested_dimensions,
            unit_cell_length=args.cell_size,
            beam_diameter=args.beam_diameter,
            sheet_thickness=args.sheet_thickness,
            sheet_extra_size=args.sheet_extra_size,
            cylinder_sections=args.sections,
            add_node_spheres=not args.no_node_spheres,
            node_diameter=args.node_diameter,
        )
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    complete_part.export(args.output)

    sheet_x = actual_dimensions[0] + args.sheet_extra_size
    sheet_y = actual_dimensions[1] + args.sheet_extra_size

    total_height = (
        actual_dimensions[2] + 2.0 * args.sheet_thickness
    )

    print(f"STL written to: {args.output}")

    print(
        "Requested lattice dimensions: "
        f"{requested_dimensions[0]:.3f} × "
        f"{requested_dimensions[1]:.3f} × "
        f"{requested_dimensions[2]:.3f} mm"
    )

    print(
        "Actual lattice dimensions:    "
        f"{actual_dimensions[0]:.3f} × "
        f"{actual_dimensions[1]:.3f} × "
        f"{actual_dimensions[2]:.3f} mm"
    )

    print(
        "Unit-cell counts:             "
        f"{cell_counts[0]} × "
        f"{cell_counts[1]} × "
        f"{cell_counts[2]}"
    )

    print(
        "Sheet dimensions:             "
        f"{sheet_x:.3f} × "
        f"{sheet_y:.3f} × "
        f"{args.sheet_thickness:.3f} mm"
    )

    print(
        "Complete external dimensions: "
        f"{sheet_x:.3f} × "
        f"{sheet_y:.3f} × "
        f"{total_height:.3f} mm"
    )

    print(f"Beam diameter:                 {args.beam_diameter:.3f} mm")
    print(f"Vertices:                      {len(complete_part.vertices)}")
    print(f"Faces:                         {len(complete_part.faces)}")


if __name__ == "__main__":
    main()