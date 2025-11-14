#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArUco Marker Generator.

Generates printable ArUco markers for calibration and robot tracking.
"""

import cv2
import numpy as np
import argparse
from pathlib import Path


def generate_marker(
    marker_id: int,
    size_pixels: int = 200,
    dict_type=cv2.aruco.DICT_4X4_50,
    output_path: str = None,
    border_bits: int = 1
) -> np.ndarray:
    """
    Generate an ArUco marker image.

    Args:
        marker_id: ID of the marker to generate (0-49 for 4x4_50 dictionary)
        size_pixels: Size of the output image in pixels
        dict_type: ArUco dictionary type
        output_path: Optional path to save the marker image
        border_bits: Number of white border bits around the marker

    Returns:
        Generated marker image as numpy array
    """
    # Get dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)

    # Generate marker
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size_pixels, borderBits=border_bits)

    # Save if output path provided
    if output_path:
        cv2.imwrite(output_path, marker_img)
        print(f"Marker {marker_id} saved to {output_path}")

    return marker_img


def generate_marker_set(
    marker_ids: list,
    size_pixels: int = 200,
    dict_type=cv2.aruco.DICT_4X4_50,
    output_dir: str = "markers",
    border_bits: int = 1
):
    """
    Generate a set of ArUco markers.

    Args:
        marker_ids: List of marker IDs to generate
        size_pixels: Size of each marker in pixels
        dict_type: ArUco dictionary type
        output_dir: Directory to save markers
        border_bits: Number of white border bits
    """
    # Create output directory
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    # Generate each marker
    for marker_id in marker_ids:
        filename = output_path / f"marker_{marker_id}.png"
        generate_marker(
            marker_id,
            size_pixels=size_pixels,
            dict_type=dict_type,
            output_path=str(filename),
            border_bits=border_bits
        )

    print(f"Generated {len(marker_ids)} markers in {output_dir}/")


def main():
    """Command-line interface for marker generation."""
    parser = argparse.ArgumentParser(description="Generate ArUco markers for soccer field SLAM")

    parser.add_argument(
        '--ids',
        type=int,
        nargs='+',
        default=[0, 1, 2, 3, 10, 11, 12, 13],
        help='Marker IDs to generate (default: 0-3 for corners, 10-13 for Spheros)'
    )
    parser.add_argument(
        '--size',
        type=int,
        default=200,
        help='Marker size in pixels (default: 200)'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default='markers',
        help='Output directory for markers (default: markers/)'
    )
    parser.add_argument(
        '--border-bits',
        type=int,
        default=1,
        help='Number of white border bits (default: 1)'
    )

    args = parser.parse_args()

    print("="*60)
    print("ArUco Marker Generator")
    print("="*60)
    print(f"Generating markers: {args.ids}")
    print(f"Size: {args.size}x{args.size} pixels")
    print(f"Output directory: {args.output_dir}")
    print()

    generate_marker_set(
        marker_ids=args.ids,
        size_pixels=args.size,
        output_dir=args.output_dir,
        border_bits=args.border_bits
    )

    print()
    print("Marker Usage:")
    print("  IDs 0-3: Field corner markers (Top-Left, Top-Right, Bottom-Right, Bottom-Left)")
    print("  ID 10: SB-3660")
    print("  ID 11: SB-74FB")
    print("  ID 12: SB-3716")
    print("  ID 13: SB-58EF")
    print()
    print("Print these markers and place them accordingly for best results.")


if __name__ == '__main__':
    main()
