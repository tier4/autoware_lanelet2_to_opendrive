#!/usr/bin/env python3
"""
Demo script for Cartesian to Frenet coordinate conversion using Splines class.

This script demonstrates how to convert Cartesian coordinates (x, y) to
Frenet coordinates (s, d) using a B-spline as the reference line.
"""

import numpy as np
import matplotlib.pyplot as plt
from autoware_lanelet2_to_opendrive.spline import Splines


def main():
    """Run the Cartesian to Frenet conversion demo."""
    # 1. Create waypoints for an S-curve road
    wx = np.array([0.0, 20.0, 40.0, 60.0, 80.0])
    wy = np.array([0.0, 0.0, 20.0, 20.0, 40.0])
    wz = np.zeros_like(wx)
    points = np.column_stack([wx, wy, wz])

    # Create the spline
    spline = Splines(points, num_control_points=12)

    # 2. Define test query points (like vehicles at different positions)
    query_points = [
        (10.0, 2.0, 0.0),  # Straight section, left side
        (40.0, 15.0, 0.0),  # Curve section, right side (inner)
        (65.0, 25.0, 0.0),  # Next curve section, left side (outer)
    ]

    print("Cartesian to Frenet Conversion Demo")
    print("=" * 60)
    print(f"{'Query (x, y, z)':<20} | {'s (Arc Length)':<15} | {'d (Lateral)':<15}")
    print("-" * 60)

    # 3. Visualize the results
    plt.figure(figsize=(12, 8))

    # Plot the reference line (spline)
    num_plot_points = 100
    s_vals = np.linspace(0, spline.total_length, num_plot_points)
    line_pts = np.array([spline.evaluate(s) for s in s_vals])
    plt.plot(
        line_pts[:, 0],
        line_pts[:, 1],
        "k-",
        linewidth=2,
        label="Reference Line (Spline)",
    )

    # Plot waypoints
    plt.plot(wx, wy, "bx", markersize=10, label="Waypoints")

    # 4. Convert each query point and visualize
    for qx, qy, qz in query_points:
        # ★ Perform Cartesian to Frenet conversion
        s, d = spline.cartesian_to_frenet(qx, qy, qz)

        print(f"({qx:.1f}, {qy:.1f}, {qz:.1f})  | {s:.4f}          | {d:.4f}")

        # Get the closest point on the spline for visualization
        closest_pt = spline.evaluate(s)

        # Plot query point
        plt.plot(qx, qy, "ro", markersize=8)

        # Draw perpendicular line from query point to spline
        plt.plot(
            [qx, closest_pt[0]], [qy, closest_pt[1]], "r--", alpha=0.5, linewidth=1.5
        )

        # Add text annotation
        side = "L" if d > 0 else "R"
        plt.text(
            qx,
            qy + 1.5,
            f"s={s:.1f}m\nd={d:.1f}m ({side})",
            fontsize=9,
            color="blue",
            ha="center",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        )

    # Configure plot
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="upper left")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(
        "Cartesian to Frenet Coordinate Conversion\n"
        "Using Splines Class with B-Spline Reference Line"
    )

    print("=" * 60)
    print(f"Total spline length: {spline.total_length:.2f} meters")
    print("\nNote:")
    print("  - s: Arc length along the reference line (meters)")
    print("  - d: Lateral offset (positive=left, negative=right)")
    print("  - L/R: Indicates left or right side of the reference line")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
