#!/usr/bin/env python3

import math
import yaml
import matplotlib.pyplot as plt
from pathlib import Path


# ---------------------------------------------------------
# SET YOUR YAML FILE HERE
# ---------------------------------------------------------
YAML_FILE = "/home/hakonb/tello/ros2_ws/src/drone/params/test_circle_waypoints.yaml"
# ---------------------------------------------------------

# Plant box parameters
PLANT_CENTER = (1.25, 0.0)   # (x, y)
PLANT_SIDE = 0.40            # meters
# ---------------------------------------------------------


def yaw_from_quaternion(x, y, z, w):
    """Extract yaw (rotation about Z) from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_waypoints(yaml_path):
    """Load waypoints from ROS2 parameter YAML."""
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    root = next(iter(data.values()))
    params = root["ros__parameters"]

    waypoint_ids = params.get("waypoint", [])
    waypoints = []

    for wid in waypoint_ids:
        key = f"waypoint_{wid}"
        if key not in params:
            continue

        pose = params[key]["pose"]
        pos = pose["position"]
        ori = pose["orientation"]

        x, y, z = float(pos["x"]), float(pos["y"]), float(pos["z"])
        qx, qy, qz, qw = ori["x"], ori["y"], ori["z"], ori["w"]

        yaw = yaw_from_quaternion(qx, qy, qz, qw)

        waypoints.append({"id": wid, "x": x, "y": y, "z": z, "yaw": yaw})

    return waypoints


def draw_plant_box(ax, center, side):
    """Draw the 40 cm × 40 cm plant box."""
    cx, cy = center
    half = side / 2.0

    # Box corners
    xs = [cx - half, cx + half, cx + half, cx - half, cx - half]
    ys = [cy - half, cy - half, cy + half, cy + half, cy - half]

    ax.plot(xs, ys, "g-", linewidth=2, label="Plant box")


def plot_waypoints(waypoints):
    """Plot x–y path with yaw arrows and plant box."""
    xs = [wp["x"] for wp in waypoints]
    ys = [wp["y"] for wp in waypoints]
    yaws = [wp["yaw"] for wp in waypoints]
    ids = [wp["id"] for wp in waypoints]

    fig, ax = plt.subplots()

    # Plot the path
    ax.plot(xs, ys, "-o", label="Waypoints")

    # Draw heading arrows
    L = 0.15
    ax.quiver(
        xs, ys,
        [math.cos(y) * L for y in yaws],
        [math.sin(y) * L for y in yaws],
        scale_units="xy", scale=1, color="black"
    )

    # Annotate waypoint IDs
    for x, y, wid in zip(xs, ys, ids):
        ax.text(x, y, str(wid), ha="right", va="bottom")

    # Draw plant box
    draw_plant_box(ax, PLANT_CENTER, PLANT_SIDE)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", "box")
    ax.grid(True)
    ax.legend()
    ax.set_title("Waypoints + Plant Box")

    plt.show()


def main():
    path = Path(YAML_FILE)
    if not path.exists():
        raise FileNotFoundError(f"YAML not found: {path}")

    waypoints = load_waypoints(path)
    plot_waypoints(waypoints)


if __name__ == "__main__":
    main()
