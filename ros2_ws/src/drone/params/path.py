import matplotlib.pyplot as plt

# Plant center and size
pcx, pcy = 1.25, 0.0
half = 0.2  # 40 cm plant edges

square_x = [pcx-half, pcx+half, pcx+half, pcx-half, pcx-half]
square_y = [pcy-half, pcy-half, pcy+half, pcy+half, pcy-half]

# Waypoints from YAML
waypoints = {
    19: (0.00, 0.00),
    20: (0.25, 0.00),
    21: (0.25, -0.40),
    22: (0.25, -0.40),
    23: (0.95, -0.80),
    24: (0.95, -0.80),
    25: (1.55, -0.80),
    27: (2.05, -0.40),
    28: (2.05, -0.40),
    29: (2.05, 0.40),
    30: (2.05, 0.40),
    31: (1.55, 0.80),
    32: (1.55, 0.80),
    33: (0.95, 0.80),
    35: (0.55, 0.40),
    37: (0.25, 0.00),
}

# Order of path (including turns in place)
order = [20, 21, 22, 23, 24, 25, 27, 28, 29, 30, 31, 32, 33, 35, 37]
xs = [waypoints[i][0] for i in order]
ys = [waypoints[i][1] for i in order]

plt.figure(figsize=(7,7))

# Plant
plt.plot(square_x, square_y, linewidth=2, label="Plant (40 cm square)")

# Path
plt.plot(xs, ys, marker='o', label="Drone waypoints")

# Annotate waypoints
for i in order:
    x, y = waypoints[i]
    plt.text(x, y, str(i), fontsize=9)

# Start segment 19->20
x19, y19 = waypoints[19]
x20, y20 = waypoints[20]
plt.plot([x19, x20], [y19, y20], linestyle='--', label="Start (19→20)")
plt.scatter(x19, y19, marker='x')
plt.text(x19, y19, "19", fontsize=9)

plt.gca().set_aspect('equal', adjustable='box')
plt.grid(True)
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Drone Path with Extra Turning Waypoints")
plt.legend()
plt.show()
