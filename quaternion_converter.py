import math
yaw_radians = 0.0
yaw_deg = -45.0
if yaw_deg:
    yaw_radians = yaw_deg / 180.0 * math.pi
print(f"x: 0.0")
print(f"y: 0.0")
print(f"z: {math.sin(yaw_radians)}")
print(f"w: {math.cos(yaw_radians/2)}")