import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion

def marker_quaternion_with_world_yaw(yaw_rad: float) -> Quaternion:
    """
    Returns the quaternion of the marker in world frame after applying
    an additional yaw rotation (world Z rotation) by yaw_rad.
    """

    # 1. Base rotation from marker frame → world frame
    #    Columns are marker axes expressed in world frame:
    #    x_m = (0, -1, 0)
    #    y_m = (0,  0, 1)
    #    z_m = (-1, 0, 0)
    R_base = np.array([
        [0,  0, -1],
        [-1, 0,  0],
        [0,  1,  0]
    ])

    # Convert base rotation to quaternion
    q_base = R.from_matrix(R_base)

    # 2. Additional yaw rotation applied in world frame
    q_yaw = R.from_euler('z', yaw_rad)

    # 3. Final rotation: apply yaw AFTER base rotation
    #    (world <- yaw) * (world <- base)
    q_final = q_yaw * q_base

    # Convert to geometry_msgs/Quaternion
    q = q_final.as_quat()  # returns (x, y, z, w)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

from math import radians

q = marker_quaternion_with_world_yaw(radians(180))
print(q)