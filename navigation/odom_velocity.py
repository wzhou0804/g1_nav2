"""Project body-frame Odometry twist into the gravity-aligned heading frame."""
import math


def body_twist_to_yaw(quaternion, linear, angular):
    """Quaternion is xyzw; ROS Odometry twist belongs to child_frame_id."""
    if (len(quaternion) != 4 or len(linear) != 3 or len(angular) != 3 or
            not all(math.isfinite(v) for v in (*quaternion, *linear, *angular))):
        raise ValueError('Invalid odometry orientation/twist')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if norm < 0.5:
        raise ValueError('Invalid odometry quaternion')
    x, y, z, w = (v/norm for v in quaternion)
    rotation = (
        (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
        (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
        (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)),
    )
    world_linear = [sum(a*b for a, b in zip(row, linear)) for row in rotation]
    world_wz = sum(a*b for a, b in zip(rotation[2], angular))
    yaw = math.atan2(rotation[1][0], rotation[0][0])
    c, s = math.cos(yaw), math.sin(yaw)
    return (c*world_linear[0]+s*world_linear[1],
            -s*world_linear[0]+c*world_linear[1], world_wz)
