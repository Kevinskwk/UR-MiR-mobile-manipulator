from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

def yaw_to_quaternion(yaw):
    """Convert a yaw angle (in radians) to a quaternion."""
    q = quaternion_from_euler(0, 0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def get_3d_position_from_depth(u, v, depth_image, intrinsics):
    # Convert normalized u, v to pixel coordinates
    width = depth_image.shape[1]
    height = depth_image.shape[0]

    u_pixel = int(u * width)
    v_pixel = int(v * height)

    # Get the depth value at this pixel
    depth_value = depth_image[v_pixel, u_pixel] / 1000.0

    if depth_value == 0:
        print("Depth value is zero, invalid pixel")
        return None

    # Camera intrinsic parameters: fx, fy, cx, cy
    fx = intrinsics[0]
    fy = intrinsics[4]
    cx = intrinsics[2]
    cy = intrinsics[5]

    # Convert depth to 3D coordinates (camera frame)
    y = -(u_pixel - cx) * depth_value / fx
    z = -(v_pixel - cy) * depth_value / fy
    x = depth_value

    # Transform the point from camera frame to base_link frame
    point_camera_frame = [x, y, z]
    return point_camera_frame
