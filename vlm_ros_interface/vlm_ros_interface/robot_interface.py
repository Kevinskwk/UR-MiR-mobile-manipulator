import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.action import NavigateToPose
from linkattacher_msgs.srv import AttachLink, DetachLink

import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import numpy as np
import math
import threading

from .utils import yaw_to_quaternion, get_3d_position_from_depth

class RobotInterface(Node):
    def __init__(self, node_name="robot_interface"):
        super().__init__(node_name)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.moveit_publisher = self.create_publisher(PoseStamped, 'moveit_poses', 10)
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')

        self.attach_request = AttachLink.Request()
        self.detach_request = DetachLink.Request()
        
        self.bridge = CvBridge()
        self.mani_depth_sub = self.create_subscription(
            Image, '/mani_realsense/camera/aligned_depth_to_color/image_raw', self.mani_depth_callback, 10)
        self.mani_camera_info_sub = self.create_subscription(
            CameraInfo, '/mani_realsense/camera/aligned_depth_to_color/camera_info', self.mani_camera_info_callback, 10)
        
        self.navi_depth_sub = self.create_subscription(
            Image, '/navi_realsense/camera/aligned_depth_to_color/image_raw', self.navi_depth_callback, 10)
        self.navi_camera_info_sub = self.create_subscription(
            CameraInfo, '/navi_realsense/camera/aligned_depth_to_color/camera_info', self.navi_camera_info_callback, 10)
        
        self.mani_intrinsics = None
        self.mani_depth_image = None
        self.navi_intrinsics = None
        self.navi_depth_image = None
        self.timestamp = None

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def mani_camera_info_callback(self, msg):
        self.mani_intrinsics = np.array(msg.k)
        self.get_logger().debug(f'Camera Info received: {self.mani_intrinsics}')

    def mani_depth_callback(self, msg):
        self.mani_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.timestamp = msg.header.stamp
        # self.get_logger().info(f"Image timestamp: {msg.header.stamp}")

    def navi_camera_info_callback(self, msg):
        self.navi_intrinsics = np.array(msg.k)
        self.get_logger().debug(f'Camera Info received: {self.navi_intrinsics}')

    def navi_depth_callback(self, msg):
        self.navi_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # self.timestamp = msg.header.stamp
        # self.get_logger().info(f"Image timestamp: {msg.header.stamp}")

    def transform_to_ref_frame(self, point_camera_frame, camera_frame, ref_frame):
        try:
            transform = self.tf_buffer.lookup_transform(ref_frame, camera_frame, self.timestamp)
            # Transform the point from camera frame to base_link frame
            point_stamped = tf2_geometry_msgs.PointStamped()
            point_stamped.header.stamp = self.timestamp
            point_stamped.header.frame_id = camera_frame
            point_stamped.point.x = point_camera_frame[0]
            point_stamped.point.y = point_camera_frame[1]
            point_stamped.point.z = point_camera_frame[2]

            transformed_point = self.tf_buffer.transform(point_stamped, ref_frame)
            return [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z]
        except (tf2_ros.TransformException, IndexError) as e:
            self.get_logger().error(f"Error transforming point: {e}")
            return None

    def process_rgb_coordinate_for_mani(self, u, v):
        # u, v are normalized coordinates from 0 to 1
        if self.mani_intrinsics is not None and self.mani_depth_image is not None:
            point_camera_frame = get_3d_position_from_depth(u, v, self.mani_depth_image, self.mani_intrinsics)
            if point_camera_frame:
                self.get_logger().info(f'3D position in camera frame: {point_camera_frame}')
                return self.transform_to_ref_frame(point_camera_frame, 'mani_realsense_color_frame', 'base_link')
            else:
                self.get_logger().warn('Could not calculate 3D position.')
        else:
            self.get_logger().warn('RGB or Depth image not received or CameraInfo unavailable.')

    def process_rgb_coordinate_for_navi(self, u, v):
        # u, v are normalized coordinates from 0 to 1
        if self.navi_intrinsics is not None and self.navi_depth_image is not None:
            point_camera_frame = get_3d_position_from_depth(u, v, self.navi_depth_image, self.navi_intrinsics)
            if point_camera_frame:
                self.get_logger().info(f'3D position in camera frame: {point_camera_frame}')
                return self.transform_to_ref_frame(point_camera_frame, 'navi_realsense_color_frame', 'map')
            else:
                self.get_logger().warn('Could not calculate 3D position.')
        else:
            self.get_logger().warn('RGB or Depth image not received or CameraInfo unavailable.')

    def get_robot_position_in_map(self):
        try:
            # Get the transform from base_link to map
            transform = self.tf_buffer.lookup_transform('map', 'base_link', self.timestamp)
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            return robot_x, robot_y
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return None

    def calculate_nav_pose(self, target_point):
        # Get the robot's position in the map frame
        [target_x, target_y, target_z] = target_point
        robot_position = self.get_robot_position_in_map()
        if robot_position is None:
            return None
        
        robot_x, robot_y = robot_position
        
        # Calculate the differences in x and y
        dx = target_x - robot_x
        dy = target_y - robot_y
        
        # Calculate the yaw angle using arctan2
        yaw = math.atan2(dy, dx)
        x = target_x - 0.5 * math.cos(yaw)
        y = target_y - 0.5 * math.sin(yaw)

        return [x, y, yaw]

    def send_navigation_goal(self, x, y, yaw):
        """Send a navigation goal to Nav2."""
        # Wait until the action server is available
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return

        # Convert yaw to quaternion
        q = yaw_to_quaternion(yaw)

        # Create a goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = q
        
        # Send the goal
        self._nav_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Sent goal to Nav2: x={x}, y={y}, yaw={yaw}")

    def send_moveit_goal(self, target_pose: Pose):
        # Pose is from base_link to ur_tool0
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "ur_base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose

        self.moveit_publisher.publish(pose_stamped)

    def send_attach_call(self, obj_name):
        if not self.attach_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service not available")
            return
        
        self.attach_request.model1_name = 'mir_robot'
        self.attach_request.link1_name = 'ur_wrist_3_link'
        self.attach_request.link2_name = 'link_0'
        self.attach_request.model2_name = obj_name
        self.get_logger().info(f"Attaching object: {obj_name}")
        self.attach_client.call_async(self.attach_request)
        # self.future.add_done_callback(self.attach_response_callback)

    def send_detach_call(self, obj_name):
        if not self.detach_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service not available")
            return
        
        self.detach_request.model1_name = 'mir_robot'
        self.detach_request.link1_name = 'ur_wrist_3_link'
        self.detach_request.link2_name = 'link_0'
        self.detach_request.model2_name = obj_name
        self.get_logger().info(f"Attaching object: {obj_name}")
        self.detach_client.call_async(self.detach_request)
        # self.future.add_done_callback(self.detach_response_callback)

    def pick_object(self, u, v):
        point_base_link = self.process_rgb_coordinate_for_mani(u, v)

        if point_base_link is not None:
            [x, y, z] = point_base_link
            # Create a target pose
            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z
            # Downward picking orientation
            target_pose.orientation.x = -0.707
            target_pose.orientation.y = 0.707
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 0.0

            self.get_logger().info(f"Sending MoveIt goal to x: {x}, y: {y}, z: {z}")
            self.send_moveit_goal(target_pose)

    def place_object(self, u, v):
        point_base_link = self.process_rgb_coordinate_for_mani(u, v)

        if point_base_link is not None:
            [x, y, z] = point_base_link
            # Create a target pose
            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z + 0.05  # additional clearance for placing
            # Downward placing orientation
            target_pose.orientation.x = -0.707
            target_pose.orientation.y = 0.707
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 0.0

            self.get_logger().info(f"Sending MoveIt goal to x: {x}, y: {y}, z: {z + 0.05}")
            self.send_moveit_goal(target_pose)

    def navigate_robopoint(self, u, v):
        point_map = self.process_rgb_coordinate_for_navi(u, v)

        if point_map is not None:
            [x, y, yaw] = self.calculate_nav_pose(point_map)
            self.get_logger().info(f"Sending navigation goal to x: {x}, y: {y}, yaw: {yaw}")
            self.send_navigation_goal(x, y, yaw)

    def attach_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service call successful, attached link: {response.success}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def detach_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service call successful, detached link: {response.success}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def parse_and_execute_command(robot: RobotInterface, command: str):
    """
    Parse the command string and execute the appropriate method on the robot interface.
    """
    parts = command.split()
    
    if len(parts) < 2:
        print("Invalid command format. Use 'nav u v' or 'pick u v' or 'place u v' or 'att obj_name' or 'det obj_name'.")
        return

    cmd_type = parts[0]

    try:
        if cmd_type == "navt" and len(parts) == 4:
            # Parse and send test navigation goal
            x, y, yaw = float(parts[1]), float(parts[2]), float(parts[3])
            print(f"Sending navigation goal to x: {x}, y: {y}, yaw: {yaw}")
            robot.send_navigation_goal(x, y, yaw)

        elif cmd_type == "movt" and len(parts) == 4:
            # Parse and send test MoveIt goal
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            print(f"Sending MoveIt goal to x: {x}, y: {y}, z: {z}")

            # Create a target pose
            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z
            # Downward picking orientation
            target_pose.orientation.x = -0.707
            target_pose.orientation.y = 0.707
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 0.0

            robot.send_moveit_goal(target_pose)

        elif cmd_type == "att" and len(parts) == 2:
            obj_name = str(parts[1])
            robot.send_attach_call(obj_name)

        elif cmd_type == "det" and len(parts) == 2:
            obj_name = str(parts[1])
            robot.send_detach_call(obj_name)

        elif cmd_type == "pick" and len(parts) == 3:
            u, v = float(parts[1]), float(parts[2])
            robot.pick_object(u, v)

        elif cmd_type == "place" and len(parts) == 3:
            u, v = float(parts[1]), float(parts[2])
            robot.place_object(u, v)

        elif cmd_type == "nav" and len(parts) == 3:
            u, v = float(parts[1]), float(parts[2])
            robot.navigate_robopoint(u, v)

        else:
            print("Invalid command format. Use 'nav u v' or 'pick u v' or 'place u v' or 'att obj_name' or 'det obj_name'.")
    
    except ValueError:
        print("Error parsing command. Ensure numbers are valid.")


def main(args=None):
    rclpy.init(args=args)
    robot = RobotInterface()

    ros_spin_thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    ros_spin_thread.start()

    print("Enter commands in the format 'nav u v' or 'pick u v' or 'place u v' or 'att obj_name' or 'det obj_name'. Type 'exit' to quit.")
    
    try:
        while True:
            command = input(">>> ").strip()
            if command.lower() == "exit":
                print("Exiting...")
                break

            parse_and_execute_command(robot, command)
    
    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        # Shutdown the ROS node and library
        robot.destroy_node()
        rclpy.shutdown()
        ros_spin_thread.join()

if __name__ == "__main__":
    main()
