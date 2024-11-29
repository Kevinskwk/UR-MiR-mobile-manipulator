import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler

import threading


class RobotInterface(Node):
    def __init__(self, node_name="navigation_client"):
        super().__init__(node_name)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.moveit_publisher = self.create_publisher(PoseStamped, 'moveit_poses', 10)

    def send_navigation_goal(self, x, y, yaw):
        """Send a navigation goal to Nav2."""
        # Wait until the action server is available
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return

        # Convert yaw to quaternion
        q = self.yaw_to_quaternion(yaw)

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

    @staticmethod
    def yaw_to_quaternion(yaw):
        """Convert a yaw angle (in radians) to a quaternion."""
        q = quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def parse_and_execute_command(robot: RobotInterface, command: str):
    """
    Parse the command string and execute the appropriate method on the robot interface.
    """
    parts = command.split()
    
    if len(parts) < 3:
        print("Invalid command format. Use 'nav x y' or 'mov x y z'.")
        return

    cmd_type = parts[0]

    try:
        if cmd_type == "nav" and len(parts) == 4:
            # Parse and send navigation goal
            x, y, yaw = float(parts[1]), float(parts[2]), float(parts[3])
            print(f"Sending navigation goal to x: {x}, y: {y}, yaw: {yaw}")
            robot.send_navigation_goal(x, y, yaw)

        elif cmd_type == "mov" and len(parts) == 4:
            # Parse and send MoveIt goal
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

        else:
            print("Invalid command format. Use 'nav x y' or 'mov x y z'.")
    
    except ValueError:
        print("Error parsing command. Ensure x, y, z are valid numbers.")


def main(args=None):
    rclpy.init(args=args)
    robot = RobotInterface()

    ros_spin_thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    ros_spin_thread.start()

    # Example usage: send a goal to (1.0, 1.0) with a yaw of 0
    # robot.send_navigation_goal(1.0, 1.0, 0.0)

    # target_pose = Pose()
    # target_pose.position.x = 0.8 # -0.2
    # target_pose.position.y = 0.0 # 0.6
    # target_pose.position.z = 1.0 # 0.2
    # # Downward picking orientation
    # target_pose.orientation.x = -0.707
    # target_pose.orientation.y = 0.707
    # target_pose.orientation.z = 0.0
    # target_pose.orientation.w = 0.0

    # robot.send_moveit_goal(target_pose)

    # rclpy.spin(robot)
    # robot.destroy_node()
    # rclpy.shutdown()

    print("Enter commands in the format 'nav x y' or 'mov x y z'. Type 'exit' to quit.")
    
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
