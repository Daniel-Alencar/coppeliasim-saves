import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from coppeliasim_zmqremoteapi_client import *


class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('goal_publisher_node')

        # Publisher for /goal (Pose2D)
        self.goal_publisher = self.create_publisher(Pose2D, 'myRobot/goal', 10)

        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.target_handle = self.sim.getObject('/myRobotTarget')

            self.get_logger().info('Connected to CoppeliaSim successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return

        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_goal)
        self.get_logger().info('ROS2 → CoppeliaSim Goal publisher node started.')

        # Last published position
        self.last_x = None
        self.last_y = None

        # Minimal distance threshold (meters)
        self.min_distance = 0.5

    def publish_goal(self):
        try:
            # --- Read current position and orientation ---
            pos = self.sim.getObjectPosition(self.target_handle, -1)
            euler = self.sim.getObjectOrientation(self.target_handle, -1)
            x, y, theta = pos[0], pos[1], euler[2]

            # --- Check minimal distance ---
            if self.last_x is not None and self.last_y is not None:
                distance = np.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
                if distance < self.min_distance:
                    return  # Too close, skip publishing

            # --- Publish Pose2D ---
            goal_msg = Pose2D()
            goal_msg.x = x
            goal_msg.y = y
            goal_msg.theta = theta

            self.goal_publisher.publish(goal_msg)

            # --- Update last published position ---
            self.last_x = x
            self.last_y = y

            self.get_logger().debug(
                f'Published Goal → x={x:.3f}, y={y:.3f}, θ={theta:.3f}'
            )

        except Exception as e:
            self.get_logger().error(f'Error reading or publishing goal: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
