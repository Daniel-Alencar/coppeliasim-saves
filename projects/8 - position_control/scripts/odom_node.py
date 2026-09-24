import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from scipy.spatial.transform import Rotation as R

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, 'myRobot/odom', 10)

        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient() # put your IP/hostname here, like 'host=meu_PC'
            self.sim = self.client.getObject('sim')
            self.get_logger().info('Connected to CoppeliaSim successfully.')

            # Get robot and motor handles
            self.robotHandle = self.sim.getObject('/myRobot')
            self.leftMotor = self.sim.getObject('/leftMotor')
            self.rightMotor = self.sim.getObject('/rightMotor')

        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return

        # Timer to publish odometry at 10Hz
        self.timer = self.create_timer(0.1, self.publish_odom)

        # Robot physical parameters
        self.wheel_radius = 0.0975  # meters
        self.wheel_base = 0.331     # meters

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.sim.getSimulationTime()

        self.get_logger().info('ROS2 → CoppeliaSim Odom node started.')

    def publish_odom(self):
        try:
            # --- 1. Get wheel joint velocities (rad/s) ---
            left_vel = self.sim.getJointVelocity(self.leftMotor)
            right_vel = self.sim.getJointVelocity(self.rightMotor)

            # --- 2. Compute linear and angular velocities ---
            v = self.wheel_radius * (right_vel + left_vel) / 2.0
            w = self.wheel_radius * (right_vel - left_vel) / self.wheel_base

            # --- 3. Integrate to update pose ---
            current_time = self.sim.getSimulationTime()
            dt = (current_time - self.last_time)
            self.last_time = current_time

            self.x += v * np.cos(self.theta) * dt
            self.y += v * np.sin(self.theta) * dt
            self.theta += w * dt

            # --- 4. Create quaternion from yaw ---
            q = R.from_euler('xyz', [0, 0, self.theta]).as_quat()

            # --- 5. Fill Odometry message ---
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            # Pose
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation = Quaternion(
                x=q[0], y=q[1], z=q[2], w=q[3]
            )

            # Twist
            odom_msg.twist.twist.linear.x = v
            odom_msg.twist.twist.angular.z = w

            # Publish
            self.odom_publisher.publish(odom_msg)

            self.get_logger().debug(
                f'Odom → x={self.x:.3f}, y={self.y:.3f}, θ={self.theta:.3f}, v={v:.3f}, w={w:.3f}'
            )

        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
