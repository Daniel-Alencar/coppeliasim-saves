import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
import tf2_ros
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class CoppeliaTFPublisher(Node):
    def __init__(self):
        super().__init__('coppelia_tf_publisher')

        # TF broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        # Get object handles
        self.ground_truth = self.sim.getObject("./Ground_truth")
        self.sensor_ref = self.sim.getObject("./camera_ref")

        # Timer to publish transforms periodically
        self.timer = self.create_timer(0.05, self.publish_transforms)

    def get_transform(self, child_handle, child_frame, parent_handle, parent_frame):
        pos = self.sim.getObjectPosition(child_handle, parent_handle)
        quat = self.sim.getObjectQuaternion(child_handle, parent_handle)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        return t

    def publish_transforms(self):
        # Build and send multiple transforms
        transforms = [
            self.get_transform(self.ground_truth, 'base_footprint', -1, 'map'),
            self.get_transform(self.ground_truth, 'base_link', -1, 'map'),
            self.get_transform(self.sensor_ref, 'camera_color_optical_frame', self.ground_truth, 'base_link')
        ]

        for t in transforms:
            self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
