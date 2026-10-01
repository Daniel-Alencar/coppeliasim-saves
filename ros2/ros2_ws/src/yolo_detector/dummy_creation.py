import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped

class CoppeliaMarkerSync(Node):
    def __init__(self, use_tf_frame=True):
        super().__init__('coppelia_marker_sync')
        self.use_tf_frame = use_tf_frame
        # Subscribe to YOLO markers
        self.sub = self.create_subscription(
            Marker,
            '/yolo/object_3d_point',  # topic where markers are published
            self.marker_callback,
            10
        )

        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Track dummies: class_name -> object handle
        self.dummies = {}
        self.current_frame_classes = set()

        # Timer to remove dummies not seen in the current frame
        self.timer = self.create_timer(1.0, self.cleanup_dummies)

    def marker_callback(self, msg: Marker):
        # Extract class name from marker text (or use marker.ns if you prefer)
        class_name = msg.text if msg.text else msg.ns
        self.current_frame_classes.add(class_name)

        #class_name = msg.text if msg.text else msg.ns
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z


        # Transform coordinates if using TF
        if self.use_tf_frame:
            try:
                # Transform marker to map frame (or odom)
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    'map',          # target frame
                    msg.header.frame_id,  # source frame
                    rclpy.time.Time()
                )
                point = PointStamped()
                point.header.frame_id = msg.header.frame_id
                point.point.x = x
                point.point.y = y
                point.point.z = z

                transformed = tf2_geometry_msgs.do_transform_point(point, transform)
                x = transformed.point.x
                y = transformed.point.y
                z = transformed.point.z
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed for {class_name}: {e}")
                
        if class_name in self.dummies:
            # Move existing dummy
            dummy_handle = self.dummies[class_name]
            self.sim.setObjectPosition(dummy_handle, -1, [x, y, z])
        else:
            # Create new dummy
            dummy_handle = self.sim.createDummy(0.1, [1,0,0])  #
            self.sim.setObjectAlias(dummy_handle, class_name) 
            self.dummies[class_name] = dummy_handle
            self.sim.setObjectPosition(dummy_handle, -1, [x, y, z])

    def cleanup_dummies(self):
        # Remove dummies not in the current frame
        missing = set(self.dummies.keys()) - self.current_frame_classes
        for class_name in missing:
            dummy_handle = self.dummies[class_name]
            self.sim.removeObject(dummy_handle)
            del self.dummies[class_name]

        self.current_frame_classes.clear()


def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaMarkerSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
# import rclpy
# from rclpy.node import Node
# from coppeliasim_zmqremoteapi_client import RemoteAPIClient
# import tf2_ros
# from geometry_msgs.msg import TransformStamped

# class CoppeliaTFSync(Node):
#     def __init__(self, target_frame="map"):
#         super().__init__('coppelia_tf_sync')

#         self.target_frame = target_frame

#         # Connect to CoppeliaSim
#         self.client = RemoteAPIClient()
#         self.sim = self.client.require('sim')

#         # TF buffer/listener
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # Track dummies: class_name -> object handle
#         self.dummies = {}

#         # Timer: poll TF tree every 0.5s
#         self.timer = self.create_timer(0.5, self.sync_dummies)

#     def sync_dummies(self):
#         try:
#             frames_str = self.tf_buffer.all_frames_as_string()
#         except Exception as e:
#             self.get_logger().warn(f"Could not fetch TF frames: {e}")
#             return

#         # Collect current detections
#         detected_classes = set()

#         # Parse TF frames (as string)
#         for line in frames_str.splitlines():
#             frame_id = line.strip()
#             if "object_" not in frame_id:  # Only YOLO object TFs
#                 continue

#             class_name = frame_id  # keep full name (e.g., object_person_0)
#             detected_classes.add(class_name)

#             try:
#                 transform: TransformStamped = self.tf_buffer.lookup_transform(
#                     frame_id,
#                     self.target_frame,
#                     rclpy.time.Time(),   # latest
#                     timeout=rclpy.duration.Duration(seconds=2.0)
#                 )

#                 x = transform.transform.translation.x
#                 y = transform.transform.translation.y
#                 z = transform.transform.translation.z

#                 if class_name in self.dummies:
#                     # Move existing dummy
#                     dummy_handle = self.dummies[class_name]
#                     self.sim.setObjectPosition(dummy_handle, -1, [x, y, z])
#                 else:
#                     # Create new dummy
#                     dummy_handle = self.sim.createDummy(0.1, [1,0,0])
#                     self.sim.setObjectAlias(dummy_handle, class_name)
#                     self.sim.setObjectPosition(dummy_handle, -1, [x, y, z])
#                     self.dummies[class_name] = dummy_handle

#             except Exception as e:
#                 self.get_logger().warn(f"TF lookup failed for {class_name}: {e}")

#         # Remove dummies not seen in this frame
#         missing = set(self.dummies.keys()) - detected_classes
#         for class_name in missing:
#             dummy_handle = self.dummies[class_name]
#             self.sim.removeObject(dummy_handle)
#             del self.dummies[class_name]


# def main(args=None):
#     rclpy.init(args=args)
#     node = CoppeliaTFSync(target_frame="camera_color_optical_frame")
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
