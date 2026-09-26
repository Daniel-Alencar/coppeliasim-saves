import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
from ultralytics import YOLO
import tf2_ros

class Yolo3DPublisher(Node):
    def __init__(self):
        super().__init__('yolo_3d_publisher')

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

        # Camera intrinsics
        self.fx = 517.306408                                       
        self.fy = 516.469215
        self.cx = 318.643040
        self.cy = 235.313989
        self.dist = np.array([0.262383, -0.953104, -0.005358, 0.002628, 1.163314])

        # Subscribers
        self.rgb_sub = self.create_subscription(Image, '/rgb/image', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth/image', self.depth_callback, 10)
        self.depth_image = None

        # Publishers                                                                                                            
        self.image_pub = self.create_publisher(Image, '/yolo/annotated', 10)
        self.marker_pub = self.create_publisher(Marker, '/yolo/object_3d_point', 10)
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Connect to CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.bowl = self.sim.getObject("./Bowl")
        self.cup = self.sim.getObject("./Cup")
        self.camera = self.sim.getObject("./camera_ref")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_image = self.depth_image[:, ::-1]

    def rgb_callback(self, msg):
        if self.depth_image is None:
            return

        img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        if msg.encoding.lower() in ["rgb8", "rgb"]:
            cv_image = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
        else:
            cv_image = img_np
        results = self.model(cv_image, verbose=False)

        annotated_frame = cv_image.copy()

        for det, cls in zip(results[0].boxes.xyxy, results[0].boxes.cls):
            x1, y1, x2, y2 = map(int, det)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            
            # Undistort point
            pt = np.array([[[cx, cy]]], dtype=np.float32)
            undist = cv2.undistortPoints(pt, 
                                         cameraMatrix=np.array([[self.fx,0,self.cx],[0,self.fy,self.cy],[0,0,1]]), 
                                         distCoeffs=self.dist,
                                         P=None)
            u_undist, v_undist = undist[0,0]

            # Depth
            depth = float(self.depth_image[cy, cx])/1000.0
            if depth == 0:
                continue

            # Project to 3D
            X = (u_undist - self.cx) * depth / self.fx
            Y = (v_undist - self.cy) * depth / self.fy
            Z = depth

            # Draw bounding box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(annotated_frame, f"{int(cls)}", (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            cls_id = int(cls)
            if cls_id == 41:
                pos = self.sim.getObjectPosition(self.bowl, self.camera)
            elif cls_id == 45:
                pos = self.sim.getObjectPosition(self.cup, self.camera)
            else:
                pos = (X, Y, Z)  # fallback: use projected depth
            # # Publish 3D point
            marker = Marker()
            marker.header.frame_id = "camera_color_optical_frame"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.text = f"class_{int(cls)}"
            self.marker_pub.publish(marker)
            #Publish TF instead of Marker
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "camera_color_optical_frame"
            t.child_frame_id = f"object_{int(cls)}"

            t.transform.translation.x = pos[0]
            t.transform.translation.y = pos[1]
            t.transform.translation.z = pos[2]

            # No orientation info → identity quaternion
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)
        # Publish annotated image
        annotated_frame = results[0].plot()

        # Convert NumPy -> ROS2 Image
        out_msg = Image()
        out_msg.header = msg.header  # preserve timestamp/frame_id
        out_msg.height = annotated_frame.shape[0]
        out_msg.width = annotated_frame.shape[1]
        out_msg.encoding = "bgr8"
        out_msg.is_bigendian = False
        out_msg.step = annotated_frame.shape[1] * 3
        out_msg.data = annotated_frame.tobytes()

        # Publish annotated image
        self.image_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Yolo3DPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

