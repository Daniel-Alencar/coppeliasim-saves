import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
from ultralytics import YOLO
import tf2_ros
import math

##Instead of using cv_bridge, manually converting to numpy
def ros_depth_to_numpy(msg: Image):
    # Determine data type
    if msg.encoding == "32FC1":
        dtype = np.float32
    elif msg.encoding in ["16UC1", "16U"]:
        dtype = np.uint16
    elif msg.encoding in ["8UC1", "8U"]:
        dtype = np.uint8
    else:
        raise ValueError(f"Unsupported depth encoding: {msg.encoding}")
    
    # Convert buffer to NumPy array
    arr = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
    
    # If depth image needs to be flipped horizontally
    #arr = arr[:, ::-1]
    return arr

class Yolo3DPublisher(Node):
    def __init__(self):
        super().__init__('yolo_3d_publisher')

        self.model = YOLO("yolo11n-seg.pt") #modelo treinado com 640x480

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
        #self.banana = self.sim.getObject("./banana")
        self.camera = self.sim.getObject("./camera_ref")

    def depth_callback(self, msg):
        self.depth_image = ros_depth_to_numpy(msg)

    def rgb_callback(self, msg):
        if self.depth_image is None:
            return

        channels = msg.step // msg.width
        img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)


        if msg.encoding.lower() in ["rgb8", "rgb"]:
            cv_image = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
        else:
            cv_image = img_np
        results = self.model(cv_image, verbose=False)

        camXResolution=cv_image.shape[1]  #320
        camYResolution=cv_image.shape[0]  #240
        annotated_frame = cv_image.copy()

        # Draw bounding box
        if results[0].boxes is not None and results[0].masks is not None:
            annotated_frame = results[0].plot()
            for det, cls, mask in zip(results[0].boxes.xyxy, results[0].boxes.cls, results[0].masks.data):
                mask = mask.cpu().numpy()
                
                # Binarize just in case values are between 0–1
                mask_bin = (mask > 0.5).astype(np.uint8)

                # Compute centroid (only if mask has non-zero area)
                ys, xs = np.nonzero(mask_bin)
                if len(xs) == 0 or len(ys) == 0:
                    continue  # skip empty masks
                cls_id = int(cls)
                centroid_x = float(np.mean(xs))
                centroid_y = float(np.mean(ys))

                # Scale down to image size due to yolo being trained on 640x480
                mask_h, mask_w = mask.shape
                img_h, img_w = cv_image.shape[:2]

                scale_x = img_w / mask_w  # e.g., 320 / 640 = 0.5
                scale_y = img_h / mask_h  # e.g., 240 / 480 = 0.5

                centroid_x *= scale_x
                centroid_y *= scale_y
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
                camXAngleInDegrees=57
                camXHalfAngle=camXAngleInDegrees*0.5*math.pi/180
                camYHalfAngle=(camXAngleInDegrees*0.5*math.pi/180)*camYResolution/camXResolution
                nearClippingPlane=0.2
                depthAmplitude=3.34
                # Depth
                depth = float(self.depth_image[cy, cx])/255.0

                #self.get_logger().info(f"Raw depth at pixel ({cx}, {cy}): {depth}")
                if depth == 0:
                    continue
                depth = depth * depthAmplitude + nearClippingPlane
                x_angle = ((camXResolution/2) - cx - 0.5) * camXHalfAngle / (camXResolution/2)
                y_angle = ((camYResolution/2) - cy + 0.5) * camYHalfAngle / (camYResolution/2)
                X = math.tan(x_angle) * depth
                Y = math.tan(y_angle) * depth
                Z = depth

                
                # Mask centroid
                cv2.rectangle(annotated_frame, ((int(centroid_x)-2), (int(centroid_y))-2), ((int(centroid_x)+2), (int(centroid_y))+2), (255, 255, 0), 2)

                #BBox center
                #cv2.rectangle(annotated_frame, (int(cx)-2,int(cy)-2), (int(cx)+2, int(cy)+2), (255,255,0), 2)     # anti-aliased edge)
                

                #cv2.putText(annotated_frame, f"{int(cls)}", (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                pos = [X, Y, Z]  # fallback: use projected depth
                pos = [float(x) for x in pos]

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
            #annotated_frame = results[0].plot()

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
