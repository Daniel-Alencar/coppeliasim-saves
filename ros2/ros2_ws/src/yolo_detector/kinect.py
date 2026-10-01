import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import RegionOfInterest
# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import *

class KinectNode(Node):
    def __init__(self):
        super().__init__('kinect_node')
        
        # Create publishers for kinect data
        self.rgb_publisher = self.create_publisher(Image, '/rgb/image', 10)
        self.depth_publisher = self.create_publisher(Image, '/depth/image', 10)
        self.camerainfo_publisher = self.create_publisher(CameraInfo, '/rgb/camera_info', 10)
        
        # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.robotHandle = self.sim.getObject('/myRobot')
            self.depthCam=self.sim.getObject('/myRobot/kinect/depth')
            self.colorCam=self.sim.getObject('/myRobot/kinect/rgb')

            self.sim.startSimulation()

            self.get_logger().info('Connected to CoppeliaSim successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return
        
        # Timer to publish kinect data every 10Hz
        self.timer = self.create_timer(0.1, self.publish_camera)
        
        self.get_logger().info('ROS2 → CoppeliaSim Kinect node started.')
    
    def publish_camera(self):
        #try:
        # Get vision sensor RGB image from CoppeliaSim
        data, resolution = self.sim.getVisionSensorImg(self.colorCam)
        data = self.sim.transformImage(data,resolution,4)
        
        if data is not None:
            # Create image message
            rgb_msg = Image()

            # Fill image data
            rgb_msg.header.stamp = self.get_clock().now().to_msg()
            rgb_msg.header.frame_id = "kinect"
            rgb_msg.height = resolution[1]
            rgb_msg.width = resolution[0]
            rgb_msg.encoding = 'rgb8'
            rgb_msg.is_bigendian = 1
            rgb_msg.step = resolution[0]*3
            rgb_msg.data = data
            
            # Publish the message
            self.rgb_publisher.publish(rgb_msg)
        else:
            self.get_logger().warn('Kinect RGB image not found')
    
        # Get vision sensor RGB depth from CoppeliaSim
        data = self.sim.getVisionSensorDepthBuffer(self.depthCam+self.sim.handleflag_codedstring)
        
        resolution, nearClippingPlane = self.sim.getObjectFloatParameter(self.depthCam,self.sim.visionfloatparam_near_clipping)
        resolution, farClippingPlane = self.sim.getObjectFloatParameter(self.depthCam,self.sim.visionfloatparam_far_clipping)
        nearClippingPlane = nearClippingPlane # we want mm
        farClippingPlane = farClippingPlane # we want mm
        data = self.sim.unpackFloatTable(data)
        data = np.array(data)
        data = data.reshape((480, 640))    # reshape to 480 rows × 640 columns
        data = data[::-1,: ] 
        
        data = data.flatten()             # back to 1D
        data = (data * 255).astype(np.uint8).tolist()         
        
        resolution = self.sim.getVisionSensorResolution(self.depthCam)
        
        # Debug prints
        # print(f"data      : {data}")
        # print(f"Near      : {nearClippingPlane}")
        # print(f"Far       : {farClippingPlane}")
        
        if data is not None:
            # Create image message
            depth_msg = Image()

            # Fill image data
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = "kinect"
            depth_msg.height = resolution[1]
            depth_msg.width = resolution[0]
            depth_msg.encoding = '8UC1'
            depth_msg.is_bigendian = 0
            depth_msg.step = resolution[0]*2
            depth_msg.data = data
            
            # Publish the message
            self.depth_publisher.publish(depth_msg)
        else:
            self.get_logger().warn('Kinect depth image not found')

        if resolution is not None:
            # Publish camera info
            info_msg = CameraInfo()
            info_msg.roi = RegionOfInterest()

            # Fill message
            info_msg.header.stamp = self.get_clock().now().to_msg()
            info_msg.header.frame_id = "kinect"
            info_msg.height = resolution[1]
            info_msg.width = resolution[0]
            info_msg.distortion_model = 'plumb_bob'
            info_msg.d = [0.262383,-0.953104,-0.005358,0.002628,1.163314]
            info_msg.k = [517.306408, 0.0, 318.643040, 0.0, 516.469215, 255.313989, 0.0, 0.0, 1.0]
            info_msg.p = [517.306408, 0.0, 318.643040, 0.0, 0.0, 516.469215, 255.313989, 0.0, 0.0, 0.0, 1.0, 0.0]
            info_msg.binning_x = 0
            info_msg.binning_y = 0
            
            info_msg.roi.x_offset = 0
            info_msg.roi.y_offset = 0
            info_msg.roi.height = 0
            info_msg.roi.width = 0
            info_msg.roi.do_rectify = False

            # Publish the message
            self.camerainfo_publisher.publish(info_msg)
        else:
            self.get_logger().warn('Kinect camera info not available')

def main(args=None):
    rclpy.init(args=args)
    node = KinectNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
