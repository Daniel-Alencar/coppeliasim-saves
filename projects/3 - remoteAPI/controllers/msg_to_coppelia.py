import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import String

# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import *

class car():
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.startSimulation()

        # self.motor_left = self.sim.getObject('left')
        # elf.motor_right = self.sim.getObject('right')

class motorBridge(Node):
    def __init__(self, motor_id):
        super().__init__(f'motor_{motor_id}_bridge')
        # self.publisher = self.create_publisher(JointState, f'/motor_{motor_id}_topic', 10)
        self.motorHandle = self.sim.getObject(f'motor_{motor_id}')

        self.motor_id = motor_id

        # Subscribe to a string topic
        self.subscription = self.create_subscription(
            String,
            '/string_topic',      # change to your topic name
            self.string_callback,
            10
        )
        self.publisher = self.create_publisher(JointState, f'/motor_{motor_id}_topic', 10)
        
        # # Connect to CoppeliaSim
        # self.client = RemoteAPIClient()
        # self.sim = self.client.getObject('sim')

        # # Find the object that has the child script
        # self.script_object = self.sim.getObject('/ROS')

        # self.get_logger().info('ROS2 → CoppeliaSim String bridge started.')

    def string_callback(self, msg: String):
        text = msg.data
        self.get_logger().info(f"Received string: {text}")

        # Call the Lua function "printMessage" in the script attached to /ROS
        # Call the "printMessage" child script in "ROS"
        self.sim.callScriptFunction(
            "printMessage@./ROS",
            self.sim.scripttype_sandbox,
            [],   # inputInts (or inputStrings depending on your script)
            [],   # inputFloats
            text  # inputStrings
        )


def main(args=None):
    rclpy.init(args=args)
    node = StringBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

