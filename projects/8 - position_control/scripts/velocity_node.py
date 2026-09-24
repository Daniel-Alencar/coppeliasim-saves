import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class SubscriberVelocity(Node):
    def __init__(self):
        super().__init__('subscriber_velocity')
        self.subscription = self.create_subscription(Twist, 'myRobot/cmd_vel', self.msg_callback, 10)
        self.wheel_base = 0.331  # meters
        self.radius = 0.195/2    # meters
        timer_interval = 0.4     # seconds
         # Connect to CoppeliaSim
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.get_logger().info('Connected to CoppeliaSim successfully.')

            # Get robot and motor handles
            self.robotHandle = self.sim.getObject('/myRobot')
            self.leftMotor = self.sim.getObject('/leftMotor')
            self.rightMotor = self.sim.getObject('/rightMotor')

            self.sim.startSimulation()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CoppeliaSim: {e}')
            return

        # Create flag for the message subscription callback status
        self.msg_flag = False
        # Create timer to send velocity to robot
        self.timer = self.create_timer(timer_interval, self.send_velocity)

    def msg_callback(self, msg):
        linVel = msg.linear.x
        rotVel = msg.angular.z
        # Inverse kinematics for differential wheeled robot
        self.rightVel = (linVel + self.wheel_base /2 * rotVel)/self.radius
        self.leftVel  = (linVel - self.wheel_base /2 * rotVel)/self.radius
        self.msg_flag = True

    def send_velocity(self):
        # If there is not a new message, send 0 velocity
        if (self.msg_flag == False):
            self.rightVel, self.leftVel = 0, 0
        self.sim.setJointTargetVelocity(self.rightMotor, self.rightVel)
        self.sim.setJointTargetVelocity(self.leftMotor, self.leftVel)
        self.msg_flag = False

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberVelocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
