import curses

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import String

# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import *

class car:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.motorHandle = self.sim.getObject('/myRobot/leftMotor')
        self.rightMotorHandle = self.sim.getObject('/myRobot/rightMotor')
        self.sensorNariz = self.sim.getObject('/myRobot/proximitySensor')
        started_here = self.sim.getSimulationState() == self.sim.simulation_stopped
        try:
            if started_here:
                self.sim.startSimulation()
            curses.wrapper(self.control)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                self.sim.setJointTargetVelocity(self.motorHandle, 0.0)
            finally:
                try:
                    self.sim.setJointTargetVelocity(self.rightMotorHandle, 0.0)
                finally:
                    if started_here:
                        self.sim.stopSimulation()

    def control(self, screen):
        # Terminal input has no key-release events. Stop after 0.3 s without
        # movement input; holding a key uses the operating system's key repeat.
        screen.keypad(True)
        screen.timeout(50)
        commands = {
            ord('w'): (1.0, 1.0), curses.KEY_UP: (1.0, 1.0),
            ord('s'): (-1.0, -1.0), curses.KEY_DOWN: (-1.0, -1.0),
            ord('a'): (-1.0, 1.0), curses.KEY_LEFT: (-1.0, 1.0),
            ord('d'): (1.0, -1.0), curses.KEY_RIGHT: (1.0, -1.0),
        }
        velocity = (0.0, 0.0)
        deadline = 0.0
        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            key = screen.getch()
            if key in (ord('q'), ord('Q'), 27):
                break
            if key == ord(' '):
                velocity = (0.0, 0.0)
                deadline = 0.0
                curses.flushinp()
            elif key in commands:
                velocity = commands[key]
                deadline = time.monotonic() + 0.3
            if time.monotonic() >= deadline:
                velocity = (0.0, 0.0)
            self.sim.setJointTargetVelocity(self.motorHandle, velocity[0])
            self.sim.setJointTargetVelocity(self.rightMotorHandle, velocity[1])
            sensor = self.sim.readProximitySensor(self.sensorNariz)
            screen.erase()
            lines = [
                'WASD / arrows: drive | Space: stop | Q / Esc: quit',
                'Hold a movement key to repeat. Input timeout: 0.3 seconds.',
                f'Wheel velocities: {velocity}',
                f'Proximity sensor: {sensor}',
            ]
            height, width = screen.getmaxyx()
            for row, line in enumerate(lines[:max(0, height - 1)]):
                screen.addnstr(row, 0, line, max(0, width - 1))
            screen.refresh()   



def main(args=None):
    car2 = car()





if __name__ == '__main__':
    main()
