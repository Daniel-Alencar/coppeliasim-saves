# 📘 ROS 2 Bridge for Docking

## 🎯 Objective

In this activity, you will extend the ROS 2 Remote API bridge to provide the communication mechanisms required by an external autonomous docking controller.

Unlike the previous activity, where the docking algorithm could directly access CoppeliaSim functions and signals, the goal now is to make the required simulation information and commands available through ROS 2 topics.

> **You do not need to implement the autonomous docking behavior in this activity.**

Your task is only to implement the communication between CoppeliaSim and ROS 2 and verify that the required sensor information can be received and commands can be sent through ROS 2.

The resulting interface will be used in the next homework to implement the complete autonomous docking system as an external ROS 2 node.

---

## 🔄 CoppeliaSim–ROS 2 Communication

The bridge must use the CoppeliaSim ZeroMQ Remote API to access the information provided by the `myRobot` model and expose it through ROS 2.

The following communication mechanisms must be implemented:

1. Read the robot battery level from the float signal:

   ```python
   str(robotHandle) + "Battery"
   ```

2. Set the robot docking mode through the integer signal:

   ```python
   str(robotHandle) + "Docking"
   ```

3. Read the robot charging state through the integer signal:

   ```python
   str(robotHandle) + "Charging"
   ```

4. Read the charging-base IR beacon signal strength from the float signal:

   ```python
   str(robotHandle) + "signalStrength"
   ```

5. Read the charging-base relative angle from the float signal:

   ```python
   str(robotHandle) + "relativeAngle"
   ```

6. Receive robot velocity commands from ROS 2 and directly control the left and right wheel joints using the CoppeliaSim ZeroMQ Remote API.

The bridge therefore provides the interface between the simulated robot and external ROS 2 nodes.

---

## 🔋 Battery Bridge

The battery level is provided inside CoppeliaSim through:

```python
battery = sim.getFloatSignal(
    str(robotHandle) + "Battery"
)
```

The bridge must periodically read this value and publish it as a ROS 2 message.

This allows an external ROS 2 node to know the current battery state. Take a look at the battery msg from ROS2.

---

## 🔌 Charging State Bridge

The charging state is available through:

```python
charging = sim.getInt32Signal(
    str(robotHandle) + "Charging"
)
```

The bridge must read this value and publish the corresponding state through ROS 2.

An external controller can therefore determine whether the robot has successfully reached the charging station and is charging.

---

## 📡 Charging-Base Beacon Bridge

The charging station provides an IR beacon similar to the docking beacon of a robotic vacuum cleaner.

Inside CoppeliaSim, the beacon information is made available through two float signals associated with the robot:

```python
str(robotHandle) + "signalStrength"
str(robotHandle) + "relativeAngle"
```

The bridge must retrieve these values through the ZeroMQ Remote API using `sim.getFloatSignal()`:

```python
signalStrength = sim.getFloatSignal(
    str(robotHandle) + "signalStrength"
)

relativeAngle = sim.getFloatSignal(
    str(robotHandle) + "relativeAngle"
)
```

The two values have different purposes:

- **`signalStrength`** indicates how strongly the robot detects the charging-base beacon. A stronger signal indicates that the robot is closer to the charging station.

- **`relativeAngle`** indicates the direction of the charging base relative to the robot.

The angle convention is:

- `relativeAngle = 0` → charging base directly ahead;

- `relativeAngle > 0` → charging base on one side;

- `relativeAngle < 0` → charging base on the other side.

The bridge must periodically read these two CoppeliaSim signals and publish them through the corresponding ROS 2 topics:

```
/myRobot/charging_base/strengthSignal
/myRobot/charging_base/relativeAngle
```

Therefore, the beacon communication follows:

```
CoppeliaSim beacon
       │
       ▼
signalStrength / relativeAngle
       │
       ▼
sim.getFloatSignal(...)
       │
       ▼
ZeroMQ Remote API
       │
       ▼
ROS 2 Bridge
       │
       ▼
/myRobot/charging_base/strengthSignal
/myRobot/charging_base/relativeAngle
```

> The autonomous docking algorithm itself must not read these CoppeliaSim signals directly. It will receive the information exclusively through the ROS 2 topics in the next activity.

---

## ⚙️ Docking Mode Bridge

The docking mode must also be controllable from ROS 2.

When the corresponding ROS 2 command is received, the bridge must update the CoppeliaSim docking signal:

```python
sim.setInt32Signal(
    str(robotHandle) + "Docking",
    dockingMode
)
```

This mechanism will later allow an external ROS 2 controller to activate docking, for example when the battery becomes low.

---

## 🤖 Velocity Bridge

Robot motion commands must be received through the ROS 2 topic:

```
/myRobot/cmd_vel
```

using:

```
geometry_msgs/msg/Twist
```

The ROS 2 command provides the desired linear velocity and angular velocity of the robot.

The bridge must convert these values into the corresponding angular velocities of the left and right wheels according to the differential-drive kinematics of `myRobot`.

After calculating the wheel velocities, the bridge must directly actuate the corresponding CoppeliaSim joints using the ZeroMQ Remote API:

```python
sim.setJointTargetVelocity(leftMotorHandle, leftMotorVel)
sim.setJointTargetVelocity(rightMotorHandle, rightMotorVel)
```

No intermediate `leftVel` or `rightVel` CoppeliaSim signals are required.

---

## 🧩 Expected Architecture

After completing this activity, the communication structure should be:

```
                       CoppeliaSim
                            ▲
                            │
                     ZeroMQ Remote API
                            │
                            ▼
                 remoteAPI_ROS2_bridge
                            ▲
                            │
                       ROS 2 Topics
                            │
                            ▼
                    External ROS 2 Nodes
```

CoppeliaSim remains responsible for simulating the robot, battery, charging station, beacon, and physical interaction with the environment.

The ROS 2 bridge reads simulation information and directly sends actuator commands through the ZeroMQ Remote API.

The autonomous docking algorithm will be implemented separately in the next activity.

---

## 🧪 Testing the Bridge

Before implementing any autonomous behavior, verify that the ROS 2 interface works correctly.

Using ROS 2 command-line tools, you should be able to:

- observe the battery level;

- observe the charging state;

- observe the beacon signal strength;

- observe the relative angle to the charging base;

- activate and deactivate docking mode;

- send Twist velocity commands and observe the robot moving.

For example:

```bash
ros2 topic list
```

can be used to verify the available topics, while:

```bash
ros2 topic echo /myRobot/charging_base/strengthSignal
```

and:

```bash
ros2 topic echo /myRobot/charging_base/relativeAngle
```

can be used to verify that the beacon information is correctly transferred from CoppeliaSim to ROS 2.

At this stage, commands can be sent manually. No autonomous docking algorithm is required.

---

## ✅ Expected Result

At the end of this activity, all information and commands required for autonomous docking must be accessible through ROS 2.

Sensor information follows:

```
CoppeliaSim signals
    → sim.getFloatSignal()/sim.getInt32Signal()
    → ZeroMQ Remote API
    → ROS 2 Bridge
    → ROS 2 topics
```

Actuator commands follow:

```
ROS 2 /cmd_vel
    → ROS 2 Bridge
    → differential-drive conversion
    → sim.setJointTargetVelocity()
    → ZeroMQ Remote API
    → CoppeliaSim wheel joints
```

The next homework will use only these ROS 2 interfaces to implement autonomous docking.

---

## 📦 Expected Result

Implementation of ROS 2 bridge code and demonstrate that:

- battery information can be read through ROS 2;

- charging state can be read through ROS 2;

- beacon `signalStrength` can be read using `sim.getFloatSignal()` and published through ROS 2;

- beacon `relativeAngle` can be read using `sim.getFloatSignal()` and published through ROS 2;

- docking mode can be changed through ROS 2;

- Twist velocity commands can be sent through ROS 2 and correctly actuate the simulated robot using `sim.setJointTargetVelocity()` through the ZeroMQ Remote API.

> **Do not implement the autonomous docking behavior yet.**
