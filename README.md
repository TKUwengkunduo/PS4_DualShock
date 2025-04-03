# PS4 DualShock Controller Tools

This project provides tools to test and visualize input from a PS4 (DualShock 4) controller using Python. It includes a simple input testing script and a speed/angular velocity output simulator for robotics or game development.

## Project Structure

```
PS4_DualShock/
├── python/
│   ├── ps4_test.py          # General controller input tester
│   └── velocity_output.py   # Speed and angular velocity simulator
└── ros2/
    └── src
        └── ps4              # ROS2 package
```

---

## 1. `ps4_test.py` - Button & Stick Input Tester

### Description:
- Print button and stick activity.
- Useful for verifying button mappings and axis IDs.

### How to Run:
```bash
cd python
python ps4_test.py
```

### Output Example:
```
[Button 0] Cross (X) pressed
[Axis 2] Left Stick X moved -> 0.65
```

---

## 2. `velocity_output.py` - Speed & Angular Velocity Simulator

### Description:
- Converts joystick input into linear and angular velocities.
- User sets maximum values interactively.

### How to Run:
```bash
cd python
python velocity_output.py
```

### Output Example:
```
Enter maximum speed (m/s): 2.0
Enter maximum angular speed (deg/s): 90
Speed: 1.20 m/s | Angular: -45.00 deg/s
```

---

## 3. `ps4_teleop_node.py` - ROS 2 Teleoperation Node

### Description:
- Publishes joystick values as `/cmd_vel` Twist messages.
- Compatible with ROS2 mobile robot stacks.

### How to Use:
1. Build the package:
```bash
cd ros2
colcon build
source install/setup.bash
```

2. Run with ROS2:
```bash
ros2 run ps4 ps4_velocity
ros2 run ps4 ps4_velocity --ros-args -p max_speed:=1.5 -p max_angular:=120.0
```

3. Subscribe `/cmd_vel` Node:
```bash
ros2 topic list
ros2 topic echo /cmd_vel
```

## Note
The numbers of the PS4 joysticks read by Linux and Windows are different. Therefore, the speed conversion function can only be used with the left joystick. If you need to change it, please reconfirm the numbers.