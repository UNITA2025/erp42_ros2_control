# erp42_ros2_control
A ROS 2-based control and interface package for the ERP42 autonomous vehicle platform, providing essential modules for communication and operation.

# Package Overview
### erp42_control_pkg
This package implements the serial communication interface between the ERP42 vehicle and the ROS 2 system. It translates high-level commands into the ERP42’s protocol and decodes incoming status data.

* Main Node: ErpSerialHandler_node

* Helper Module: ByteHandler.py

* Launch File: erp42_base.launch.py

* Main Tasks:

    * Convert ErpCmdMsg to byte stream and transmit it via serial

  * Parse incoming byte stream and publish it as ErpStatusMsg

### erp42_interfaces_pkg
This package contains ROS 2 .msg definitions for control and feedback. These messages are used by other nodes in the system to send commands and read status from ERP42.

# Data Format
### Transmission Data Format (ErpCmdMsg → Serial Bytes)

```python
"""
┌─────┬─────┬─────┬─────┬─────┬──────┬───────┬───────┬────┬─────┬──────┬──────┐
│  S  │  T  │  X  │ A,M │  E  │ Gear │ Speed │ Steer │ Br │ Al  │ ETX0 │ ETX1 │
├─────┼─────┼─────┼─────┼─────┼──────┼───────┼───────┼────┼─────┼──────┼──────┤
│0x53 │0x54 │0x58 │0 , 1│0 , 1│0,1,2 │0 ~ 200│ ±2000 │1~33│0~255│ 0x0D │ 0x0A │
└─────┴─────┴─────┴─────┴─────┴──────┴───────┴───────┴────┴─────┴──────┴──────┘
```

### Reception Data Format (Serial Bytes → ErpStatusMsg)

```python
"""
┌─────┬─────┬─────┬─────┬─────┬──────┬───────┬───────┬────┬───────┬─────┬──────┬──────┐
│  S  │  T  │  X  │ A,M │  E  │ Gear │ Speed │ Steer │ Br │  ENC  │ Al  │ ETX0 │ ETX1 │
├─────┼─────┼─────┼─────┼─────┼──────┼───────┼───────┼────┼───────┼─────┼──────┼──────┤
│0x53 │0x54 │0x58 │0 , 1│0 , 1│0,1,2 │0 ~ 200│ ±2000 │1~33│ ±2^31 │0~255│ 0x0D │ 0x0A │
└─────┴─────┴─────┴─────┴─────┴──────┴───────┴───────┴────┴───────┴─────┴──────┴──────┘
"""
```


# Message Definitions
### ErpStatusMsg.msg (from ERP42)
```msg
int8   control_mode  # Control mode: 0 = Manual, 1 = Autonomous, etc.  
bool   e_stop        # Emergency stop signal  
uint8  gear          # Gear state: 0 = Neutral, 1 = Forward, 2 = Reverse  
uint8  speed         # Speed value in range 0 ~ 200  
int32  steer         # Steering angle in range ±2000  
uint8  brake         # Brake intensity in range 0 ~ 33  
int32  encoder       # Wheel encoder count (e.g., number of pulses or rotations)  
uint8  alive         # Alive counter for monitoring communication status  

```
### ErpCmdMsg.msg (to ERP42)
```msg
bool   e_stop     # Emergency stop signal  
uint8  gear       # Gear state: 0 = Neutral, 1 = Forward, 2 = Reverse  
uint8  speed      # Speed value in range 0 ~ 200  
int32  steer      # Steering angle in range ±2000  
uint8  brake      # Brake intensity in range 0 ~ 33  
```

### Build & Launch Instructions
From the workspace root:
```sh
cd ~/erp42_ros2_control
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch erp42_control_pkg erp42_base.launch.py
```

### Maintainer
Author: Hyeongjin Kim  
Email: ky942400@gmail.com  
License: MIT  