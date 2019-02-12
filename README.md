[[_TOC_]]
# Overview
ODVA confomrant Ethernet/IP interface drivers for Tolomatic stepper and servo controllers, where the ROS node is implimented as an Ethernet/IP adapter. Drivers depend on [odva_ethernetp]('https://github.com/ros-drivers/odva_ethernetip'), a ROS-ready library implimenting the Ethernet/IP protocol.

## Implimentation Limitations
The odva_ethernetp driver support of implicit messaging has limitations for multiple devices on the same network, so stepper and ACSI servo controller interface is implimented using explicit messaging only.

## ROS Distro Support

|         | Indigo | Jade | Kinetic | Melodic |
|:-------:|:------:|:----:|:-------:|:-------:|
| Branch  | [`indigo-devel`] | [`jade-devel`] | [`kinetic-devel`] | [`melodic-devel`] |
| Status  |  not tested | not tested |  supported |  testing |
| Version | [version] | [version] | [version] | [version] |

## Installation
'''
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
git clone https://github.com/wpmccormick/tolomatic.git src/tolomatic
wstool init src
wstool merge -t src src/tolomatic/.rosinstall
wstool update -t src
catkin build
source /opt/ros/kinetic/setup.bash
source devel/setup.bash
'''

## Usage
'''
roslaunch acsi_eip_driver servo.launch
roslaunch stepper_eip_driver stepper.launch
'''

## Launch Files
- stepper.launch
- servo.launch

# Nodes
## stepper_eip_driver/stepper_node
### Parameters
- host: The IP address of the controller
- local_ip: Local IP used for implicit messaging 
- joint_name:
- joint_states_topic:

### Published Topics
- stepper_inputs
```
float32 current_position
uint32 drive_status
uint32 drive_faults
uint32 digital_input
uint32 digital_output
float32 analog_input
float32 analog_output
```

- stepper_status
```
bool stopped
bool host_control
bool homed
bool enabled
bool moving
bool brake_off
bool in_position
float32 target_position
float32 current_position
```

### Advertised Services
- enable
```
bool enable
---
bool success
```

- profileMove
```
uint8 profile
---
bool success
```

- home
```
bool home
---
bool success
```

- stop
```
bool stop
---
bool success
```

## acsi_eip_driver/servo_node
### Parameters
- host: The IP address of the controller
- local_ip: Local IP used for implicit messaging
- joint_name:
- joint_states_topic:
- default_velocity:
- default_accel:
- default_decel:
- default_force:

### Published Topics
- acsi_inputs
```
float32 current_position
uint32 drive_status
uint32 drive_faults
uint32 digital_input
uint32 digital_output
float32 analog_input
float32 analog_output
```

- acsi_status
```
bool stopped
bool host_control
bool homed
bool enabled
bool moving
bool brake_off
bool in_position
float32 target_position
float32 position_error
float32 current_position
```

- sensor_msgs
```
JointState joint_states
```

### Advertised Services
- enable
```
bool enable
---
bool success
```

- estop
```
bool estop
---
bool success
```

- setHome
```
bool sethome
---
bool success
```

- setProfile
```
float32 velocity
float32 acceleration
float32 deceleration
float32 force
---
bool success
```

- moveAbsolute
```
float32 position
---
bool success
```

- moveHome
```
bool home
---
bool success
```

- moveIncremental
```
float32 increment
---
bool success
```

- moveRotary
```
float32 increment
---
bool success
```

- moveSelect
```
uint8 select
---
bool success
```

- moveStop
```
bool stop
---
bool success
```
#Creators
Bill McCormick - http://swri.org
