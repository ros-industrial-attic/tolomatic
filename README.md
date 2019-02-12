# Overview
ODVA confomrant Ethernet/IP interface drivers for Tolomatic stepper and servo controllers, where the ROS node is implimented as an Ethernet/IP adapter. Drivers depend on [odva_ethernetp]('https://github.com/ros-drivers/odva_ethernetip'), a ROS-ready library implimenting the Ethernet/IP protocol.

Topics named _\*\_inputs_ are a direct echo of interface produced values, while _\*\_status_ topics are derived from elements of these inputs. Note that, in the case of _servo_status_, _target_postion_ and _position_error_ require TMI Network Assembly Remapping of Register 1 (Commanded Postion) and Register 2 (Actual Postion Error). For stepper, default mapping is used.

## Implimentation Limitations
The [odva_ethernetp](https://github.com/ros-drivers/odva_ethernetip) driver support of implicit messaging has limitations for multiple devices on the same network, so stepper and ACSI servo controller interface is implimented using explicit messaging only.

## Reference
* See config dirs for TMI configuration examples: [stepper](stepper_eip_driver/config) and [servo](acsi_eip_driver/config)
* For latest docs and TMI software: [Tolomatic Support](https://www.tolomatic.com/info-center/product-support.aspx)
* Archive docs: [stepper](stepper_eip_driver/doc) and [servo](acsi_eip_driver/doc)
* [odva_ehternetip](https://github.com/ros-drivers/odva_ethernetip) driver

## ROS Distro Support

|         | Indigo | Jade | Kinetic | Melodic |
|:-------:|:------:|:----:|:-------:|:-------:|
| Branch  | [`indigo-devel`] | [`jade-devel`] | [`kinetic-devel`] | [`melodic-devel`] |
| Status  |  not tested | not tested |  supported |  testing |
| Version | [version] | [version] | [version] | [version] |

## Installation
```
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
```

## Usage
```
roslaunch acsi_eip_driver servo.launch
roslaunch stepper_eip_driver stepper.launch
```

## Launch Files
- stepper.launch
- servo.launch

# Nodes
## stepper_node
The stepper_node uses the compact output assembly, and so requires TMI configuration under _Mode Setup_ for each desired move selection. At some point, the compact output assemply could be replaced by the _full assemply_, allowing for enhanced control features. See the manual for more detailed info.
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

- joint_states (sensor_msgs/JointState)

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

## servo_node
The servo_node uses the full output assembly, where each service (generally) impliments a specific motion type. For service call parameters velocity, postion, and incriment, the parameter sign will control drive direction. See the manual for more detailed info.
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

- joint_states (sensor_msgs/JointState)

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

- moveVelocity
```
float32 velocity
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
# Creators
Bill McCormick - http://swri.org
