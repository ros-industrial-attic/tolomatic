# Overview
ODVA confomrant Ethernet/IP interface drivers for Tolomatic stepper and servo controllers, where the ROS node is implimented as an Ethernet/IP adapter. Drivers depend on odva_ethernetp ['https://github.com/ros-drivers/odva_ethernetip'], a ROS-ready library implimenting the Ethernet/IP protocol.

## Implimentation Limitations
Currently, only a stepper controller interface is currently implimented using explicit messaging, as the odva_ethernetp driver support of implicit messaging has limitations for multiple devices on the same network.

## ROS Distro Support


|         | Indigo | Jade | Kinetic |
|:-------:|:------:|:----:|:-------:|
| Branch  | [`indigo-devel`] | [`jade-devel`] | [`kinetic-devel`] |
| Status  |  not tested | not tested |  supported |
| Version | [version] | [version] | [version] |


# Nodes

## stepper_eip_driver/stepper_node
### Parameters
- host: The IP address of the controller
- local_ip: Local IP used for implicit messaging

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
```
- enable
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
