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
- host
- local_ip
### Published Topics
- stepper_inputs
- stepper_status
### Advertised Services
- enable
- profileMove
- home
- stop

