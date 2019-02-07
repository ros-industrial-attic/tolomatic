/**
Software License Agreement (BSD)

\file      stepper_node.cpp
\authors   Bill McCormick <wmccormick@swri.org>
\copyright

*/


#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <sensor_msgs/JointState.h>

#include "odva_ethernetip/socket/tcp_socket.h"
#include "odva_ethernetip/socket/udp_socket.h"

#include "stepper.h"

using std::cout;
using std::endl;
using boost::shared_ptr;
using eip::socket::TCPSocket;
using eip::socket::UDPSocket;

using namespace stepper_eip_driver;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "stepper");

  bool debug;
  ros::param::param<bool>("debug", debug, false);
  ROS_INFO_STREAM("debug is: " << debug);
  while(debug) {
    sleep(1);
    ros::param::get("debug", debug);
  }

  ros::Time::init();

  ros::Rate throttle(10);

  ros::NodeHandle nh;


  // get sensor config from params
  string host, local_ip;
  nh.param<std::string>("~host", host, "192.168.0.1");
  ROS_INFO_STREAM("Host is: " << host);
  nh.param<std::string>("~local_ip", local_ip, "192.168.0.104");

  // optionally publish ROS joint_state messages
  bool publish_joint_state;
  string joint_name, joint_states_topic;
  nh.param<bool>("~publish_joint_state", publish_joint_state, false);
  if (publish_joint_state)
  {
    nh.param<std::string>("~joint_name", joint_name, "x_axis");
    nh.param<std::string>("~joint_states_topic", joint_states_topic, "/joint_states");
  }

  boost::asio::io_service io_service;
  shared_ptr<TCPSocket> socket = shared_ptr<TCPSocket>(new TCPSocket(io_service));
  shared_ptr<UDPSocket> io_socket = shared_ptr<UDPSocket>(new UDPSocket(io_service, 0, local_ip));

  STEPPER stepper(socket, io_socket);

  ROS_INFO_STREAM("Socket created");

  try
  {
    stepper.open(host);
    ROS_INFO_STREAM("Host is open");
  }
  catch (std::runtime_error& ex)
  {
    ROS_FATAL_STREAM("Exception caught opening session: " << ex.what());
    return -1;
  }

  try
  {
    //get status
  }
  catch (std::invalid_argument& ex)
  {
    ROS_FATAL_STREAM("Invalid arguments in sensor configuration: " << ex.what());
    return -1;
  }

  try
  {
    //TODO: Setup implicit messaging here
    //stepper.startUDPIO();
    //ROS_INFO_STREAM("UDP Started");
  }
  catch (std::logic_error& ex)
  {
    ROS_FATAL_STREAM("Could not start UDP IO: " << ex.what());
    return -1;
  }

  // publisher for stepper status
  ros::Publisher stepper_pub = nh.advertise<stepper_inputs>("stepper_inputs", 1);
  ros::Publisher status_pub = nh.advertise<stepper_status>("stepper_status", 1);

  // publisher and message for joint state
  sensor_msgs::JointState joint_state;
  ros::Publisher joint_state_pub;
  if (publish_joint_state)
  {
    joint_state_pub = nh.advertise<sensor_msgs::JointState>(joint_states_topic, 1);
  }

  ros::ServiceServer enable_service = nh.advertiseService("enable", &STEPPER::enable, &stepper);
  ros::ServiceServer move_service = nh.advertiseService("profileMove", &STEPPER::moveProfile, &stepper);
  ros::ServiceServer home_service = nh.advertiseService("home", &STEPPER::home, &stepper);
  ros::ServiceServer stop_service = nh.advertiseService("stop", &STEPPER::stop, &stepper);

  //not implimented
  //ros::ServiceServer estop_service = nh.advertiseService("estop", &STEPPER::estop, &stepper);
  //ros::ServiceServer sethome_service = nh.advertiseService("setHome", &STEPPER::setHome, &stepper);

  while (ros::ok())
  {
    try
    {
      // Collect status from controller, convert to ROS message format.
      stepper.updateDriveStatus(stepper.getDriveData());

      if (publish_joint_state)
      {
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(1);
        joint_state.position.resize(1);
        joint_state.name[0] = joint_name;
        //TODO: See issue #2
        joint_state.position[0] = double(stepper.ss.current_position) / 1000.0;
        joint_state_pub.publish(joint_state);
      }

      //publish stepper inputs
      stepper_pub.publish(stepper.si);

      //publish stepper status
      status_pub.publish(stepper.ss);

      //set outputs to stepper drive controller
      stepper.setDriveData();
    }
    catch (std::runtime_error& ex)
    {
      ROS_ERROR_STREAM("Exception caught requesting scan data: " << ex.what());
    }
    catch (std::logic_error& ex)
    {
      ROS_ERROR_STREAM("Problem parsing return data: " << ex.what());
    }


    ros::spinOnce();

    throttle.sleep();

  }

  stepper.closeConnection(0);
  stepper.close();
  return 0;
}
