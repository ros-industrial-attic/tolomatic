/**
Software License Agreement (BSD)

\file      stepper_node.cpp
\authors   Bill McCormick <wmccormick@swri.org>
\copyright

*/


#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include "odva_ethernetip/socket/tcp_socket.h"
#include "odva_ethernetip/socket/udp_socket.h"

#include "servo.h"

using std::cout;
using std::endl;
using boost::shared_ptr;
using eip::socket::TCPSocket;
using eip::socket::UDPSocket;

using namespace acsi_eip_driver;

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
  ros::param::param<std::string>("~host", host, "192.168.100.10");

  ROS_INFO_STREAM("Host is: " << host);
  ros::param::param<std::string>("~local_ip", local_ip, "192.168.100.2");

  boost::asio::io_service io_service;
  shared_ptr<TCPSocket> socket = shared_ptr<TCPSocket>(new TCPSocket(io_service));
  shared_ptr<UDPSocket> io_socket = shared_ptr<UDPSocket>(new UDPSocket(io_service, 0, local_ip));

  ACSI servo(socket, io_socket);

  ROS_INFO_STREAM("Socket created");

  try
  {
    servo.open(host);
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
  ros::Publisher servo_pub = nh.advertise<acsi_inputs>("acsi_inputs", 1);
  ros::Publisher status_pub = nh.advertise<acsi_status>("acsi_status", 1);

  ros::ServiceServer enable_service = nh.advertiseService("enable", &ACSI::enable, &servo);
  ros::ServiceServer move_service = nh.advertiseService("moveSelect", &ACSI::moveSelect, &servo);
  ros::ServiceServer home_service = nh.advertiseService("home", &ACSI::home, &servo);
  ros::ServiceServer stop_service = nh.advertiseService("stop", &ACSI::stop, &servo);

  //not implimented
  //ros::ServiceServer estop_service = nh.advertiseService("estop", &STEPPER::estop, &stepper);
  //ros::ServiceServer sethome_service = nh.advertiseService("setHome", &STEPPER::setHome, &stepper);

  while (ros::ok())
  {
    try
    {
      // Collect status from controller, convert to ROS message format.
      servo.updateDriveStatus(servo.getDriveData());

      //publish stepper inputs
      servo_pub.publish(servo.si);

      //publish stepper status
      status_pub.publish(servo.ss);

      //set outputs to stepper drive controller
      servo.setDriveData();
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

  servo.closeConnection(0);
  servo.close();
  return 0;
}
