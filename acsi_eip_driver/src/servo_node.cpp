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

#include "servo.h"

using std::cout;
using std::endl;
using boost::shared_ptr;
using eip::socket::TCPSocket;
using eip::socket::UDPSocket;

using namespace acsi_eip_driver;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "servo");

  bool debug;
  ros::param::param<bool>("debug", debug, true);
  ROS_INFO_STREAM("debug is: " << debug);
  while(debug) {
    sleep(1);
    ros::param::get("debug", debug);
  }

  ros::Time::init();

  ros::Rate throttle(10);

  ros::NodeHandle nh("~");


  // get sensor config from params
  string host, local_ip;
  nh.param<std::string>("host", host, "192.168.100.10");
  ROS_INFO_STREAM("Host is: " << host);

  nh.param<std::string>("local_ip", local_ip, "192.168.100.2");

  // optionally publish ROS joint_state messages
  bool publish_joint_state;
  string joint_name, joint_states_topic;
  nh.param<bool>("publish_joint_state", publish_joint_state, false);
  if (publish_joint_state)
  {
    nh.param<std::string>("joint_name", joint_name, "drive1");
    nh.param<std::string>("joint_states_topic", joint_states_topic, "/joint_states");
  }


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

  float default_accel;
  nh.param<float>("default_accel", default_accel, 10.0);
  servo.so.accel = default_accel;

  float default_decel;
  nh.param<float>("default_decel", default_decel, 10.0);
  servo.so.decel = default_decel;

  float default_force;
  nh.param<float>("default_force", default_force, 10.0);
  servo.so.force = default_force;

  float default_velocity;
  nh.param<float>("default_velocity", default_velocity, 10.0);
  servo.so.velocity = default_velocity;

  // publisher for stepper status
  ros::Publisher servo_pub = nh.advertise<acsi_inputs>("acsi_inputs", 1);
  ros::Publisher status_pub = nh.advertise<acsi_status>("acsi_status", 1);

  // publisher and message for joint state
  sensor_msgs::JointState joint_state;
  ros::Publisher joint_state_pub;
  if (publish_joint_state)
  {
    joint_state_pub = nh.advertise<sensor_msgs::JointState>(joint_states_topic, 1);
  }


  //services
  ros::ServiceServer enable_service = nh.advertiseService("enable", &ACSI::enable, &servo);
  ros::ServiceServer moveSelect_service = nh.advertiseService("moveSelect", &ACSI::moveSelect, &servo);
  ros::ServiceServer moveHome_service = nh.advertiseService("moveHome", &ACSI::moveHome, &servo);
  ros::ServiceServer moveStop_service = nh.advertiseService("moveStop", &ACSI::moveStop, &servo);
  ros::ServiceServer moveVelocity_service = nh.advertiseService("moveVelocity", &ACSI::moveVelocity, &servo);
  ros::ServiceServer moveAbsolute_service = nh.advertiseService("moveAbsolute", &ACSI::moveAbsolute, &servo);
  ros::ServiceServer moveIncremental_service = nh.advertiseService("moveIncremental", &ACSI::moveIncremental, &servo);
  ros::ServiceServer moveRotary_service = nh.advertiseService("moveRotary", &ACSI::moveRotary, &servo);
  ros::ServiceServer setHome_service = nh.advertiseService("setHome", &ACSI::setHome, &servo);
  ros::ServiceServer setProfile_service = nh.advertiseService("setProfile", &ACSI::setProfile, &servo);

  //not implimented
  //ros::ServiceServer estop_service = nh.advertiseService("estop", &STEPPER::estop, &stepper);

  while (ros::ok())
  {
    try
    {
      // Collect status from controller, convert to ROS message format.
      servo.updateDriveStatus(servo.getDriveData());

      if (publish_joint_state)
      {
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(1);
        joint_state.position.resize(1);
        joint_state.name[0] = joint_name;
        //TODO: See issue #2
        joint_state.position[0] = double(servo.ss.current_position) / 1000.0;
        joint_state_pub.publish(joint_state);
      }

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
