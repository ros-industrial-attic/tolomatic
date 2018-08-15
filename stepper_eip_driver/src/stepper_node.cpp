/**
Software License Agreement (BSD)

\file      os32c_node.cpp
\authors   Kareem Shehata <kareem@shehata.ca>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

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
  ros::param::param<std::string>("~host", host, "192.168.0.1");

  ROS_INFO_STREAM("Host is: " << host);
  ros::param::param<std::string>("~local_ip", local_ip, "192.168.0.104");

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

  ros::ServiceServer enable_service = nh.advertiseService("enable", &STEPPER::enable, &stepper);
  ros::ServiceServer stop_service = nh.advertiseService("stop", &STEPPER::stop, &stepper);
  ros::ServiceServer estop_service = nh.advertiseService("estop", &STEPPER::estop, &stepper);
  ros::ServiceServer move_service = nh.advertiseService("profileMove", &STEPPER::moveProfile, &stepper);
  ros::ServiceServer home_service = nh.advertiseService("home", &STEPPER::home, &stepper);
  ros::ServiceServer sethome_service = nh.advertiseService("setHome", &STEPPER::setHome, &stepper);

  while (ros::ok())
  {
    try
    {
      // Collect status from controller, convert to ROS message format.
      stepper.updateDriveStatus(stepper.getDriveData());

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
