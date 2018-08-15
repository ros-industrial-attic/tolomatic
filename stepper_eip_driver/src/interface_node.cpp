#include "ros/ros.h"

#include "input_assembly.h"
#include "output_assembly.h"
#include <stepper_eip_driver/stepper_inputs.h>
#include <stepper_eip_driver/stepper_outputs.h>
#include <stepper_eip_driver/stepper_status.h>

#include <stepper_eip_driver/stepper_enable.h>
#include <stepper_eip_driver/stepper_estop.h>
#include <stepper_eip_driver/stepper_home.h>
#include <stepper_eip_driver/stepper_move.h>
#include <stepper_eip_driver/stepper_sethome.h>
#include <stepper_eip_driver/stepper_stop.h>
using namespace stepper_eip_driver;

stepper_outputs so;
stepper_status ss;

bool enable(stepper_eip_driver::stepper_enable::Request  &req,
            stepper_eip_driver::stepper_enable::Response &res)
{

  so.drive_command = (req.enable) ? (STEPPER_DRIVE_COMMAND)ENABLE : (STEPPER_DRIVE_COMMAND)DISABLE;
  res.success = true;

  return true;
}

bool move(stepper_eip_driver::stepper_move::Request  &req,
          stepper_eip_driver::stepper_move::Response &res)
{

  if(req.profile > 0) {
    so.drive_command = (STEPPER_DRIVE_COMMAND)START;
    so.move_select = req.profile;
    res.success = true;
  } else {
    res.success = false;
  }

  return true;
}

bool stop(stepper_eip_driver::stepper_stop::Request  &req,
          stepper_eip_driver::stepper_stop::Response &res)
{

  so.drive_command = (req.stop) ? (STEPPER_DRIVE_COMMAND)STOP : so.drive_command;
  res.success = true;

  return true;
}

bool home(stepper_eip_driver::stepper_home::Request  &req,
          stepper_eip_driver::stepper_home::Response &res)
{

  so.drive_command = (req.home) ? (STEPPER_DRIVE_COMMAND)GOHOME: so.drive_command;
  res.success = true;

  return true;
}


void status(const stepper_eip_driver::stepper_inputs::ConstPtr& status)
{
  ss.enabled      = status.get()->drive_status & (STEPPER_STATUS)ENABLE;
  ss.homed        = status.get()->drive_status & (STEPPER_STATUS)HOMED;
  ss.brake_off    = status.get()->drive_status & (STEPPER_STATUS)BRAKE_OFF;
  ss.host_control = status.get()->drive_status & (STEPPER_STATUS)HOST_CTRL;
  ss.moving       = status.get()->drive_status & (STEPPER_STATUS)MOTION;
  ss.stopped      = status.get()->drive_status & (STEPPER_STATUS)SSTOP;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interface_node");

  ros::Time::init();

  ros::Rate throttle(10);

  ros::NodeHandle nh;

  // publisher for stepper status
  ros::Publisher control_pub = nh.advertise<stepper_outputs>("stepper_control", 1);
  ros::Publisher status_pub = nh.advertise<stepper_status>("stepper_status", 1);

  ros::Subscriber sub = nh.subscribe("stepper_inputs", 1000, status);

  ros::ServiceServer enable_service = nh.advertiseService("enable", enable);
  ros::ServiceServer stop_service = nh.advertiseService("stop", stop);
  ros::ServiceServer move_service = nh.advertiseService("move", move);
  ros::ServiceServer home_service = nh.advertiseService("home", home);

  while (ros::ok())
  {
    try
    {
      // Publish stepper control.
      control_pub.publish(so);
      // Publish stepper status.
      status_pub.publish(ss);
    }
    catch (std::runtime_error ex)
    {
      ROS_ERROR_STREAM("Exception caught publishing stepper control data: " << ex.what());
    }
    catch (std::logic_error ex)
    {
      ROS_ERROR_STREAM("Problem parsing return data: " << ex.what());
    }


    ros::spinOnce();

    throttle.sleep();

  }

  return 0;
}
