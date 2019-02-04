/**

\file      stepper.cpp
\authors   Bill McCormick <wmccormick@swri.org>
\copyright
*/


#include <ros/ros.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/asio.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include "servo.h"

#include "odva_ethernetip/serialization/serializable_buffer.h"
#include "odva_ethernetip/cpf_packet.h"
#include "odva_ethernetip/cpf_item.h"
#include "odva_ethernetip/sequenced_address_item.h"
#include "odva_ethernetip/sequenced_data_item.h"

using std::cout;
using std::endl;

using boost::shared_ptr;
using boost::make_shared;
using boost::asio::buffer;

using eip::Session;
using eip::serialization::SerializableBuffer;
using eip::RRDataResponse;
using eip::CPFItem;
using eip::CPFPacket;
using eip::SequencedAddressItem;
using eip::SequencedDataItem;

namespace acsi_eip_driver {

InputAssembly ACSI::getDriveData()
{
  InputAssembly ia;
  //TODO: put Attr ID, Assmy ID, and Inst ID in header
  //0x04 = Object ID
  //0x64 = 100 Instance
  //3 - Attr ID
  getSingleAttributeSerializable(0x04, 0x64, 3, ia);

  si.current_position = ia.current_position;
  si.drive_status = ia.drive_status;
  si.drive_faults = ia.drive_faults;
  si.analog_input = ia.analog_input;
  si.analog_output = ia.analog_output;
  si.digital_input = ia.digital_input;
  si.digital_output = ia.digital_output;

  return ia;
}

void ACSI::updateDriveStatus(InputAssembly ia)
{
  ss.enabled      = (ia.drive_status & ENABLE) > 0;
  ss.homed        = (ia.drive_status & HOMED) > 0;
  ss.brake_off    = (ia.drive_status & BRAKE_OFF) > 0;
  ss.host_control = (ia.drive_status & HOST_CTRL) > 0;
  ss.moving       = (ia.drive_status & MOTION) > 0;
  ss.stopped      = (ia.drive_status & SSTOP) > 0;
  ss.current_position = ia.current_position;

  //if we just started moving and all is well
  if(ss.enabled) {
    if(!ss.stopped && ss.moving && !ss_last.moving) {
      //revert the drive command to a simple enable
      ss.target_position = si.analog_input;
      ss.in_position = false;
    } else if (!ss.moving && ss_last.moving) {
      if(std::abs(si.current_position - si.analog_output) <= float(IN_POSITION_TOLERANCE)) {
        ss.in_position = true;
      }
    }
  }

  memcpy(&ss_last,&ss, sizeof(ss));
}

//TODO: make this use one of either OutputAssemblies (i.e. Full or not Full)
void ACSI::setDriveData()
{
  OutputAssembly oa;

  oa.drive_command = so.drive_command;
  oa.move_select = so.move_select;
  oa.accel = so.accel;
  oa.decel = so.decel;
  oa.force = so.force;
  oa.outputs = so.outputs;
  oa.postion = so.position;
  oa.velocity = so.velocity;
  oa.motion_type = so.motion_type;

  shared_ptr<OutputAssembly> sb = make_shared<OutputAssembly>(oa);

  //TODO: put Attr ID, Assmy ID, and Inst ID in header
  //0x04 = Object ID
  //0x71 = 113 Instance
  //3 - Attr ID
  setSingleAttributeSerializable(0x04, 0x71, 3, sb);

  //need to make sure the START drive command changes back to START so that
  //the next START will work
  if(so.drive_command == START) so.drive_command = ENABLE;
}

bool ACSI::enable(acsi_eip_driver::acsi_enable::Request  &req,
                     acsi_eip_driver::acsi_enable::Response &res)
{

  if(!ss.host_control) {
    so.drive_command = (req.enable) ? ENABLE : DISABLE;
    return res.success = true;
  } else {
    return res.success = false;
  }
}

bool ACSI::moveHome(acsi_eip_driver::acsi_moveHome::Request  &req,
                   acsi_eip_driver::acsi_moveHome::Response &res)
{

  if(!ss.host_control) {
    so.drive_command = (req.home) ? GOHOME: so.drive_command;
    return res.success = true;
  } else {
    return res.success = false;
  }
}

bool ACSI::moveStop(acsi_eip_driver::acsi_moveStop::Request  &req,
                   acsi_eip_driver::acsi_moveStop::Response &res)
{

  if(!ss.host_control) {
    so.drive_command = (req.stop) ? STOP : so.drive_command;
    return res.success = true;
  } else {
    return res.success = false;
  }
}

bool ACSI::setHome(acsi_eip_driver::acsi_setHome::Request  &req,
                      acsi_eip_driver::acsi_setHome::Response &res)
{

  if(!ss.host_control) {
    so.drive_command = (req.sethome) ? HOME_HERE: so.drive_command;
    return res.success = true;
  } else {
    return res.success = false;
  }

}

bool ACSI::setProfile(acsi_eip_driver::acsi_setProfile::Request &req, 
                      acsi_eip_driver::acsi_setProfile::Response &res)
{
    if(!ss.host_control) {
        so.velocity = req.velocity;
        so.accel = req.acceleration;
        so.decel = req.deceleration;
        so.force = req.force;
      return res.success = true;
    } else {
      return res.success = false;
    }
}

bool ACSI::moveVelocity(acsi_eip_driver::acsi_moveVelocity::Request &req, 
                        acsi_eip_driver::acsi_moveVelocity::Response &res)
{
    if(!ss.host_control) {
        so.drive_command = START;
        if(req.velocity > 0)
            so.motion_type = VELOCITY_FWD;
        else if ((req.velocity < 0))
            so.motion_type = VELOCITY_REV;
        else {
            so.motion_type = NO_ACTION;
        }
        so.velocity = req.velocity;

      return res.success = true;
    } else {
      return res.success = false;
    }
}

bool ACSI::moveAbsolute(acsi_eip_driver::acsi_moveAbsolute::Request  &req,
                   acsi_eip_driver::acsi_moveAbsolute::Response &res)
{
    if(!ss.host_control) {
        so.drive_command = START;
        if(req.position > 0)
            so.motion_type = ABSOLUTE;
        else {
            so.motion_type = NO_ACTION;
        }
        so.position = req.position;

      return res.success = true;
    } else {
      return res.success = false;
    }
}

bool ACSI::moveIncremental(acsi_eip_driver::acsi_moveIncremental::Request  &req,
                 acsi_eip_driver::acsi_moveIncremental::Response &res)
{
    if(!ss.host_control) {
        so.drive_command = START;
        if(req.increment > 0)
            so.motion_type = INC_POSITIVE;
        else if (req.increment < 0)
            so.motion_type = INC_NEGATIVE;
        else
            so.motion_type = NO_ACTION;

        so.position = req.increment;

      return res.success = true;
    } else {
      return res.success = false;
    }

}

bool ACSI::moveRotary(acsi_eip_driver::acsi_moveRotary::Request  &req,
                 acsi_eip_driver::acsi_moveRotary::Response &res)
{
    if(!ss.host_control) {
        so.drive_command = START;
        if(req.increment > 0)
            so.motion_type = INC_POS_ROTARY;
        else if (req.increment < 0)
            so.motion_type = INC_NEG_ROTARY;
        else
            so.motion_type = NO_ACTION;

        so.position = req.increment;

      return res.success = true;
    } else {
      return res.success = false;
    }
}

bool ACSI::moveSelect(acsi_eip_driver::acsi_moveSelect::Request  &req,
                          acsi_eip_driver::acsi_moveSelect::Response &res)
{
  ROS_INFO_STREAM("Move select: " << req.select);
  if(!ss.host_control && req.select > 0 && req.select <= 16) {
    so.drive_command = START;
    so.move_select = req.select;
    return res.success = true;
  } else {
    return res.success = false;
  }

  return true;
}


//not implimented
//bool STEPPER::estop(stepper_eip_driver::stepper_estop::Request  &req,
//                    stepper_eip_driver::stepper_estop::Response &res)
//{
//
//  if(!ss.host_control) {
//    so.drive_command = (req.e_stop) ? (STEPPER_DRIVE_COMMAND)ESTOP : so.drive_command;
//    return res.success = true;
//  } else {
//    return res.success = false;
//  }
//
//}
//
//
//void STEPPER::startUDPIO()
//{
//  EIP_CONNECTION_INFO_T o_to_t, t_to_o;
//  o_to_t.assembly_id = 0x70;
//  o_to_t.buffer_size = 0x0004;
//  o_to_t.rpi = 0x000186A0;
//  t_to_o.assembly_id = 0x64;
//  t_to_o.buffer_size = 0x001c;
//  t_to_o.rpi = 0x0000C350;
//
//  connection_num_ = createConnection(o_to_t, t_to_o);
//}

} // namespace os32c
