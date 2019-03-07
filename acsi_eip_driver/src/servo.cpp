/**
 * @file servo.cpp
 * @brief Tolomatic ACSI servo interface using Ethernet/IP - servo functions
 *class.
 *
 * @author Bill McCormick <wmccormick@swri.org>
 * @date Feb 13, 2019
 * @version 0.1
 * @bug Implicit messaging not functional
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>  // std::abs
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

namespace acsi_eip_driver
{
InputAssembly ACSI::getDriveData()
{
  InputAssembly ia;
  // TODO: put Attr ID, Assmy ID, and Inst ID in header
  // 0x04 = Object ID
  // 0x64 = 100 Instance
  // 3 - Attr ID
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

// update the ROS drive status message
// requires regs are remapped:Remappable Reg1=Commanded Position and Remappable
// Reg2=Position Error
void ACSI::updateDriveStatus(InputAssembly ia)
{
  ss.enabled = (ia.drive_status & ENABLE) > 0;
  ss.homed = (ia.drive_status & HOMED) > 0;
  ss.brake_off = (ia.drive_status & BRAKE_OFF) > 0;
  ss.host_control = (ia.drive_status & HOST_CTRL) > 0;
  ss.moving = (ia.drive_status & MOTION) > 0;
  ss.stopped = (ia.drive_status & SSTOP) > 0;
  ss.in_position = (ia.drive_status & IN_POSITION) > 0;
  ss.current_position = ia.current_position;
  // these 2 require remapping using the Tolomatic Windows configuration
  // software
  ss.position_error = ia.analog_output;
  ss.target_position = si.analog_input;

  memcpy(&ss_last, &ss, sizeof(ss));
}

// TODO: make this use one of either OutputAssemblies (i.e. Full or not Full)
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

  // TODO: put Attr ID, Assmy ID, and Inst ID in header
  // 0x04 = Object ID
  // 0x71 = 113 Instance
  // 3 - Attr ID
  setSingleAttributeSerializable(0x04, 0x71, 3, sb);

  // need to make sure the START drive command changes back to ENABLE so that
  // the next START will work
  switch (so.drive_command)
  {
    case ENABLE:
      so.drive_command = ENABLE;
      break;
    case START:
      so.drive_command = ENABLE;
      break;
    case GOHOME:
      so.drive_command = GOHOME;
      break;
    case ESTOP:
      so.drive_command = ESTOP;
      break;
    case STOP:
      so.drive_command = STOP;
      break;
    case HOME_HERE:
      so.drive_command = ENABLE;
      break;
    default:
      so.drive_command = DISABLE;
  }
}

bool ACSI::enable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if (!ss.host_control)
  {
    so.drive_command = (req.data) ? ENABLE : DISABLE;
    res.message = "Drive enabled";
    return res.success = true;
  }
  else
  {
    res.message = "Cannot command drive";
    return res.success = false;
  }
}

bool ACSI::estop(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if (!ss.host_control)
  {
    so.drive_command = (req.data) ? ESTOP : so.drive_command;
    res.message = "Drive stopped";
    return res.success = true;
  }
  else
  {
    res.message = "Cannot command drive";
    return res.success = false;
  }
}

bool ACSI::moveHome(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (!ss.host_control)
  {
    so.drive_command = GOHOME;
    so.motion_type = HOME;
    res.message = "Moving Home";
    return res.success = true;
  }
  else
  {
    res.message = "Cannot command drive";
    return res.success = false;
  }
}

bool ACSI::moveStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (!ss.host_control)
  {
    so.drive_command = STOP;
    res.message = "Stopping";
    return res.success = true;
  }
  else
  {
    res.message = "Cannot command drive";
    return res.success = false;
  }
}

bool ACSI::setHome(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (!ss.host_control)
  {
    so.drive_command = HOME_HERE;
    res.message = "Set home";
    return res.success = true;
  }
  else
  {
    res.message = "Cannot command drive";
    return res.success = false;
  }
}

bool ACSI::setProfile(acsi_eip_driver::acsi_setProfile::Request& req,
                      acsi_eip_driver::acsi_setProfile::Response& res)
{
  if (!ss.host_control)
  {
    so.velocity = req.velocity;
    so.accel = req.acceleration;
    so.decel = req.deceleration;
    so.force = req.force;
    res.message = "Profile set";
    return res.success = true;
  }
  else
  {
    res.message = "Cannot command drive";
    return res.success = false;
  }
}

bool ACSI::moveVelocity(acsi_eip_driver::acsi_moveVelocity::Request& req,
                        acsi_eip_driver::acsi_moveVelocity::Response& res)
{
  if (!ss.host_control && ss.enabled)
  {
    so.drive_command = START;
    if (req.velocity > 0)
    {
      res.message = "Velocity move forward";
      so.motion_type = VELOCITY_FWD;
    }
    else if ((req.velocity < 0))
    {
      res.message = "Velocity move reverse";
      so.motion_type = VELOCITY_REV;
    }
    else
    {
      res.message = "Velocity zero";
    }
    so.velocity = std::abs(req.velocity);
    return res.success = true;
  }
  else
  {
    res.message = "Cannot command drive";
    return res.success = false;
  }
}

bool ACSI::moveAbsolute(acsi_eip_driver::acsi_moveAbsolute::Request& req,
                        acsi_eip_driver::acsi_moveAbsolute::Response& res)
{
  if (!ss.host_control && ss.enabled)
  {
    so.drive_command = START;
    so.motion_type = ABSOLUTE;
    so.position = req.position;

    res.message = "Move abolute";
    return res.success = true;
  }
  else
  {
    res.message = "Cannot command drive";
    return res.success = false;
  }
}

bool ACSI::moveIncremental(acsi_eip_driver::acsi_moveIncremental::Request& req,
                           acsi_eip_driver::acsi_moveIncremental::Response& res)
{
  if (!ss.host_control && ss.enabled)
  {
    so.drive_command = START;
    if (req.increment > 0)
    {
      res.message = "Incremental move forward";
      so.motion_type = INC_POSITIVE;
    }
    else if (req.increment < 0)
    {
      res.message = "Incremental move reverse";
      so.motion_type = INC_NEGATIVE;
    }
    else
    {
      res.message = "Incremental move no action";
      so.motion_type = NO_ACTION;
    }

    so.position = std::abs(req.increment);
    return res.success = true;
  }
  else
  {
    res.message = "Cannot command drive";
    return res.success = false;
  }
}

bool ACSI::moveRotary(acsi_eip_driver::acsi_moveRotary::Request& req,
                      acsi_eip_driver::acsi_moveRotary::Response& res)
{
  if (!ss.host_control && ss.enabled)
  {
    so.drive_command = START;
    if (req.increment > 0)
    {
      res.message = "Rotary move positive";
      so.motion_type = INC_POS_ROTARY;
    }
    else if (req.increment < 0)
    {
      res.message = "Rotary move negative";
      so.motion_type = INC_NEG_ROTARY;
    }
    else
    {
      res.message = "Rotary move no action";
      so.motion_type = NO_ACTION;
    }

    so.position = std::abs(req.increment);

    return res.success = true;
  }
  else
  {
    res.message = "Cannot command drive";
    return res.success = false;
  }
}

bool ACSI::moveSelect(acsi_eip_driver::acsi_moveSelect::Request& req,
                      acsi_eip_driver::acsi_moveSelect::Response& res)
{
  ROS_INFO_STREAM("Move select: " << req.select);
  if (!ss.host_control && ss.enabled && req.select > 0 && req.select <= 16)
  {
    so.drive_command = START;
    so.move_select = req.select;
    res.message = "Saved profile move";
    return res.success = true;
  }
  else
  {
    res.message = "Cannot command drive";
    return res.success = false;
  }

  return true;
}

// void ACSI::startUDPIO()
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

}  // namespace os32c
