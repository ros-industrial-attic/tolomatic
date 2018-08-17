/**

\file      stepper.cpp
\authors   Bill McCormick <wmccormick@swri.org>
\copyright
*/


#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/asio.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include "stepper.h"

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

namespace stepper_eip_driver {

InputAssembly STEPPER::getDriveData()
{
  InputAssembly ia;
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

void STEPPER::updateDriveStatus(InputAssembly ia)
{
  ss.enabled      = (ia.drive_status & (STEPPER_STATUS)ENABLE) > 0;
  ss.homed        = (ia.drive_status & (STEPPER_STATUS)HOMED) > 0;
  ss.brake_off    = (ia.drive_status & (STEPPER_STATUS)BRAKE_OFF) > 0;
  ss.host_control = (ia.drive_status & (STEPPER_STATUS)HOST_CTRL) > 0;
  ss.moving       = (ia.drive_status & (STEPPER_STATUS)MOTION) > 0;
  ss.stopped      = (ia.drive_status & (STEPPER_STATUS)SSTOP) > 0;
  ss.current_position = ia.current_position;

  //if we just started moving and all is well
  if(ss.enabled) {
    if(!ss.stopped && ss.moving && !ss_last.moving) {
      //revert the drive command to a simple enable
      ss.target_position = si.analog_input;
      ss.in_position = false;
      if(!ss.host_control) so.drive_command = (STEPPER_DRIVE_COMMAND)ENABLE;
    } else if (!ss.moving && ss_last.moving) {
      if(abs(si.current_position - si.analog_output) <= 0.1) {
        ss.in_position = true;
      }
    }
  }

  memcpy(&ss_last,&ss, sizeof(ss));
}


void STEPPER::setDriveData()
{
  OutputAssembly oa;

  oa.drive_command = so.drive_command;
  oa.move_select = so.move_select;

  shared_ptr<OutputAssembly> sb = make_shared<OutputAssembly>(oa);

  setSingleAttributeSerializable(0x04, 0x70, 3, sb);
}

bool STEPPER::enable(stepper_eip_driver::stepper_enable::Request  &req,
                     stepper_eip_driver::stepper_enable::Response &res)
{

  if(!ss.host_control) {
    so.drive_command = (req.enable) ? (STEPPER_DRIVE_COMMAND)ENABLE : (STEPPER_DRIVE_COMMAND)DISABLE;
    return res.success = true;
  } else {
    return res.success = false;
  }
}

bool STEPPER::moveProfile(stepper_eip_driver::stepper_moveProfile::Request  &req,
                          stepper_eip_driver::stepper_moveProfile::Response &res)
{

  if(!ss.host_control && req.profile > 0 && req.profile < 16) {
    so.drive_command = (STEPPER_DRIVE_COMMAND)START;
    so.move_select = req.profile;
    return res.success = true;
  } else {
    return res.success = false;
  }

  return true;
}

bool STEPPER::stop(stepper_eip_driver::stepper_stop::Request  &req,
                   stepper_eip_driver::stepper_stop::Response &res)
{

  if(!ss.host_control) {
    so.drive_command = (req.stop) ? (STEPPER_DRIVE_COMMAND)STOP : so.drive_command;
    return res.success = true;
  } else {
    return res.success = false;
  }
}

bool STEPPER::estop(stepper_eip_driver::stepper_estop::Request  &req,
                    stepper_eip_driver::stepper_estop::Response &res)
{

  if(!ss.host_control) {
    so.drive_command = (req.e_stop) ? (STEPPER_DRIVE_COMMAND)ESTOP : so.drive_command;
    return res.success = true;
  } else {
    return res.success = false;
  }

}

bool STEPPER::home(stepper_eip_driver::stepper_home::Request  &req,
                   stepper_eip_driver::stepper_home::Response &res)
{

  if(!ss.host_control) {
    so.drive_command = (req.home) ? (STEPPER_DRIVE_COMMAND)GOHOME: so.drive_command;
    return res.success = true;
  } else {
    return res.success = false;
  }
}


bool STEPPER::setHome(stepper_eip_driver::stepper_sethome::Request  &req,
                      stepper_eip_driver::stepper_sethome::Response &res)
{

  if(!ss.host_control) {
    so.drive_command = (req.sethome) ? (STEPPER_DRIVE_COMMAND)HOME_HERE: so.drive_command;
    return res.success = true;
  } else {
    return res.success = false;
  }

}


void STEPPER::startUDPIO()
{
  EIP_CONNECTION_INFO_T o_to_t, t_to_o;
  o_to_t.assembly_id = 0x70;
  o_to_t.buffer_size = 0x0004;
  o_to_t.rpi = 0x000186A0;
  t_to_o.assembly_id = 0x64;
  t_to_o.buffer_size = 0x001c;
  t_to_o.rpi = 0x0000C350;

  connection_num_ = createConnection(o_to_t, t_to_o);
}

} // namespace os32c
