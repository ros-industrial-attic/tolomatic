/**
 * @file servo.h
 * @brief Tolomatic ACSI servo interface using Ethernet/IP - servo functions
 *class definition.
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

#ifndef ACSI_EIP_DRIVER_H
#define ACSI_EIP_DRIVER_H

#include <boost/shared_ptr.hpp>

#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"

#include "odva_ethernetip/session.h"
#include "odva_ethernetip/socket/socket.h"

#include "input_assembly.h"
#include "output_assembly.h"

#include <acsi_eip_driver/acsi_inputs.h>
#include <acsi_eip_driver/acsi_outputs.h>
#include <acsi_eip_driver/acsi_status.h>

#include <acsi_eip_driver/acsi_setProfile.h>
#include <acsi_eip_driver/acsi_moveAbsolute.h>
#include <acsi_eip_driver/acsi_moveIncremental.h>
#include <acsi_eip_driver/acsi_moveRotary.h>
#include <acsi_eip_driver/acsi_moveSelect.h>
#include <acsi_eip_driver/acsi_moveVelocity.h>

using boost::shared_ptr;
using eip::Session;
using eip::socket::Socket;

namespace acsi_eip_driver
{
const double IN_POSITION_TOLERANCE = 0.10;

/**
 * Main interface for the Tolomatic stepper controller.
 * Produces methods to access the stepper controller from a high level.
 */
class ACSI : public Session
{
public:
  /**
   * Construct a new instance.
   * @param socket Socket instance to use for communication with the stepper
   * controller
   */
  ACSI(shared_ptr<Socket> socket, shared_ptr<Socket> io_socket)
    : Session(socket, io_socket), connection_num_(-1), mrc_sequence_num_(1)
  {
  }

  // drive interface functions
  InputAssembly getDriveData();
  void setDriveData();
  void servoControlCallback(const acsi_outputs::ConstPtr& oa);
  void updateDriveStatus(InputAssembly ia);

  // ROS service callback handler functions
  bool enable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  bool estop(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  bool setHome(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool moveHome(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool moveStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool setProfile(acsi_eip_driver::acsi_setProfile::Request& req,
                  acsi_eip_driver::acsi_setProfile::Response& res);

  bool moveAbsolute(acsi_eip_driver::acsi_moveAbsolute::Request& req,
                    acsi_eip_driver::acsi_moveAbsolute::Response& res);

  bool moveIncremental(acsi_eip_driver::acsi_moveIncremental::Request& req,
                       acsi_eip_driver::acsi_moveIncremental::Response& res);

  bool moveRotary(acsi_eip_driver::acsi_moveRotary::Request& req,
                  acsi_eip_driver::acsi_moveRotary::Response& res);

  bool moveSelect(acsi_eip_driver::acsi_moveSelect::Request& req,
                  acsi_eip_driver::acsi_moveSelect::Response& res);

  bool moveVelocity(acsi_eip_driver::acsi_moveVelocity::Request& req,
                    acsi_eip_driver::acsi_moveVelocity::Response& res);


  // TODO: Debug this
  // void startUDPIO(); //for implicit data; not working

  acsi_inputs si;
  acsi_outputs so;
  acsi_status ss;
  acsi_status ss_last;

private:
  // data for sending to stepper controller
  int connection_num_;
  EIP_UDINT mrc_sequence_num_;
};

}  // namespace acsi_eip_driver

#endif  // ACSI_EIP_DRIVER_H
