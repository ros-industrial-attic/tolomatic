/**
*/

#ifndef ACSI_EIP_DRIVER_H
#define ACSI_EIP_DRIVER_H

#include <boost/shared_ptr.hpp>

#include "odva_ethernetip/session.h"
#include "odva_ethernetip/socket/socket.h"

#include "input_assembly.h"
#include "output_assembly.h"

#include <acsi_eip_driver/acsi_inputs.h>
#include <acsi_eip_driver/acsi_outputs.h>
#include <acsi_eip_driver/acsi_status.h>

#include <acsi_eip_driver/acsi_enable.h>
#include <acsi_eip_driver/acsi_moveHome.h>
#include <acsi_eip_driver/acsi_moveAbsolute.h>
#include <acsi_eip_driver/acsi_moveIncremental.h>
#include <acsi_eip_driver/acsi_setProfile.h>
#include <acsi_eip_driver/acsi_moveRotary.h>
#include <acsi_eip_driver/acsi_moveSelect.h>
#include <acsi_eip_driver/acsi_moveVelocity.h>
#include <acsi_eip_driver/acsi_moveStop.h>
#include <acsi_eip_driver/acsi_estop.h>
#include <acsi_eip_driver/acsi_setHome.h>


using boost::shared_ptr;
using eip::Session;
using eip::socket::Socket;

namespace acsi_eip_driver {

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
   * @param socket Socket instance to use for communication with the stepper controller
   */
  ACSI(shared_ptr<Socket> socket, shared_ptr<Socket> io_socket)
    : Session(socket, io_socket), connection_num_(-1), mrc_sequence_num_(1)
  {
  }

  InputAssembly getDriveData();
  void setDriveData();
  void servoControlCallback(const acsi_outputs::ConstPtr& oa);

  void updateDriveStatus(InputAssembly ia);

  bool enable(acsi_eip_driver::acsi_enable::Request  &req,
              acsi_eip_driver::acsi_enable::Response &res);

  bool setHome(acsi_eip_driver::acsi_setHome::Request &req,
               acsi_eip_driver::acsi_setHome::Response &res);

  bool setProfile(acsi_eip_driver::acsi_setProfile::Request  &req,
                   acsi_eip_driver::acsi_setProfile::Response &res);

  bool moveStop(acsi_eip_driver::acsi_moveStop::Request &req,
            acsi_eip_driver::acsi_moveStop::Response &res);

  bool moveAbsolute(acsi_eip_driver::acsi_moveAbsolute::Request  &req,
                   acsi_eip_driver::acsi_moveAbsolute::Response &res);

  bool moveIncremental(acsi_eip_driver::acsi_moveIncremental::Request  &req,
                   acsi_eip_driver::acsi_moveIncremental::Response &res);

  bool moveRotary(acsi_eip_driver::acsi_moveRotary::Request  &req,
                   acsi_eip_driver::acsi_moveRotary::Response &res);

  bool moveSelect(acsi_eip_driver::acsi_moveSelect::Request  &req,
                   acsi_eip_driver::acsi_moveSelect::Response &res);

  bool moveVelocity(acsi_eip_driver::acsi_moveVelocity::Request  &req,
                   acsi_eip_driver::acsi_moveVelocity::Response &res);

  bool moveHome(acsi_eip_driver::acsi_moveHome::Request &req,
            acsi_eip_driver::acsi_moveHome::Response &res);

  bool estop(acsi_eip_driver::acsi_estop::Request &req,
             acsi_eip_driver::acsi_estop::Response &res);

  //TODO: Debug this
  //void startUDPIO(); //for implicit data; not working

  acsi_inputs si;
  acsi_outputs so;
  acsi_status ss;
  acsi_status ss_last;

private:
  // data for sending to stepper controller

  int connection_num_;
  EIP_UDINT mrc_sequence_num_;

};

} // namespace acsi_eip_driver

#endif  // ACSI_EIP_DRIVER_H
