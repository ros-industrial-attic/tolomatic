/**
*/

#ifndef EIP_DRIVER_H
#define EIP_DRIVER_H

#include <boost/shared_ptr.hpp>

#include "odva_ethernetip/session.h"
#include "odva_ethernetip/socket/socket.h"

#include "input_assembly.h"
#include "output_assembly.h"

#include <acsi_eip_driver/acsi_inputs.h>
#include <acsi_eip_driver/acsi_outputs.h>
#include <acsi_eip_driver/acsi_status.h>

#include <acsi_eip_driver/acsi_enable.h>
#include <acsi_eip_driver/acsi_home.h>
#include <acsi_eip_driver/acsi_moveProfile.h>
#include <acsi_eip_driver/acsi_stop.h>

//not impliemnted
//#include <acsi_eip_driver/acsi_estop.h>
//#include <acsi_eip_driver/acsi_sethome.h>

using boost::shared_ptr;
using eip::Session;
using eip::socket::Socket;

namespace acsi_eip_driver {


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

  bool moveProfile(acsi_eip_driver::acsi_moveProfile::Request  &req,
                   acsi_eip_driver::acsi_moveProfile::Response &res);

  bool home(acsi_eip_driver::acsi_home::Request &req,
            acsi_eip_driver::acsi_home::Response &res);

  bool stop(acsi_eip_driver::acsi_stop::Request &req,
            acsi_eip_driver::acsi_stop::Response &res);

  //not implimented
  //bool estop(acsi_stepper_driver::stepper_estop::Request &req,
  //           acsi_stepper_driver::stepper_estop::Response &res);
  //
  //bool setHome(acsi_stepper_driver::stepper_sethome::Request &req,
  //             acsi_stepper_driver::stepper_sethome::Response &res);

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

} // namespace eip_driver

#endif  // EIP_DRIVER_H
