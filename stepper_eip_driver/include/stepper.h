/**
*/

#ifndef EIP_DRIVER_H
#define EIP_DRIVER_H

#include <boost/shared_ptr.hpp>

#include "odva_ethernetip/session.h"
#include "odva_ethernetip/socket/socket.h"

#include "input_assembly.h"
#include "output_assembly.h"

#include <stepper_eip_driver/stepper_inputs.h>
#include <stepper_eip_driver/stepper_outputs.h>
#include <stepper_eip_driver/stepper_status.h>

#include <stepper_eip_driver/stepper_enable.h>
#include <stepper_eip_driver/stepper_estop.h>
#include <stepper_eip_driver/stepper_home.h>
#include <stepper_eip_driver/stepper_moveProfile.h>
#include <stepper_eip_driver/stepper_sethome.h>
#include <stepper_eip_driver/stepper_stop.h>

using boost::shared_ptr;
using eip::Session;
using eip::socket::Socket;

#define DEG2RAD(a) (a * M_PI / 180)
#define RAD2DEG(a) (a * 180 / M_PI)

namespace stepper_eip_driver {


/**
 * Main interface for the Tolomatic stepper controller. 
 * Produces methods to access the stepper controller from a high level.
 */
class STEPPER : public Session
{
public:
  /**
   * Construct a new instance.
   * @param socket Socket instance to use for communication with the stepper controller
   */
  STEPPER(shared_ptr<Socket> socket, shared_ptr<Socket> io_socket)
    : Session(socket, io_socket), connection_num_(-1), mrc_sequence_num_(1)
  {
  }

  InputAssembly getDriveData();
  void setDriveData();
  void stepperControlCallback(const stepper_outputs::ConstPtr& oa);

  void updateDriveStatus(InputAssembly ia);

  bool enable(stepper_eip_driver::stepper_enable::Request  &req,
              stepper_eip_driver::stepper_enable::Response &res);

  bool moveProfile(stepper_eip_driver::stepper_moveProfile::Request  &req,
                   stepper_eip_driver::stepper_moveProfile::Response &res);

  bool stop(stepper_eip_driver::stepper_stop::Request &req,
            stepper_eip_driver::stepper_stop::Response &res);

  bool estop(stepper_eip_driver::stepper_estop::Request &req,
             stepper_eip_driver::stepper_estop::Response &res);

  bool home(stepper_eip_driver::stepper_home::Request &req,
            stepper_eip_driver::stepper_home::Response &res);

  bool setHome(stepper_eip_driver::stepper_sethome::Request &req,
               stepper_eip_driver::stepper_sethome::Response &res);

  //TODO: Debug this
  void startUDPIO(); //for implicit data; not working

  stepper_inputs si;
  stepper_outputs so;
  stepper_status ss;
  stepper_status ss_last;

private:
  // data for sending to stepper controller

  int connection_num_;
  EIP_UDINT mrc_sequence_num_;

};

} // namespace eip_driver

#endif  // EIP_DRIVER_H
