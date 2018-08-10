/**

\file      stepper.cpp
\authors   Bill McCormick <wmccormick@swri.org>
\copyright
*/


#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/asio.hpp>

#include "stepper.h"
#include "input_assembly.h"
#include "output_assembly.h"
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
  return ia;
}

void STEPPER::setDriveData(OutputAssembly oa)
{
  shared_ptr<Serializable> sb = make_shared<Serializable>(oa);

  setSingleAttributeSerializable(0x04, 0x70, 3, sb);
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
