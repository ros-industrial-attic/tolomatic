/**
Software License Agreement

\file      input_assembly.h
\authors   Bill McCormick <wmccormick@swri.org>
\copyright

*/

#ifndef INPUT_ASSEMBLY_H
#define INPUT_ASSEMBLY_H

#include "odva_ethernetip/eip_types.h"
#include "odva_ethernetip/serialization/reader.h"
#include "odva_ethernetip/serialization/writer.h"
#include "odva_ethernetip/serialization/serializable.h"

using eip::serialization::Serializable;
using eip::serialization::Reader;
using eip::serialization::Writer;

namespace stepper_eip_driver {

typedef enum
{
  ENABLED = 0x00000001,
  HOMED   = 0x00000002,
  MOTION  = 0x00000004,
  SSTOP   = 0x00000008,
  BRAKE_OFF = 0x10000000,
  HOST_CTRL = 0x80000000
} STEPPER_STATUS;

/**
 * Data structure and operators.
 */
class InputAssembly : public Serializable
{
public:
  float current_position;
  EIP_DWORD drive_status;
  EIP_DWORD drive_faults;
  EIP_DWORD digital_input;
  EIP_DWORD digital_output;
  float analog_input;
  float analog_output;

  /**
   * Size of this message including all data
   */
  virtual size_t getLength() const
  {
    return 28;
  }

  /**
   * Serialize data into the given buffer
   * @param writer Writer to use for serialization
   * @return the writer again
   * @throw std::length_error if the buffer is too small for the header data
   */
  virtual Writer& serialize(Writer& writer) const
  {
    writer.write(current_position);
    writer.write(drive_status);
    writer.write(drive_faults);
    writer.write(digital_input);
    writer.write(digital_output);
    writer.write(analog_input);
    writer.write(analog_output);
    return writer;
  }

  /**
   * Extra length information is not relevant in this context. Same as deserialize(reader)
   */
  virtual Reader& deserialize(Reader& reader, size_t length)
  {
    deserialize(reader);
    return reader;
  }

  /**
   * Deserialize data from the given reader without length information
   * @param reader Reader to use for deserialization
   * @return the reader again
   * @throw std::length_error if the buffer is overrun while deserializing
   */
  virtual Reader& deserialize(Reader& reader)
  {
    reader.read(current_position);
    reader.read(drive_status);
    reader.read(drive_faults);
    reader.read(digital_input);
    reader.read(digital_output);
    reader.read(analog_input);
    reader.read(analog_output);
    return reader;
  }
};

} // namespace omron_os32c_driver

#endif  // INPUT_ASSEMBLY_H
