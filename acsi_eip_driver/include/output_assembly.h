/**
Software License Agreement

\file      input_assembly.h
\authors   Bill McCormick <wmccormick@swri.org>
\copyright

*/

#ifndef OUTPUT_ASSEMBLY_H
#define OUTPUT_ASSEMBLY_H

#include "odva_ethernetip/eip_types.h"
#include "odva_ethernetip/serialization/reader.h"
#include "odva_ethernetip/serialization/writer.h"
#include "odva_ethernetip/serialization/serializable.h"

using eip::serialization::Serializable;
using eip::serialization::Reader;
using eip::serialization::Writer;

namespace acsi_eip_driver {

typedef enum
{
  DISABLE   = 0,
  ENABLE    = 1,
  START     = 3,
  GOHOME    = 5,
  ESTOP     = 8,
  STOP      = 17,
  HOME_HERE = 21
} STEPPER_DRIVE_COMMAND;

typedef enum
{
  HOME        = 0,
  LEFT,BOTTOM = 1,
  CENTER      = 2,
  RIGHT,TOP   = 3
} MOVE_PROFILE;


/**
 * Data structure and operators.
 */
class OutputAssembly : public Serializable
{
public:
  EIP_USINT drive_command;
  EIP_USINT move_select;
  /**
   * Size of this message including all data
   */
  virtual size_t getLength() const
  {
    return 4;
  }

  /**
   * Serialize data into the given buffer
   * @param writer Writer to use for serialization
   * @return the writer again
   * @throw std::length_error if the buffer is too small for the header data
   */
  virtual Writer& serialize(Writer& writer) const
  {
    EIP_UINT reserved = 0;
    writer.write(drive_command);
    writer.write(move_select);
    writer.write(reserved);
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
    reader.read(drive_command);
    reader.read(move_select);
    reader.skip(sizeof(EIP_UINT));
    return reader;
  }
};

} // namespace omron_os32c_driver

#endif  // OUTPUT_ASSEMBLY_H
