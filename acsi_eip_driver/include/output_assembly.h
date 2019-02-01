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
} ACSI_DRIVE_COMMAND;

typedef enum
{
    ABSOLUTE        = 0,    //Moves to a location at profile defined for Target 0
    INC_POSITIVE    = 1,    //Moves in the positive direction the distance specified by Target 0 Position at the defined motion profile
    INC_NEGATIVE    = 2,    //Moves in the negative direction the distance specified by Target 0 Position at the defined motion profile
    HOME            = 5,    //Executes a home move using the defined motion profile
    NO_ACTION       = 6,    //Does not execute motion
    FORCE           = 9,    //Press to Force (% current) (see manual for complete description)
    INC_POS_ROTARY  = 11,   //Moves in the positive direction the distance specified by Target 0 Position at the defined motion profile.
                            //If position is commanded past the maximum distance, current postion is reset
    INC_NEG_ROTARY  = 12,   //Moves in the negative direction the distance specified by Target 0 Position at the defined motion profile.
                            //If position is commanded past the maximum distance, current postion is reset
    VELOCITY_FWD    = 13,   //Starts a velocity move in positive direction at the profile velocity and acceleration. When complete, postion is cleared and un-homed.
    VELOCITY_REV    = 14    //Starts a velocity move in negative direction at the profile velocity and acceleration. When complete, postion is cleared and un-homed.
} ACSI_MOTION_TYPE;

/**
 * Data structure and operators.
 */

class OutputAssembly : public Serializable
{
public:
  EIP_USINT drive_command;
  EIP_USINT move_select;
  EIP_UINT reserved;
  float postion;
  float velocity;
  float accel;
  float decel;
  float force;
  EIP_UDINT motion_type;
  EIP_UDINT outputs;

  /**
   * Size of this message including all data
   */
  virtual size_t getLength() const
  {
    return 32;
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
    writer.write(postion);
    writer.write(velocity);
    writer.write(accel);
    writer.write(decel);
    writer.write(force);
    writer.write(motion_type);
    writer.write(outputs);
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
    reader.read(postion);
    reader.read(velocity);
    reader.read(accel);
    reader.read(decel);
    reader.read(force);
    reader.read(motion_type);
    reader.read(outputs);
    return reader;
  }
};

} // namespace acsi_eip_driver

#endif  // OUTPUT_ASSEMBLY_H
