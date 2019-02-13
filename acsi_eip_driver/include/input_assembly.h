/**
 * @file input_assembly.h
 * @brief Tolomatic ACSI servo interface using Ethernet/IP - EIP input assembly definition.
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

#ifndef INPUT_ASSEMBLY_H
#define INPUT_ASSEMBLY_H

#include "odva_ethernetip/eip_types.h"
#include "odva_ethernetip/serialization/reader.h"
#include "odva_ethernetip/serialization/writer.h"
#include "odva_ethernetip/serialization/serializable.h"

using eip::serialization::Serializable;
using eip::serialization::Reader;
using eip::serialization::Writer;

namespace acsi_eip_driver {

//31 30 29 28 | 27 26 25 24 | 23 22 21 20 | 19 18 17 16 | 15 14 13 12 | 11 10 9 8 | 7 6 5 4 | 3 2 1 0 |
typedef enum
{
  ENABLED = 0x00000001,
  HOMED   = 0x00000002,
  MOTION  = 0x00000004,
  SSTOP   = 0x00000008,
  BRAKE_OFF = 0x00100000,
  IN_POSITION = 0x04000000,
  HOST_CTRL = 0x80000000
} ACSI_STATUS;

typedef enum
{
  POS_LIMIT = 0x00000001,
  NEG_LIMIT = 0x00000002,
  SOFT_STOP = 0x00000004,
  POS_ERROR = 0x00000008,
  FB_ERROR  = 0x00000010,
  OVER_CURRENT = 0x00000020,
  MOTOR_TEMP = 0x00000040,
  DRIVE_TEMP = 0x00000080,
  DRIVE_OVER_VOLTAGE = 0x00000100,
  DRIVE_UNDER_VOLTAGE = 0x00000200,
  FLASH_ERROR = 0x00000400,
  I2T_LIMIT = 0x00000800,
  SHORT_CIRCUIT = 0x00001000,
  WD_RESET = 0x00002000,
  VELOCITY_ERROR = 0x00004000
} ACSI_FAULT;

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
  virtual Reader& deserialize(Reader& reader, __attribute__((unused)) size_t length)
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
