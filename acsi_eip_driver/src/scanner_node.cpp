/**
 * @file scanner_node.cpp
 * @brief Tolomatic ACSI servo interface using Ethernet/IP - scanner
 *interigation tool.
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

#include <ros/ros.h>

#include "odva_ethernetip/io_scanner.h"

using std::cout;
using std::endl;
using eip::IOScanner;

int main(int argc, char const* argv[])
{
  if (argc != 2)
  {
    cout << "Usage: scanner_node [hostname]" << endl;
    return 1;
  }
  boost::asio::io_service io_service;
  try
  {
    IOScanner scanner(io_service, argv[1]);
    scanner.run();
  }
  catch (std::runtime_error ex)
  {
    cout << "Exception caught doing IO scan: " << ex.what() << endl;
  }
  return 0;
}
