/**
Software License Agreement (BSD)

\file      scanner_node.cpp
\authors   Bill McCormick <wmccormick@swri.org>
\copyright
*/


#include <ros/ros.h>

#include "odva_ethernetip/io_scanner.h"

using std::cout;
using std::endl;
using eip::IOScanner;

int main(int argc, char const *argv[])
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
