cmake_minimum_required(VERSION 2.8.3)
project(stepper_eip_driver)

find_package(catkin REQUIRED COMPONENTS
    odva_ethernetip
    roscpp
    std_msgs
    message_generation
)

find_package(Boost 1.58 REQUIRED COMPONENTS system)

add_message_files(
  FILES
  stepper_inputs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS odva_ethernetip roscpp message_runtime
  LIBRARIES stepper
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(stepper src/stepper.cpp)
target_link_libraries(stepper
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(scanner_node src/scanner_node.cpp)
target_link_libraries(scanner_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(stepper_node src/stepper_node.cpp)
target_link_libraries(stepper_node
  stepper
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Mark executables and libraries for installation
install(TARGETS stepper stepper_node scanner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/stepper.launch)

endif()

