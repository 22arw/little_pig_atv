cmake_minimum_required(VERSION 2.8.3)
project(little_pig_control)

find_package(catkin REQUIRED roscpp rospy std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

catkin_package()

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(sender src/sender-example.cpp)
target_link_libraries(sender ${catkin_LIBRARIES})
add_dependencies(sender little_pig_control_generate_messages_cpp)
