cmake_minimum_required(VERSION 2.8.3)
project(little_pig_control)

find_package(catkin REQUIRED roscpp rospy std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

catkin_package()

catkin_install_python(PROGRAMS src/little_ackerman_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
