cmake_minimum_required(VERSION 3.0.2)
project(franka_button_watch)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  franka_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/button_watch_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

