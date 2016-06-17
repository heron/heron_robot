cmake_minimum_required(VERSION 2.8.3)
project(heron_bringup)

find_package(catkin REQUIRED)

catkin_package()
catkin_python_setup()

install(DIRECTORY launch config udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/install scripts/setup scripts/navsat_rtk_relay
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
