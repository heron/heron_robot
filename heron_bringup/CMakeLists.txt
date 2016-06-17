cmake_minimum_required(VERSION 2.8.3)
project(heron_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/install scripts/setup
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
