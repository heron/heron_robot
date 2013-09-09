cmake_minimum_required(VERSION 2.8.3)
project(kingfisher_bringup)

find_package(catkin)

catkin_package()

add_subdirectory(src/vserial)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY udev/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/udev)

install(PROGRAMS scripts/bridge scripts/install scripts/netserial
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
