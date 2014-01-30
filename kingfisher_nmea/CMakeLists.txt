cmake_minimum_required(VERSION 2.8.3)
project(kingfisher_nmea)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

install(PROGRAMS
  scripts/install
  scripts/simple_tester
  nodes/battery_publisher
  nodes/command_publisher
  nodes/gps_time_offset_publisher
  nodes/imu_publisher
  nodes/nav_publisher
  nodes/tt_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
