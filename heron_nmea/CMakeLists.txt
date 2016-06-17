cmake_minimum_required(VERSION 2.8.3)
project(heron_nmea)

find_package(catkin REQUIRED COMPONENTS rospy roscpp nmea_msgs geometry_msgs heron_msgs)

find_package(Boost REQUIRED)

catkin_python_setup()
catkin_package()

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_command_publisher src/command_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_command_publisher ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME}_command_publisher heron_msgs_gencpp)
set_target_properties(${PROJECT_NAME}_command_publisher PROPERTIES OUTPUT_NAME command_publisher PREFIX "")

install(TARGETS ${PROJECT_NAME}_command_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/install
  scripts/simple_tester
  nodes/battery_publisher
  nodes/command_publisher_old
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
