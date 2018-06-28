cmake_minimum_required(VERSION 2.8.3)
project(heron_base)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
   scripts/snmp_lights.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_add_env_hooks(50.heron_find_controller_config
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_add_env_hooks(50.heron_find_mag_config
    SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/base.launch)
endif()
