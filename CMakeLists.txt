cmake_minimum_required(VERSION 2.8.3)
project(kingfisher_bringup)

find_package(catkin)

catkin_package()

add_subdirectory(src/vserial)
