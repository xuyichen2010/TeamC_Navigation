cmake_minimum_required(VERSION 2.8.3)
project(base_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  dji_sdk
  image_transport
  move_base
  tf
  sensor_msgs
)

find_package(DJIOSDK REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_drone
#  CATKIN_DEPENDS guidance move_base
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(base_controller
        src/base_controller.cpp)
target_link_libraries(base_controller
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(base_controller dji_sdk_generate_messages_cpp)



install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )


