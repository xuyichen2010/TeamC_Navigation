cmake_minimum_required(VERSION 2.8.3)
project(point_cloud)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(point_cloud
        src/point_cloud.cpp)
target_link_libraries(point_cloud
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )


