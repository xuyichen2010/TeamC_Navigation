cmake_minimum_required(VERSION 2.8.3)
project(dji_m100_controllers_gazebo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  gazebo_ros
  gazebo_dev
  gazebo_ros_control
)

find_package(gazebo REQUIRED)
catkin_package(
)

include_directories(
	include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
	${GAZEBO_LIRARY_DIRS} ${catkin_LIBRARY_DIRS}
)

### Create HITL Library.
add_library(dji_m100_controller_gazebo_hitl SHARED src/dji_m100_ros_hitl_controller_gazebo.cpp)
target_link_libraries(dji_m100_controller_gazebo_hitl ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
install(TARGETS dji_m100_controller_gazebo_hitl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  dji_m100_controller_gazebo.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
