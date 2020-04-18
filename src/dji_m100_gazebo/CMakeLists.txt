cmake_minimum_required(VERSION 2.8.3)
project(dji_m100_gazebo)
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
  	gazebo_ros
  	rviz_visual_tools
	dji_sdk
	tf
)

find_package(DJIOSDK REQUIRED)
find_package(gazebo REQUIRED)
catkin_package(
)

include_directories(
	include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
	${GAZEBO_LIRARY_DIRS} ${catkin_LIBRARY_DIRS}
)

link_libraries(
  ${catkin_LIBRARIES}
  yaml-cpp)


add_executable(rviz_collision_publisher src/rviz_collision_publisher.cpp)
target_link_libraries(rviz_collision_publisher ${catkin_LIBRARIES})
install(TARGETS rviz_collision_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(dji_m100_pcsim_gazebo_bridge src/dji_m100_pcsim_gazebo_bridge.cpp)
target_link_libraries(dji_m100_pcsim_gazebo_bridge
	${catkin_LIBRARIES}
	${GAZEBO_LIBRARIES}
	#${DJIOSDK_LIBRARIES}
)
add_dependencies(dji_m100_pcsim_gazebo_bridge dji_sdk_generate_messages_cpp)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
install(DIRECTORY rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
