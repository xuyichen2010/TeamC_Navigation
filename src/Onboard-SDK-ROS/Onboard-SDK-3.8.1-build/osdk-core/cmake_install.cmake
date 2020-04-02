# Install script for directory: /home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/djiosdk/DJIOSDKConfig.cmake;/usr/local/lib/cmake/djiosdk/DJIOSDKConfigVersion.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib/cmake/djiosdk" TYPE FILE FILES
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/cmake-modules/DJIOSDKConfig.cmake"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/cmake-modules/DJIOSDKConfigVersion.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "shlib")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libadvanced-sensing.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE FILE FILES "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/advanced-sensing-2.0.3/lib/libadvanced-sensing.a")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "shlib")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libdjiosdk-core.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE STATIC_LIBRARY FILES "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/libs/libdjiosdk-core.a")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/djiosdk/dji_mobile_communication.hpp;/usr/local/include/djiosdk/dji_payload_device.hpp;/usr/local/include/djiosdk/dji_gimbal.hpp;/usr/local/include/djiosdk/dji_mission_type.hpp;/usr/local/include/djiosdk/dji_status.hpp;/usr/local/include/djiosdk/dji_version.hpp;/usr/local/include/djiosdk/dji_error.hpp;/usr/local/include/djiosdk/dji_waypoint.hpp;/usr/local/include/djiosdk/dji_hardware_sync.hpp;/usr/local/include/djiosdk/dji_mission_manager.hpp;/usr/local/include/djiosdk/dji_mfio.hpp;/usr/local/include/djiosdk/dji_vehicle.hpp;/usr/local/include/djiosdk/dji_camera.hpp;/usr/local/include/djiosdk/dji_telemetry.hpp;/usr/local/include/djiosdk/dji_hotpoint.hpp;/usr/local/include/djiosdk/dji_ack.hpp;/usr/local/include/djiosdk/dji_virtual_rc.hpp;/usr/local/include/djiosdk/dji_telemetry_doc.hpp;/usr/local/include/djiosdk/dji_broadcast.hpp;/usr/local/include/djiosdk/dji_control.hpp;/usr/local/include/djiosdk/dji_type.hpp;/usr/local/include/djiosdk/dji_subscription.hpp;/usr/local/include/djiosdk/dji_command.hpp;/usr/local/include/djiosdk/dji_mobile_device.hpp;/usr/local/include/djiosdk/dji_mission_base.hpp;/usr/local/include/djiosdk/dji_vehicle_callback.hpp;/usr/local/include/djiosdk/dji_open_protocol.hpp;/usr/local/include/djiosdk/dji_aes.hpp;/usr/local/include/djiosdk/dji_protocol_base.hpp;/usr/local/include/djiosdk/dji_crc.hpp;/usr/local/include/djiosdk/dji_hard_driver.hpp;/usr/local/include/djiosdk/dji_memory.hpp;/usr/local/include/djiosdk/dji_log.hpp;/usr/local/include/djiosdk/dji_platform_manager.hpp;/usr/local/include/djiosdk/dji_thread_manager.hpp;/usr/local/include/djiosdk/dji_circular_buffer.hpp;/usr/local/include/djiosdk/dji_singleton.hpp;/usr/local/include/djiosdk/dji_macros.hpp;/usr/local/include/djiosdk/posix_thread.hpp;/usr/local/include/djiosdk/posix_thread_manager.hpp;/usr/local/include/djiosdk/linux_serial_device.hpp;/usr/local/include/djiosdk/dji_advanced_sensing.hpp;/usr/local/include/djiosdk/dji_advanced_sensing_protocol.hpp;/usr/local/include/djiosdk/linux_usb_device.hpp;/usr/local/include/djiosdk/dji_camera_image.hpp;/usr/local/include/djiosdk/dji_camera_stream.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/djiosdk" TYPE FILE FILES
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_mobile_communication.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_payload_device.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_gimbal.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_mission_type.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_status.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_version.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_error.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_waypoint.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_hardware_sync.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_mission_manager.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_mfio.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_vehicle.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_camera.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_telemetry.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_hotpoint.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_ack.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_virtual_rc.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_telemetry_doc.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_broadcast.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_control.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_type.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_subscription.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_command.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_mobile_device.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_mission_base.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/api/inc/dji_vehicle_callback.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/protocol/inc/dji_open_protocol.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/protocol/inc/dji_aes.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/protocol/inc/dji_protocol_base.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/protocol/inc/dji_crc.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/hal/inc/dji_hard_driver.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/hal/inc/dji_memory.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/hal/inc/dji_log.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/hal/inc/dji_platform_manager.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/hal/inc/dji_thread_manager.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/utility/inc/dji_circular_buffer.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/utility/inc/dji_singleton.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/utility/inc/dji_macros.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/platform/linux/inc/posix_thread.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/platform/linux/inc/posix_thread_manager.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/platform/linux/inc/linux_serial_device.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/advanced-sensing-2.0.3/inc/dji_advanced_sensing.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/advanced-sensing-2.0.3/inc/dji_advanced_sensing_protocol.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/advanced-sensing-2.0.3/inc/linux_usb_device.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/advanced-sensing-2.0.3/inc/dji_camera_image.hpp"
    "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1/osdk-core/advanced-sensing-2.0.3/inc/dji_camera_stream.hpp"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/cmake/djiosdk/djiosdkTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}/usr/local/lib/cmake/djiosdk/djiosdkTargets.cmake"
         "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/CMakeFiles/Export/_usr/local/lib/cmake/djiosdk/djiosdkTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}/usr/local/lib/cmake/djiosdk/djiosdkTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}/usr/local/lib/cmake/djiosdk/djiosdkTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/djiosdk/djiosdkTargets.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib/cmake/djiosdk" TYPE FILE FILES "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/CMakeFiles/Export/_usr/local/lib/cmake/djiosdk/djiosdkTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/lib/cmake/djiosdk/djiosdkTargets-debug.cmake")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/lib/cmake/djiosdk" TYPE FILE FILES "/home/aaronguan/Matrice100_ws/src/Onboard-SDK-ROS/Onboard-SDK-3.8.1-build/osdk-core/CMakeFiles/Export/_usr/local/lib/cmake/djiosdk/djiosdkTargets-debug.cmake")
  endif()
endif()

