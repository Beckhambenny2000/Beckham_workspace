cmake_minimum_required(VERSION 3.8)

project(gv_ros2)

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

message("cmake current source dir : ${CMAKE_CURRENT_SOURCE_DIR}")
find_package(OpenCV REQUIRED)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(libusb REQUIRED libusb-1.0)

# find_package(libuvc REQUIRED)
#find_package(PCL REQUIRED)
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
  )
#link_directories(${PCL_LIBRARY_DIRS})

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  message(STATUS "cmake system is x86_64")
  set(LIBUVC_SO_PATH "/usr/lib/x86_64-linux-gnu/libuvc.so")
  set(LIBUSB_SO_PATH "/usr/lib/x86_64-linux-gnu/libusb-1.0.so")
  set(LIBBOOST_FILESYSTEM_SO_PATH "/usr/lib/x86_64-linux-gnu/libboost_filesystem.so")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  message(STATUS "camke system is arm_64")
  set(LIBUVC_SO_PATH "/usr/lib/aarch64-linux-gnu/libuvc.so")
  set(LIBUSB_SO_PATH "/usr/lib/aarch64-linux-gnu/libusb-1.0.so")
  set(LIBBOOST_FILESYSTEM_SO_PATH "/usr/lib/aarch64-linux-gnu/libboost_filesystem.so")
else()
  message(STATUS "cmake system is not support")
endif()

set(dependencies
  cv_bridge
  image_transport
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
)

add_executable(
  ${PROJECT_NAME}_node 
  src/gv_d100/stereoDepth.cpp
  src/gv_d100/sendDataByUSB.cpp
  src/gv_d100/usb.cpp
  src/gv_d100/depth2color.cpp
  src/gv_camera.cpp
  src/gv_camera_node.cpp
  )
# uvc usb-1.0 
target_link_libraries(
  ${PROJECT_NAME}_node
  #${PROJECT_NAME}
  ${OpenCV_LIBRARIES} 
  ${LIBUVC_SO_PATH}
  ${LIBUSB_SO_PATH}
  ${LIBBOOST_FILESYSTEM_SO_PATH}
  yaml-cpp 
  #${CMAKE_CURRENT_SOURCE_DIR}/lib/libgv_stereo_depth.so
)


#target_link_libraries(${PROJECT_NAME}_node  ${PROJECT_NAME})

ament_target_dependencies(
  ${PROJECT_NAME}_node
  ${dependencies})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION 
  lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
