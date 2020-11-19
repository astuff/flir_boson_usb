cmake_minimum_required(VERSION 3.5)
project(flir_boson_usb)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(
  flir_boson_usb_node
  src/flir_boson_usb_node.cpp
  src/BosonCamera.cpp
)

target_include_directories(flir_boson_usb_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  flir_boson_usb_node 
  rclcpp 
  std_msgs
  camera_info_manager
  image_transport
  cv_bridge
  sensor_msgs
  OpenCV
)

install(TARGETS 
 flir_boson_usb_node
 DESTINATION lib/${PROJECT_NAME}
 )
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  example_calibrations
  DESTINATION share/${PROJECT_NAME}
)

ament_package()