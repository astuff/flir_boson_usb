cmake_minimum_required(VERSION 2.8.3)
project(flir_boson_usb)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS cv_bridge
)

add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/flir_boson_usb_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
