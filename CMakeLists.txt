cmake_minimum_required(VERSION 2.8.3)
project(cubeeye_camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  message_generation
)

add_service_files(
  FILES
  "LastState.srv"
  "LastError.srv"
  "Scan.srv"
  "Connect.srv"
  "Run.srv"
  "Stop.srv"
  "Disconnect.srv"
)

generate_dynamic_reconfigure_options(
  cfg/scube_camera.cfg
  cfg/icube_camera.cfg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/include/CubeEye
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/liblive555/lib/Release
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/libopencv/lib
)

add_executable(${PROJECT_NAME}_node 
  src/CameraModule.cpp
  src/CubeEyeCameraNode.cpp
  src/ModelParameter.cpp
  src/ICubeModelParameter.cpp
  src/SCubeModelParameter.cpp
)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  CubeEye 
)

target_link_libraries(${PROJECT_NAME}_node "-Wl,--disable-new-dtags")
