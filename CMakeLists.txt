cmake_minimum_required(VERSION 3.10)

project(webrtc_stream)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 设置默认构建类型为 Debug
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0 gstreamer-app-1.0)
find_package(Boost REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_search_module(RK_MPP REQUIRED rockchip_mpp)
set(RGA_INCLUDE_DIR "/usr/include")
set(RGA_LIBRARY "/usr/lib/aarch64-linux-gnu/librga.so")

set(YUV_INCLUDE_DIR "/usr/include")
set(YUV_LIBRARY "/usr/lib/aarch64-linux-gnu/libyuv.so")

include_directories(
  include
  src/interprocess
  ${GST_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${RK_MPP_INCLUDE_DIRS}
  ${YUV_INCLUDE_DIR}
  ${RGA_INCLUDE_DIR}
)

add_executable(producer
  src/interprocess/producer.cpp
  # webrtc_stream.cpp
)

ament_target_dependencies(producer
  rclcpp
  sensor_msgs
)

target_link_libraries(producer
  ${GST_LIBRARIES}
  ${Boost_LIBRARIES}
  ${RK_MPP_LIBRARIES}
  ${YUV_LIBRARY}
  ${RGA_LIBRARY}
  jsoncpp
)

install(TARGETS
  producer
  DESTINATION lib/${PROJECT_NAME})
############################################################################################################

add_executable(consumer
  src/interprocess/consumer.cpp
  # webrtc_stream.cpp
)

ament_target_dependencies(consumer
  rclcpp
  sensor_msgs
)

target_link_libraries(consumer
  ${GST_LIBRARIES}
  ${Boost_LIBRARIES}
  ${RK_MPP_LIBRARIES}
  ${YUV_LIBRARY}
  ${RGA_LIBRARY}
  jsoncpp
)

install(TARGETS
consumer
  DESTINATION lib/${PROJECT_NAME})


############################################################################################################

add_executable(server
  src/server.cpp
)

target_link_libraries(server
  ${Boost_LIBRARIES}
  jsoncpp
)

install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME})

add_executable(function_test
  test/single_thread_ros_publish.cpp
)

ament_target_dependencies(function_test
  rclcpp
  sensor_msgs
)

target_link_libraries(function_test
  ${RK_MPP_LIBRARIES}
  ${YUV_LIBRARY}
  ${RGA_LIBRARY}
)

install(TARGETS
  function_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()


