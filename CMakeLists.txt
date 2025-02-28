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
# set(RGA_INCLUDE_DIR "/usr/include")
# set(RGA_LIBRARY "/usr/lib/aarch64-linux-gnu/librga.so")

set(YUV_INCLUDE_DIR "/usr/include")
set(YUV_LIBRARY "/usr/lib/aarch64-linux-gnu/libyuv.so")



include_directories(
  include
  ${GST_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${RK_MPP_INCLUDE_DIRS}
  ${YUV_INCLUDE_DIR}
  # ${RGA_INCLUDE_DIR}
)


add_executable(main
  # test/available_devices.cpp
  # test/save_to_shm.cpp
  # test/multi_thread_shm.cpp
  # test/ring_buffer_shm.cpp
  # test/zero_copy_shm.cpp
  # test/data_proccess_shm.cpp
  # test/webrtc_appsrc_shm.cpp
  # test/gstreamer_appsrc.cpp
  # test/gstreamer_appsrc_webertc.cpp
  # test/thread_test.cpp
  src/main.cpp
  # webrtc_stream.cpp
)

ament_target_dependencies(main
  rclcpp
  sensor_msgs
)

target_link_libraries(main
  ${GST_LIBRARIES}
  ${Boost_LIBRARIES}
  ${RK_MPP_LIBRARIES}
  ${YUV_LIBRARY}
  # ${RGA_LIBRARY}
  jsoncpp
)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

add_executable(server
  src/server.cpp
)

target_link_libraries(server
  ${Boost_LIBRARIES}
  jsoncpp
  -lyuv
)

install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME})

ament_package()



