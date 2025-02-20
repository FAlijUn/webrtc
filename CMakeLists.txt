cmake_minimum_required(VERSION 3.10)

project(webrtc_stream)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 设置默认构建类型为 Debug
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()


find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0 gstreamer-app-1.0)
find_package(Boost REQUIRED)
# find_package(opencv REQUIRED)
# find_package(nlohmann_json REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(PkgConfig REQUIRED)

# pkg_search_module(RK_MPP REQUIRED rockchip_mpp)
# set(RGA_INCLUDE_DIR "/usr/include")
# set(RGA_LIBRARY "/usr/lib/aarch64-linux-gnu/librga.so")



include_directories(
  include
  ${GST_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${RK_MPP_INCLUDE_DIRS}
  ${RGA_INCLUDE_DIR}
  # ${OpenCV_INCLUDE_DIRS}
)

add_executable(webrtc_stream
  webrtc_stream.cpp
)

add_executable(test
  # test/available_devices.cpp
  # test/save_to_shm.cpp
  # test/multi_thread_shm.cpp
  # test/ring_buffer_shm.cpp
  # test/zero_copy_shm.cpp
  # test/data_proccess_shm.cpp
  # test/webrtc_appsrc_shm.cpp
  # test/gstreamer_appsrc.cpp
  # test/gstreamer_appsrc_webertc.cpp
  src/main.cpp
)


target_link_libraries(test
  ${GST_LIBRARIES}
  ${Boost_LIBRARIES}
  ${RK_MPP_LIBRARIES}
  ${RGA_LIBRARY}
  # ${OpenCV_LIBS}
  # nlohmann_json::nlohmann_json
  jsoncpp
)

target_link_libraries(webrtc_stream
  ${GST_LIBRARIES}
  ${Boost_LIBRARIES}
  ${RK_MPP_LIBRARIES}
  ${RGA_LIBRARY}
  # ${OpenCV_LIBS}
  # nlohmann_json::nlohmann_json
  jsoncpp
)

# set_target_properties(test PROPERTIES
#   CXX_STANDARD 14
#   CXX_STANDARD_REQUIRED ON
# )

link_directories(
  ${GST_LIBRARY_DIRS}
)