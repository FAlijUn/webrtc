#pragma once

#include "mjpeg2yuv.h"
#include "shm.h"
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cstring>

class VideoROS{
public:
  VideoROS(SharedMemory *shm, rclcpp::Node::SharedPtr node)
    : shm_(shm), node_(node) {
    std::cout << "Video ROS Start" << std::endl;
    publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("rgb_image", 10);
  }
  
  bool init(){
    mjpeg2yuv_ = new MjpegToYuvConverter();
    if(!mjpeg2yuv_->init()){
      std::cerr << "Failed to initialize mjpeg2yuv" << std::endl;
      return false;
    }
    return true;
  }

  void read_data(){
    std::cout << "Read thread started" << std::endl;
    uint8_t* yuv_data = new uint8_t[WIDTH * HEIGHT * 3];
    while(!is_exit_.load()){
      {
        std::shared_lock<std::shared_mutex> lock(shm_->mtx_);
        mjpeg2yuv_->decode(shm_->shared_data_, shm_->data_size_, yuv_data);
      }
      auto img_msg = sensor_msgs::msg::Image();
      img_msg.header.stamp = node_->now();
      img_msg.header.frame_id = "camera_frame";
      img_msg.height = HEIGHT;
      img_msg.width = WIDTH;
      img_msg.encoding = "rgb8";
      img_msg.is_bigendian = 0;
      img_msg.step = WIDTH * 3;
      img_msg.data.resize(WIDTH * HEIGHT * 3);
      std::memcpy(&img_msg.data[0], yuv_data, WIDTH * HEIGHT * 3);
      publisher_->publish(img_msg);  
      usleep(33333);
    }
    delete[] yuv_data;
  }

  void stop(){
    is_exit_.store(true);
  }

  ~VideoROS(){
    std::cout<< "Video ROS End" << std::endl;
    is_exit_.store(true);
  }

private:
  SharedMemory *shm_;
  MjpegToYuvConverter* mjpeg2yuv_= nullptr;
  std::atomic<bool> is_exit_{false};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};