#pragma once
#include <memory>
#include <thread>
#include <mutex>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "utils/mjpeg2yuv.h"
#include "share_data/share_data.h"
#include <atomic>

class Ros2Publish{
public:
  Ros2Publish(std::shared_ptr<ShareData> share_data): share_data_(share_data){
    std::cout << "Ros2Publish constructor" << std::endl;
    node_ = rclcpp::Node::make_shared("camera_node");
    publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    mjpeg2yuv_ = std::make_shared<MjpegToYuvConverter>(share_data_->get_width(), share_data_->get_height());  
    yuv_data_ = std::shared_ptr<uint8_t>(new uint8_t[share_data->get_width() * share_data->get_height() * 3], 
      std::default_delete<uint8_t[]>());
  }

  bool init(){
    if(!mjpeg2yuv_->init()){
      std::cerr << "Failed to initialize mjpeg2yuv" << std::endl;
      return false;
    }
    return true;
  }

  void start()
  {
    is_exit_.store(false);
    publish_thread_ = std::thread(&Ros2Publish::update, this); 
  }

  void update(){
    std::cout << "Start publish" << std::endl;
    const int period_ms = 1000.0 / 30;
    rclcpp::Rate loop_rate(30);
    while(rclcpp::ok && !is_exit_.load()){
      // if(!share_data_->data_ready_.load()){
      //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
      //   continue;
      // }
      {
        std::shared_lock<std::shared_mutex> lock(share_data_->get_mutex());
        if(share_data_->get_frame_buffer()->length == 0){
          continue;
        }
        if(share_data_->get_frame_buffer()->start == nullptr){
          continue;
        }
        mjpeg2yuv_->decode((uint8_t*)share_data_->get_frame_buffer()->start, share_data_->get_frame_buffer()->length, yuv_data_.get());
      }
      sensor_msgs::msg::Image image_msg;
      image_msg.height = share_data_->get_height();
      image_msg.width = share_data_->get_width();
      image_msg.encoding = "rgb8";
      image_msg.step = share_data_->get_width() * 3;
      image_msg.data.resize(share_data_->get_width() * share_data_->get_height() * 3);
      memcpy(image_msg.data.data(), yuv_data_.get(), share_data_->get_width() * share_data_->get_height() * 3);
      publisher_->publish(image_msg);
      loop_rate.sleep();
    }
    std::cout << "End publish" << std::endl;
  }
  

  void stop(){
    is_exit_.store(true);
    if(publish_thread_.joinable()){
      publish_thread_.join();
    }
    rclcpp::shutdown();
  }

  ~Ros2Publish(){
    std::cout << "Ros2Publish destructor" << std::endl;
    stop();
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  MjpegToYuvConverter::Ptr mjpeg2yuv_;
  std::shared_ptr<uint8_t> yuv_data_;
  std::shared_ptr<ShareData> share_data_;
  std::thread publish_thread_;
  std::atomic<bool> is_exit_{false};
};