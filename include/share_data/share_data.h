#pragma once
#include <stdint.h>
#include <memory>
#include <mutex>
#include <iostream>
#include <linux/videodev2.h>
#include <shared_mutex>
#include <atomic>
#include "utils/util.h"

class ShareData
{
public:
  ShareData(uint32_t width, uint32_t height, uint32_t buffer_count):
    width_(width),
    height_(height),
    buffer_count_(buffer_count),
    buffers_(std::shared_ptr<Buffer>(new Buffer[buffer_count], std::default_delete<Buffer[]>())),
    frame_buffer_(std::make_shared<Buffer>())
    { std::cout << "ShareData constructor" << std::endl; }

  void set_fd(uint32_t fd){
    fd_ = fd;
  }

  void set_width(uint32_t width){
    width_ = width;
  }

  void set_height(uint32_t height){
    height_ = height;
  }

  void set_buffer_count(uint32_t buffer_count){
    buffer_count_ = buffer_count;
  }

  void set_buffer(int index, void *start, uint32_t length){
    buffers_.get()[index].start = start;
    buffers_.get()[index].length = length;
  }

  void set_frame_buffer(void *start, uint32_t length){
    frame_buffer_->start = start;
    frame_buffer_->length = length;
  }

  uint32_t get_fd(){
    return fd_;
  }

  u_int32_t get_width(){
    return width_;
  }

  u_int32_t get_height(){
    return height_;
  }

  u_int32_t get_buffer_count(){
    return buffer_count_;
  }

  Buffer *get_buffers(){
    return buffers_.get();
  }

  std::shared_mutex &get_mutex(){
    return mtx_;
  }

  Buffer *get_frame_buffer(){
    return frame_buffer_.get();
  }

  ~ShareData(){
    std::cout << "ShareData destructor" << std::endl;
  }

public:
  // std::atomic<bool> data_ready_{false};
private:
  uint32_t width_;
  uint32_t height_;
  uint32_t buffer_count_;
  uint32_t fd_;
  std::shared_ptr<Buffer> buffers_;
  std::shared_ptr<Buffer> frame_buffer_;
  std::shared_mutex mtx_;
};