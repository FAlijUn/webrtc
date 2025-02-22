#pragma once
#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <cstring>
#include <sys/shm.h>
#include <thread>
#include <mutex>

#include "shm.h"

struct buffer{
  void *start;
  uint32_t length;
};

class VideoCapture {
private:
  SharedMemory *shm_;
  int fd_;
  struct buffer buffers_[4];
public:
  VideoCapture(SharedMemory *shm): shm_(shm) {}

  bool init(){
    // 打开设备
    if(!open_device()){
      return false;
    }
    
    // 设置视频格式
    if(!set_video_format()){
      return false;
    }

    // 请求缓冲区
    if(!request_buffers()){
      return false;
    }

    // 查询和映射缓冲区
    if(!mmap_buffers()){
      return false;
    }

    // 将缓冲区放入队列
    if(!enqueue_buffer()){
      return false;
    }

    // 开始流采集
    if(!start_streaming()){
      return false;
    }

    return true;
  } 

  bool open_device(){
    // 打开设备
    fd_ = open("/dev/video0", O_RDWR);
    if(fd_ == -1){
      std::cerr << "Failed to open /dev/video0" << std::endl;
      return false;
    }
    return true;
  }

  bool set_video_format(){
    // 设置视频格式
    struct v4l2_format fmt;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if(ioctl(fd_, VIDIOC_S_FMT, &fmt) == -1){
      std::cerr << "Failed to set format" << std::endl;
      close(fd_);
      return false;
    }
    return true;
  }

  bool request_buffers(){
    // 请求缓冲区
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 4; // 请求4个缓冲区
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP; // 请求内存映射方式
    if(ioctl(fd_, VIDIOC_REQBUFS, &req) == -1){
      std::cerr << "Failed to request buffers" << std::endl;
      close(fd_);
      return false;
    }
    return true;
  }

  bool mmap_buffers(){
    // 查询和映射缓冲区
    struct v4l2_buffer buf;
    for(int i=0; i < 4 ; i++){
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if(ioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1){
        std::cerr << "Failed to query buffer" << std::endl;
        close(fd_);
        return false;
      }

      buffers_[i].length = buf.length;
      buffers_[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
      if(buffers_[i].start == MAP_FAILED){
        std::cerr << "Failed to mmap buffer" << std::endl;
        close(fd_);
        return false;
      }
    }
    return true;
  }

  bool enqueue_buffer(){
    // 使用 VIDIOC_QBUF 将缓冲区放入队列
    struct v4l2_buffer buf;
    for(int i=0; i < 4; i++){
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if(ioctl(fd_, VIDIOC_QBUF, &buf) == -1){
        std::cerr << "Failed to enqueue buffer" << std::endl;
        close(fd_);
        return false;
      }
    }
    return true;
  }

  bool start_streaming(){
    // 开始流采集
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(fd_, VIDIOC_STREAMON, &type) == -1){
      std::cerr << "Failed to start streaming" << std::endl;
      close(fd_);
      return false;
    }
    return true;
  }

  void capture_data(){
    std::cout << "Capture thread started" << std::endl;
    while(true){
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      if(ioctl(fd_, VIDIOC_DQBUF, &buf) == -1){
        std::cerr << "Failed to dequeue buffer" << std::endl;
        close(fd_);
        return;
      }
      
      if (buffers_[buf.index].start == MAP_FAILED) {
        std::cerr << "Failed to mmp buffer: " << buf.index << std::endl;
        close(fd_);
        return;
      }

      if (shm_ == nullptr) {
        std::cerr << "Invaild shared ptr" << std::endl;
        close(fd_);
        return;
      }

      if (buf.bytesused <= WIDTH * HEIGHT * 2) {
        std::unique_lock<std::mutex> lock(shm_->mtx_);
        shm_ ->ros_cv_.wait(lock, [this]{ return shm_->ros_data_processed_; });
        shm_ ->gst_cv_.wait(lock, [this]{ return shm_->gst_data_processed_; });
        memcpy(shm_->shared_data_, buffers_[buf.index].start, buf.bytesused);
        shm_->data_size_ = buf.bytesused;
        shm_->ros_data_ready_ = true;
        shm_->gst_data_ready_ = true;
        shm_->ros_data_processed_ = false;
        shm_->gst_data_processed_ = false;
        std::cout << "Capture thread: shared_data = " << (int)shm_->shared_data_[0] << std::endl;
        lock.unlock();
        shm_->ros_cv_.notify_all();
        shm_->gst_cv_.notify_all();
      } else {
        std::cerr << "Warning, buffer size beyond shm size" << std::endl;
        return;
      }    
      // 将缓冲区重新排队
      if(ioctl(fd_, VIDIOC_QBUF, &buf) == -1){
        std::cerr << "Failed to enqueue buffer 3" << std::endl;
        close(fd_);
        return;
      }
    }
  }
  
  ~VideoCapture(){
    // 停止流采集
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(fd_, VIDIOC_STREAMOFF, &type) == -1){
      std::cerr << "Failed to stop streaming" << std::endl;
      close(fd_);
      return;
    }

    // 清理资源
    for(int i=0; i< 4 ; i++){
      munmap(buffers_[i].start, buffers_[i].length);
    }
    close(fd_);
  }
};