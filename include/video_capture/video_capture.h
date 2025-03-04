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
#include <atomic>

#include "utils/util.h"
#include "share_data/share_data.h"


class VideoCapture{
public:
  VideoCapture(std::shared_ptr<ShareData> share_data): share_data_(share_data) {
    std::cout << "VideoCapture constructor" << std::endl;
  }

  bool init(){
    if(!init_device()){
      return false;
    }
    if(!init_mmp()){
      return false;
    }
    return true;
  }

  void start(){
    is_exit_.store(false);
    capture_thread_ = std::thread(&VideoCapture::start_capture, this);
  }

  void stop(){
    std::cout << "VideoCapture stop" << std::endl;
    is_exit_.store(true);
    // share_data_->data_ready_.store(false);
    if(capture_thread_.joinable()){
      capture_thread_.join();
    }
  }

  void start_capture(){
    enum v4l2_buf_type type;
    for(int i=0; i < share_data_->get_buffer_count(); i++){
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if(-1 == xioctl(share_data_->get_fd(), static_cast<int>(VIDIOC_QBUF), &buf)){
        std::cerr << "Failed to enqueue buffer" << std::endl;
        return;
      }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(-1 == xioctl(share_data_->get_fd(), static_cast<int>(VIDIOC_STREAMON), &type)){
      std::cerr << "Failed to start streaming" << std::endl;
      return;
    }
    
    std::cout << "Start capture" << std::endl;
    // share_data_->data_ready_.store(true);
    while(!is_exit_.load()){
      fd_set fds;
      struct timeval tv;
      int r;

      FD_ZERO(&fds);
      FD_SET(share_data_->get_fd(), &fds);

      tv.tv_sec = 5;
      tv.tv_usec = 0;

      r = select(share_data_->get_fd()+1, &fds, NULL, NULL, &tv);
      try{
        if(-1 == r){
          if(EINTR == errno){
            std::cerr << "Interrupted by signal" << std::endl;
            return;
          }
          std::cerr << "Something went wrong, exiting..." << errno << std::endl;
          throw errno;
        }
        if(0 == r){
          std::cerr << "Select timeout, exiting..." << std::endl;
          throw "select timeout";
        }
      }catch(const std::exception &e){
        std::cerr << "Exception: " << e.what() << std::endl;
        return;
      }

      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      if(-1 == xioctl(share_data_->get_fd(), static_cast<int>(VIDIOC_DQBUF), &buf)){
        std::cerr << "Failed to dequeue buffer" << std::endl;
        return;
      }

      {
        std::unique_lock<std::shared_mutex> lock(share_data_->get_mutex());
        share_data_->set_frame_buffer(share_data_->get_buffers()[buf.index].start, buf.bytesused);
      }
      

      if(-1 == xioctl(share_data_->get_fd(), static_cast<int>(VIDIOC_QBUF), &buf)){
        std::cerr << "Failed to enqueue buffer" << std::endl;
        return;
      }
    }
    std::cout << "End capture" << std::endl;
  }
    
  

  ~VideoCapture(){
    std::cout << "VideoCapture destructor" << std::endl;
    stop();
    if(share_data_->get_fd() != -1){
      close(share_data_->get_fd());
    }
  }

private:
  bool open_device(){
    // 打开设备
    share_data_->set_fd(open("/dev/video1", O_RDWR));
    if(share_data_->get_fd() == -1){
      std::cerr << "Failed to open /dev/video1" << std::endl;
      return false;
    }
    return true;
  }

  bool init_device(){
    if(!open_device()){
      return false;
    }

    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;

    if(-1 == xioctl(share_data_->get_fd(), static_cast<int>(VIDIOC_QUERYCAP), &cap)){
      std::cerr << "Failed to query capabilities" << std::endl;
      return false;
    }

    if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
      std::cerr << "The device does not support video capture" << std::endl;
      return false;
    }

    memset(&cropcap, 0, sizeof(cropcap));
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(0 == xioctl(share_data_->get_fd(), static_cast<int>(VIDIOC_CROPCAP), &cropcap)){
      crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      crop.c = cropcap.defrect;
      if(-1 == xioctl(share_data_->get_fd(), static_cast<int>(VIDIOC_S_CROP), &crop)){
        switch(errno){
          case EINVAL:
            break;
          default:
            break;
        }
      }
    }

    struct v4l2_format fmt;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = share_data_->get_width();
    fmt.fmt.pix.height = share_data_->get_height();
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if(-1 == xioctl(share_data_->get_fd(), static_cast<int>(VIDIOC_S_FMT), &fmt)){
      std::cerr << "Failed to set format" << std::endl;
      return false;
    }

    struct v4l2_streamparm stream_params;
    memset(&stream_params, 0, sizeof(stream_params));
    stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(-1 == xioctl(share_data_->get_fd(), static_cast<int>(VIDIOC_G_PARM), &stream_params)){
      std::cerr << "Failed to get stream params" << std::endl;
      return false;
    }
    if(!stream_params.parm.capture.capability && V4L2_CAP_TIMEPERFRAME){
      std::cerr << "The device does not support timeperframe" << std::endl;
      return false;
    }
    stream_params.parm.capture.timeperframe.numerator = 1;
    stream_params.parm.capture.timeperframe.denominator = 30;

    if(xioctl(share_data_->get_fd(), static_cast<int>(VIDIOC_S_PARM), &stream_params) < 0){
      std::cerr << "Failed to set stream params" << std::endl;
      return false;
    }
    return true;
  }

  bool init_mmp(){
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = share_data_->get_buffer_count();
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if(-1 == xioctl(share_data_->get_fd(), static_cast<int>(VIDIOC_REQBUFS), &req)){
      std::cerr << "Failed to request buffers" << std::endl;
      return false;
    }

    for(int i=0; i < share_data_->get_buffer_count(); i++){
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if(-1 == xioctl(share_data_->get_fd(), static_cast<int>(VIDIOC_QUERYBUF), &buf)){
        std::cerr << "Failed to query buffer" << std::endl;
        return false;
      }

      share_data_->set_buffer(i, reinterpret_cast<char *>(mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, share_data_->get_fd(), buf.m.offset)), 
                  buf.length);
      if(share_data_->get_buffers()[i].start == MAP_FAILED){
        std::cerr << "Failed to mmap buffer" << std::endl;
        return false;
      }
    }

    return true;
  }

private:
  std::shared_ptr<ShareData> share_data_;
  std::thread capture_thread_;
  std::atomic<bool> is_exit_{false};
};
