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
#include <vector>
#include <signal.h>
#include <memory>
#include <semaphore.h>


#include "utils/util.h"


class VideoCapture{
public:
  VideoCapture(){
    std::cout << "VideoCapture constructor" << std::endl;
  }

  bool init(){
    if(!init_device()){
      return false;
    }
    if(!init_shared_memory()){
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
    for(int i=0; i < 4; i++){
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if(-1 == xioctl(fd_, static_cast<int>(VIDIOC_QBUF), &buf)){
        std::cerr << "Failed to enqueue buffer" << std::endl;
        return;
      }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(-1 == xioctl(fd_, static_cast<int>(VIDIOC_STREAMON), &type)){
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
      FD_SET(fd_, &fds);

      tv.tv_sec = 5;
      tv.tv_usec = 0;

      r = select(fd_+1, &fds, NULL, NULL, &tv);
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
      if(-1 == xioctl(fd_, static_cast<int>(VIDIOC_DQBUF), &buf)){
        std::cerr << "Failed to dequeue buffer" << std::endl;
        return;
      }

      {
        memcpy(shm_ptr_, buffers_[buf.index].start, buf.length);
        sem_post(sem_);
      }

      if(-1 == xioctl(fd_, static_cast<int>(VIDIOC_QBUF), &buf)){
        std::cerr << "Failed to enqueue buffer" << std::endl;
        return;
      }
    }
    std::cout << "End capture" << std::endl;
  }
    
  

  ~VideoCapture(){
    std::cout << "VideoCapture destructor" << std::endl;
    stop();
    if(fd_ != -1){
      close(fd_);
    }
    if(shm_ptr_ != MAP_FAILED){
      munmap(shm_ptr_, shm_size_);
    }
    if(shm_fd_ != -1){
      close(shm_fd_);
    }
    if(sem_ != SEM_FAILED){
      sem_close(sem_);
    }
    sem_unlink("/frame_sem");
  }

private:
  bool open_device(){
    // 打开设备
    fd_ = open("/dev/video1", O_RDWR);
    if(fd_ == -1){
      std::cerr << "Failed to open /dev/video1" << std::endl;
      return false;
    }
    return true;
  }

  bool init_shared_memory(){
    shm_fd_ = shm_open("/frame_shm", O_CREAT | O_RDWR, 0666);
    if(shm_fd_ == -1){
      std::cerr << "Failed to open shared memory" << std::endl;
      return false;
    }
    shm_size_ = width_ * height_ * 2;
    if(ftruncate(shm_fd_, shm_size_) == -1){
      std::cerr << "Failed to truncate shared memory" << std::endl;
      return false;
    }
    shm_ptr_ = mmap(0, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if(shm_ptr_ == MAP_FAILED){
      std::cerr << "Failed to map shared memory" << std::endl;
      return false;
    }
    sem_ = sem_open("/frame_sem", O_CREAT, 0666, 0);
    if(sem_ == SEM_FAILED){
      std::cerr << "Failed to open semaphore" << std::endl;
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

    if(-1 == xioctl(fd_, static_cast<int>(VIDIOC_QUERYCAP), &cap)){
      std::cerr << "Failed to query capabilities" << std::endl;
      return false;
    }

    if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
      std::cerr << "The device does not support video capture" << std::endl;
      return false;
    }

    memset(&cropcap, 0, sizeof(cropcap));
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(0 == xioctl(fd_, static_cast<int>(VIDIOC_CROPCAP), &cropcap)){
      crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      crop.c = cropcap.defrect;
      if(-1 == xioctl(fd_, static_cast<int>(VIDIOC_S_CROP), &crop)){
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
    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if(-1 == xioctl(fd_, static_cast<int>(VIDIOC_S_FMT), &fmt)){
      std::cerr << "Failed to set format" << std::endl;
      return false;
    }

    struct v4l2_streamparm stream_params;
    memset(&stream_params, 0, sizeof(stream_params));
    stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(-1 == xioctl(fd_, static_cast<int>(VIDIOC_G_PARM), &stream_params)){
      std::cerr << "Failed to get stream params" << std::endl;
      return false;
    }
    if(!stream_params.parm.capture.capability && V4L2_CAP_TIMEPERFRAME){
      std::cerr << "The device does not support timeperframe" << std::endl;
      return false;
    }
    stream_params.parm.capture.timeperframe.numerator = 1;
    stream_params.parm.capture.timeperframe.denominator = 30;

    if(xioctl(fd_, static_cast<int>(VIDIOC_S_PARM), &stream_params) < 0){
      std::cerr << "Failed to set stream params" << std::endl;
      return false;
    }
    return true;
  }

  bool init_mmp(){
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = count_;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if(-1 == xioctl(fd_, static_cast<int>(VIDIOC_REQBUFS), &req)){
      std::cerr << "Failed to request buffers" << std::endl;
      return false;
    }

    buffers_.resize(count_);
    for(int i=0; i < 4; i++){
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if(-1 == xioctl(fd_, static_cast<int>(VIDIOC_QUERYBUF), &buf)){
        std::cerr << "Failed to query buffer" << std::endl;
        return false;
      }
      buffers_[i].length = buf.length;
      buffers_[i].start = mmap(NULL, buf.length, PROT_READ, MAP_SHARED, fd_, buf.m.offset);
      if (buffers_[i].start == MAP_FAILED) {
          close(fd_);
          return false;
      }
    }

    return true;
  }

private:
  int fd_;
  int width_= 1280;
  int height_= 720;
  int count_ = 4;
  std::vector<Buffer> buffers_;
  std::thread capture_thread_;
  std::atomic<bool> is_exit_{false};

  int shm_fd_ = -1;
  size_t shm_size_;
  void* shm_ptr_ = MAP_FAILED;
  sem_t* sem_ = SEM_FAILED;
};
