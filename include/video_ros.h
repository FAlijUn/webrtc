#pragma once

// #include "mjpeg2yuv.h"
#include "shm.h"

class VideoROS{
private:
  SharedMemory *shm_;
  // MjpegToYuvConverter* mjpeg2yuv_= nullptr;
public:
  VideoROS(SharedMemory *shm): shm_(shm) {}
  
  bool init(){
    // mjpeg2yuv_ = new MjpegToYuvConverter();
    // if(!mjpeg2yuv_->init()){
    //   std::cerr << "Failed to initialize mjpeg2yuv" << std::endl;
    //   return false;
    // }

    return true;
  }

  void read_data(){
    while(true){
      std::unique_lock<std::mutex> lock(shm_->mtx_);
      
      // 等待新数据就绪
      shm_->ros_cv_.wait(lock, [this]{ return shm_->ros_data_ready_; });

      // 读取数据
      std::cout << "Read thread: shared_data = " << (int)shm_->shared_data_[0] << std::endl;
      std::cout << "Read thread: data_size = " << shm_->data_size_ << std::endl;
      // uint8_t* yuv_data = new uint8_t[WIDTH * HEIGHT];
      // mjpeg2yuv_->decode(shm_->shared_data_, shm_->data_size_, yuv_data);

      // 更新状态
      // shm_->data_ready_ = false;
      // shm_->ros_data_processed_ = true;
      
      shm_->ros_data_ready_ = false;
      shm_->ros_data_processed_ = true;

      lock.unlock();
      shm_->ros_cv_.notify_all();
    }
  }
};