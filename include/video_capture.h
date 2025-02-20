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

class VideoCapture {
private:
  SharedMemory *shm_;
public:
  VideoCapture(SharedMemory *shm): shm_(shm) {}

  void write_data(){
    int i = 0;
    while(true){
      std::unique_lock<std::mutex> lock(shm_->mtx);

      // 等待数据被处理
      shm_->cv.wait(lock, [this]{ return shm_->data_processed; });

      *shm_->shared_data = i;
      std::cout << "Write thread: shared_data = " << *shm_->shared_data << std::endl;

      // 更新状态
      shm_->data_ready = true;
      shm_->data_processed = false;

      lock.unlock();
      shm_->cv.notify_one();

      // 更新数值
      i = (i == 100) ? 0 : i + 1;
    }
  }

};