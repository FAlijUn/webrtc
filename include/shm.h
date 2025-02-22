#pragma once
#include <iostream>
#include <thread>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <chrono>
#include <mutex>
#include <shared_mutex>
// #include <condition_variable>  // 添加条件变量
#include <atomic>  

#define WIDTH 640
#define HEIGHT 480


class SharedMemory {
public:
  unsigned char shared_data_[WIDTH * HEIGHT * 2];
  size_t data_size_ = 0;
  const char *shm_name_ = "video0_shm";
  std::shared_mutex mtx_;
  // std::condition_variable ros_cv_;
  // std::condition_variable gst_cv_;
  // bool ros_data_ready_ = false;
  // bool gst_data_ready_ = false;
  SharedMemory() {}

  bool init(){
    int shm_fd = shm_open(shm_name_, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
      std::cerr << "Failed to open shared memory" << std::endl;
      return false;
    }

    if (ftruncate(shm_fd, sizeof(shared_data_)) == -1) {
      std::cerr << "Failed to truncate shared memory" << std::endl;
      close(shm_fd);
      return false;
    }

    void* ptr = mmap(0, sizeof(shared_data_), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
      std::cerr << "Failed to map shared memory" << std::endl;
      close(shm_fd);
      return false;
    }
    memcpy(shared_data_, ptr, sizeof(shared_data_));
    close(shm_fd);
    return true;
  }

  void cleanup(){
    munmap(shared_data_, sizeof(shared_data_));
    shm_unlink(shm_name_);
  }
};;
