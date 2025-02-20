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
#include <condition_variable>  // 添加条件变量

class SharedMemory {
public:
  int *shared_data;
  const char *shm_name = "video0_shm";
  std::mutex mtx;
  std::condition_variable cv;
  bool data_ready = false; // 新的数据是否已经就绪
  bool data_processed = true; // 数据是否已经被处理


  SharedMemory(): shared_data(nullptr) {}

  bool init(){
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
      std::cerr << "Failed to open shared memory" << std::endl;
      return false;
    }

    if (ftruncate(shm_fd, sizeof(int)) == -1) {
      std::cerr << "Failed to truncate shared memory" << std::endl;
      close(shm_fd);
      return false;
    }

    shared_data = (int *)mmap(0, sizeof(int), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_data == MAP_FAILED) {
      std::cerr << "Failed to map shared memory" << std::endl;
      close(shm_fd);
      return false;
    }
    close(shm_fd);
    return true;
  }

  void cleanup(){
    munmap(shared_data, sizeof(int));
    shm_unlink(shm_name);
  }
};
