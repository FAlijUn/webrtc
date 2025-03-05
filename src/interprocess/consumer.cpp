#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <unistd.h>
#include <cstring>

int main() {
  // 打开共享内存
  int shm_fd = shm_open("/video1_shm", O_RDONLY, 0666);
  if (shm_fd == -1) {
    std::cerr << "Failed to open shared memory" << std::endl;
    return -1;
  }

  // 获取共享内存大小
  size_t shm_size = 640 * 480 * 2; // Assuming 3 bytes per pixel (RGB)

  // 映射共享内存到进程地址空间
  void* shm_ptr = mmap(0, shm_size, PROT_READ, MAP_SHARED, shm_fd, 0);
  if (shm_ptr == MAP_FAILED) {
    std::cerr << "Failed to map shared memory" << std::endl;
    close(shm_fd);
    return -1;
  }

  // 打开信号量
  sem_t* sem = sem_open("/video1_sem", 0);
  if (sem == SEM_FAILED) {
    std::cerr << "Failed to open semaphore" << std::endl;
    munmap(shm_ptr, shm_size);
    close(shm_fd);
    return -1;
  }

  while (true) {
    // 等待信号量通知数据可用
    sem_wait(sem);

    // 读取共享内存中的数据
    char* frame_data = new char[shm_size];
    memcpy(frame_data, shm_ptr, shm_size);

    // 处理帧数据
    std::cout << "Frame data received" << std::endl;
    std::cout << (int)frame_data[0] << (int)frame_data[1] << (int)frame_data[2] << std::endl;
    // 释放帧数据内存
    delete[] frame_data;
  }

  // 清理资源
  munmap(shm_ptr, shm_size);
  close(shm_fd);
  sem_close(sem);

  return 0;
}