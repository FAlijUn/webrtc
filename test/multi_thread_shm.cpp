#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <cstring>
#include <sys/shm.h>
#include <atomic>

#define WIDTH 640
#define HEIGHT 480

// 定义共享内存的内存结构
struct SharedMemory{
  unsigned char data[WIDTH * HEIGHT * 2];
  std::atomic<bool> new_frame_available;
};
SharedMemory *shm_ptr;

struct buffer{
  void *start;
  size_t length;
};

struct buffer buffers[4];

int device_fd;

void* capture_thread(void* arg){
  device_fd = open("/dev/video0", O_RDWR);
  if(device_fd == -1){
    std::cerr << "Failed to open /dev/video0" << std::endl;
    close(device_fd);
    return nullptr;
  }

  // 配置v4l2格式
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = WIDTH;
  fmt.fmt.pix.height = HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if(ioctl(device_fd, VIDIOC_S_FMT, &fmt) == -1){
    std::cerr << "Failed to set format" << std::endl;
    close(device_fd);
    return nullptr;
  }

  // 请求缓冲区
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = 4; // 请求4个缓冲区
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP; // 请求内存映射方式
  if(ioctl(device_fd, VIDIOC_REQBUFS, &req) == -1){
    std::cerr << "Failed to request buffers" << std::endl;
    close(device_fd);
    return nullptr;
  }

  // 查询和映射缓冲区
  struct v4l2_buffer buf;
  for(int i=0; i < req.count; i++){
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if(ioctl(device_fd, VIDIOC_QUERYBUF, &buf) == -1){
      std::cerr << "Failed to query buffer" << std::endl;
      close(device_fd);
      return nullptr;
    }

    buffers[i].length = buf.length;
    buffers[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, device_fd, buf.m.offset);
    if(buffers[i].start == MAP_FAILED){
      std::cerr << "Failed to mmap buffer" << std::endl;
      close(device_fd);
      return nullptr;
    }
  }

  // 使用 VIDIOC_QBUF 将缓冲区放入队列
  for(int i=0; i < req.count; i++){
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if(ioctl(device_fd, VIDIOC_QBUF, &buf) == -1){
      std::cerr << "Failed to enqueue buffer" << std::endl;
      close(device_fd);
      return nullptr;
    }
  }

  // 开始流采集
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(ioctl(device_fd, VIDIOC_STREAMON, &type) == -1){
    std::cerr << "Failed to start streaming" << std::endl;
    close(device_fd);
    return nullptr;
  }

  while (true){
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if(ioctl(device_fd, VIDIOC_DQBUF, &buf) == -1){
      std::cerr << "Failed to dequeue buffer" << std::endl;
      close(device_fd);
      return nullptr;
    }

    // 处理缓冲区数据
    std::cout << "proccessing buffer" <<  std::endl;
    std::cout << "buffer bytesused: " << buf.bytesused << std::endl;

    if (buffers[buf.index].start == MAP_FAILED) {
      std::cerr << "Failed to mmap buffer" << std::endl;
      close(device_fd);
      return nullptr;
    }

    if (shm_ptr == nullptr) {
      std::cerr << "shm ptr is invalid" << std::endl;
      close(device_fd);
      return nullptr;
    }

    if (buf.bytesused <= WIDTH * HEIGHT * 2) {
      memcpy(shm_ptr->data, buffers[buf.index].start, buf.bytesused);
    } else {
      std::cerr << "Warning! Buffer bytesused is beyond buffer arry size" << std::endl;
      return nullptr;
    }
    shm_ptr->new_frame_available.store(true, std::memory_order_release);  // 使用原子操作更新标志位

    // 将缓冲区重新排队
    if(ioctl(device_fd, VIDIOC_QBUF, &buf) == -1){
      std::cerr << "Failed to enqueue buffer 3" << std::endl;
      close(device_fd);
      return nullptr;
    }
    usleep(1000);
  }
}

void* display_thread(void* arg){
  while(true){
    if(shm_ptr->new_frame_available.load(std::memory_order_acquire)){
      std::cout << "new frame available" << std::endl;
      // 处理共享内存数据
      shm_ptr->new_frame_available.store(false, std::memory_order_release);
    }
    usleep(1000);
  }
}

int main(){
  // 创建共享内存
  int shm_id = shmget(IPC_PRIVATE, sizeof(SharedMemory), IPC_CREAT | 0666);
  if (shm_id == -1) {
    perror("创建共享内存失败");
    return -1;
  }

  shm_ptr = (SharedMemory *)shmat(shm_id, nullptr, 0);
  if (shm_ptr == (void *)-1) {
    perror("映射共享内存失败");
    return -1;
  }
  
  // 初始化共享内存
  shm_ptr->new_frame_available.store(false, std::memory_order_release);
  memset(shm_ptr->data, 0, WIDTH * HEIGHT * 2);

  // 创建线程
  pthread_t capture_thread_id;
  pthread_create(&capture_thread_id, nullptr, capture_thread, nullptr);
  pthread_t display_thread_id;
  pthread_create(&display_thread_id, nullptr, display_thread, nullptr);

  pthread_join(capture_thread_id, nullptr);
  pthread_join(display_thread_id, nullptr);

  // 停止流采集
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(ioctl(device_fd, VIDIOC_STREAMOFF, &type) == -1){
    std::cerr << "Failed to stop streaming" << std::endl;
    close(device_fd);
    return -1;
  }

  // 释放缓冲区
  for(int i=0; i < 4; i++){
    if(munmap(buffers[i].start, buffers[i].length) == -1){
      std::cerr << "Failed to munmap buffer" << std::endl;
      close(device_fd);
      return -1;
    }
  }

  // 释放共享内存
  shmdt(shm_ptr);
  shmctl(shm_id, IPC_RMID, nullptr);
  return 0;
}
