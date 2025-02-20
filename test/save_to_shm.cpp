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

int main(){
  // 打开设备
  int fd = open("/dev/video0", O_RDWR);
  if(fd == -1){
    std::cerr << "Failed to open /dev/video0" << std::endl;
    close(fd);
    return -1;
  }

  // 设置视频格式
  struct v4l2_format fmt;
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = WIDTH;
  fmt.fmt.pix.height = HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if(ioctl(fd, VIDIOC_S_FMT, &fmt) == -1){
    std::cerr << "Failed to set format" << std::endl;
    close(fd);
    return -1;
  }

  // 请求缓冲区
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = 4; // 请求4个缓冲区
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP; // 请求内存映射方式
  if(ioctl(fd, VIDIOC_REQBUFS, &req) == -1){
    std::cerr << "Failed to request buffers" << std::endl;
    close(fd);
    return -1;
  }

  // 查询和映射缓冲区
  // 使用VIDIOC_QUERYBUF查询缓冲区信息,并使用mmap映射到用户空间
  struct v4l2_buffer buf;
  for(int i=0; i < req.count; i++){
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if(ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1){
      std::cerr << "Failed to query buffer" << std::endl;
      close(fd);
      return -1;
    }

    buffers[i].length = buf.length;
    buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    if(buffers[i].start == MAP_FAILED){
      std::cerr << "Failed to mmap buffer" << std::endl;
      close(fd);
      return -1;
    }
  }
  
  // 使用 VIDIOC_QBUF 将缓冲区放入队列
  for(int i=0; i < req.count; i++){
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if(ioctl(fd, VIDIOC_QBUF, &buf) == -1){
      std::cerr << "Failed to enqueue buffer 1" << std::endl;
      close(fd);
      return -1;
    }
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(ioctl(fd, VIDIOC_STREAMON, &type) == -1){
    std::cerr << "Failed to start streaming" << std::endl;
    close(fd);
    return -1;
  }

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

  // int shm_fd = shm_open("/video0_shm", O_CREAT | O_RDWR, 0666);
  // if(shm_fd == -1){
  //   std::cerr << "Failed to create shared memory" << std::endl;
  //   close(shm_fd);
  //   close(fd);
  //   return -1;
  // }

  // if(ftruncate(shm_fd, WIDTH * HEIGHT * 2) == -1){
  //   std::cerr << "Failed to truncate shared memory" << std::endl;
  //   close(shm_fd);
  //   close(fd);
  //   return -1;
  // }

  while(true){
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if(ioctl(fd, VIDIOC_DQBUF, &buf) == -1){
      std::cerr << "Failed to dequeue buffer" << std::endl;
      close(fd);
      return -1;
    }

    // 处理缓冲区数据
    std::cout << "proccessing buffer" <<  std::endl;
    std::cout << "buffer bytesused: " << buf.bytesused << std::endl;

    if (buffers[buf.index].start == MAP_FAILED) {
      std::cerr << "缓冲区映射失败，索引: " << buf.index << std::endl;
      close(fd);
      return -1;
    }

    if (shm_ptr == nullptr) {
      std::cerr << "共享内存指针无效!" << std::endl;
      close(fd);
      return -1;
    }

    if (buf.bytesused <= WIDTH * HEIGHT * 2) {
      memcpy(shm_ptr->data, buffers[buf.index].start, buf.bytesused);
    } else {
      std::cerr << "警告：缓冲区数据超过共享内存的大小!" << std::endl;
      return -1;
    }
    
    // memcpy(shm_ptr->data, buffers[buf.index].start, buf.bytesused);
    shm_ptr->new_frame_available.store(true, std::memory_order_release);  // 使用原子操作更新标志位

    // 将缓冲区重新排队
    if(ioctl(fd, VIDIOC_QBUF, &buf) == -1){
      std::cerr << "Failed to enqueue buffer 3" << std::endl;
      close(fd);
      return -1;
    }
  }

  // 停止流采集
  if(ioctl(fd, VIDIOC_STREAMOFF, &type) == -1){
    std::cerr << "Failed to stop streaming" << std::endl;
    close(fd);
    return -1;
  }

  // 清理资源
  for(int i=0; i< req.count; i++){
    munmap(buffers[i].start, buffers[i].length);
  }
  close(fd);
}