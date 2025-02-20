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

#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_err.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_log.h>
#include <rockchip/mpp_packet.h>
#include <rockchip/mpp_rc_defs.h>
#include <rockchip/mpp_task.h>
#include <rockchip/rk_mpi.h>

#include <rga/drmrga.h>
#include <rga/RgaApi.h>
#include <rga/RgaUtils.h>

#include <csignal>

#define WIDTH 640
#define HEIGHT 480
#define MPP_ALIGN(x, a) (((x) + (a)-1) & ~((a)-1))

// 定义共享内存的内存结构
// struct SharedMemory{
//   unsigned char data[WIDTH * HEIGHT * 2];
//   std::atomic<bool> new_frame_available;
// };
struct SharedMemory{
  unsigned char* data[4]; // 指向缓冲区的指针数组,零拷贝
  size_t size[4]; // 缓冲区大小
  std::atomic<bool> new_frame_available[4];
  std::atomic<int> write_index;
  std::atomic<int> read_index;
};
SharedMemory *shm_ptr;

struct buffer{
  void *start;
  size_t length;
}__attribute__((aligned(16))); // 内存对齐

struct buffer buffers[4];
int device_fd;

std::atomic<bool> running(true);

void signal_handler(int signal) {
  if (signal == SIGINT) {
      std::cout << "Interrupt signal (Ctrl+C) received. Stopping..." << std::endl;
      running.store(false, std::memory_order_release);  // 标记程序停止
  }
}

uint8_t* rgb_buffer_ = nullptr;
MppCtx mpp_ctx_ = nullptr;
MppApi* mpp_api_ = nullptr;
MppPacket mpp_packet_ = nullptr;
MppFrame mpp_frame_ = nullptr;
MppDecCfg mpp_dec_cfg_ = nullptr;
MppBuffer mpp_frame_buffer_ = nullptr;
MppBuffer mpp_packet_buffer_ = nullptr;
uint8_t* data_buffer_ = nullptr;
MppBufferGroup mpp_frame_group_ = nullptr;
MppBufferGroup mpp_packet_group_ = nullptr;
MppTask mpp_task_ = nullptr;
uint32_t need_split_ = 0;

void rk_jpeg_decoder_init(){  
  rgb_buffer_ = new uint8_t[WIDTH * HEIGHT * 3];
  MPP_RET ret = mpp_create(&mpp_ctx_, &mpp_api_);
  if (ret != MPP_OK) {
    std::cerr << "mpp_create failed" << std::endl;
    return;
  }
  MpiCmd mpi_cmd = MPP_CMD_BASE;
  MppParam mpp_param = nullptr;

  mpi_cmd = MPP_DEC_SET_PARSER_SPLIT_MODE;
  mpp_param = &need_split_;
  ret = mpp_api_->control(mpp_ctx_, mpi_cmd, mpp_param);
  if (ret != MPP_OK) {
    std::cerr << "mpp_api_->control failed" << std::endl;
    return;
  }
  ret = mpp_init(mpp_ctx_, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG);
  if (ret != MPP_OK) {
    std::cerr << "mpp_init failed" << std::endl;
    return;
  }
  MppFrameFormat fmt = MPP_FMT_YUV420SP_VU;
  mpp_param = &fmt;
  ret = mpp_api_->control(mpp_ctx_, MPP_DEC_SET_OUTPUT_FORMAT, mpp_param);
  if (ret != MPP_OK) {
    std::cerr << "mpp_api_->control failed" << std::endl;
    return;
  }
  ret = mpp_frame_init(&mpp_frame_);
  if (ret != MPP_OK) {
    std::cerr << "mpp_frame_init failed" << std::endl;
    return;
  }
  ret = mpp_buffer_group_get_internal(&mpp_frame_group_, MPP_BUFFER_TYPE_ION);
  if (ret != MPP_OK) {
    std::cerr << "mpp_buffer_group_get_internal failed" << std::endl;
    return;
  }
  ret = mpp_buffer_group_get_internal(&mpp_packet_group_, MPP_BUFFER_TYPE_ION);
  if (ret != MPP_OK) {
    std::cerr << "mpp_buffer_group_get_internal failed" << std::endl;
    return;
  }
  RK_U32 hor_stride = MPP_ALIGN(WIDTH, 16);
  RK_U32 ver_stride = MPP_ALIGN(HEIGHT, 16);
  ret = mpp_buffer_get(mpp_frame_group_, &mpp_frame_buffer_,
                       hor_stride * ver_stride * 4);
  if (ret != MPP_OK) {
    std::cerr << "mpp_buffer_get failed" << std::endl;
    return;
  }
  mpp_frame_set_buffer(mpp_frame_, mpp_frame_buffer_);
  ret = mpp_buffer_get(mpp_packet_group_, &mpp_packet_buffer_,
                       WIDTH * HEIGHT * 3);
  if (ret != MPP_OK) {
    std::cerr << "mpp_buffer_get failed" << std::endl;
    return;
  }
  mpp_packet_init_with_buffer(&mpp_packet_, mpp_packet_buffer_);
  data_buffer_ = (uint8_t *)mpp_buffer_get_ptr(mpp_packet_buffer_);
}

bool mppFrame2RGB(const MppFrame frame, uint8_t *data) {
  int width = mpp_frame_get_width(frame);
  int height = mpp_frame_get_height(frame);
  MppBuffer buffer = mpp_frame_get_buffer(frame);
  if(width != WIDTH || height != HEIGHT){
    std::cerr << "mpp frame size error" << std::endl;
    return false;
  }
  if(data == nullptr){
    std::cerr << "data is nullptr" << std::endl;
    return false;
  }
  memset(data, 0, width * height * 3);
  auto buffer_ptr = mpp_buffer_get_ptr(buffer);
  if (buffer_ptr == nullptr) {
    std::cerr << "mpp buffer get ptr failed" << std::endl;
    return false;
  }
  rga_info_t src_info;
  rga_info_t dst_info;
  memset(&src_info, 0, sizeof(rga_info_t));
  memset(&dst_info, 0, sizeof(rga_info_t));
  src_info.fd = -1;
  src_info.mmuFlag = 1;
  src_info.virAddr = buffer_ptr;
  src_info.format = RK_FORMAT_YCbCr_420_SP;
  dst_info.fd = -1;
  dst_info.mmuFlag = 1;
  dst_info.virAddr = data;
  dst_info.format = RK_FORMAT_BGR_888;
  rga_set_rect(&src_info.rect, 0, 0, width, height, width, height,
               RK_FORMAT_YCbCr_420_SP);
  rga_set_rect(&dst_info.rect, 0, 0, width, height, width, height,
                RK_FORMAT_BGR_888);
  int ret = c_RkRgaBlit(&src_info, &dst_info, nullptr);
  if(ret){
    std::cerr << "c_RkRgaBlit error " << ret << " errno " << strerror(errno) << std::endl;
    return false;
  }
  return true;
}

void decode(const uint8_t *data, uint8_t *dest, size_t size) {
  MPP_RET ret = MPP_OK;
  memset(data_buffer_, 0, WIDTH * HEIGHT * 3);
  memcpy(data_buffer_, data, size);
  mpp_packet_set_pos(mpp_packet_, data_buffer_);
  mpp_packet_set_length(mpp_packet_, size);
  mpp_packet_set_eos(mpp_packet_);
  if(mpp_api_ == nullptr){
    std::cerr << "mpp_api_ is nullptr" << std::endl;
    return;
  }
  ret = mpp_api_->poll(mpp_ctx_, MPP_PORT_INPUT, MPP_POLL_BLOCK);
  if (ret != MPP_OK) {
    std::cerr << "mpp poll failed" << std::endl;
    return;
  }
  ret = mpp_api_->dequeue(mpp_ctx_, MPP_PORT_INPUT, &mpp_task_);
  if (ret != MPP_OK) {
    std::cerr << "mpp dequeue failed" << std::endl;
    return;
  }
  mpp_task_meta_set_packet(mpp_task_, KEY_INPUT_PACKET, mpp_packet_);
  mpp_task_meta_set_frame(mpp_task_, KEY_OUTPUT_FRAME, mpp_frame_);
  ret = mpp_api_->enqueue(mpp_ctx_, MPP_PORT_INPUT, mpp_task_);
  if (ret != MPP_OK) {
    std::cerr << "mpp enqueue failed" << std::endl;
    return;
  }
  ret = mpp_api_->poll(mpp_ctx_, MPP_PORT_OUTPUT, MPP_POLL_BLOCK);
  if (ret != MPP_OK) {
    std::cerr << "mpp poll failed" << std::endl;
    return;
  }
  ret = mpp_api_->dequeue(mpp_ctx_, MPP_PORT_OUTPUT, &mpp_task_);
  if (ret != MPP_OK) {
    std::cerr << "mpp dequeue failed" << std::endl;
    return;
  }
  if (mpp_task_) {
    MppFrame output_frame = nullptr;
    mpp_task_meta_get_frame(mpp_task_, KEY_OUTPUT_FRAME, &output_frame);
    if (mpp_frame_) {
      int width = mpp_frame_get_width(mpp_frame_);
      int height = mpp_frame_get_height(mpp_frame_);
      if (width != WIDTH || height != HEIGHT) {
        std::cerr << "mpp frame size error" << std::endl;
        return;
      }
      if (!mppFrame2RGB(mpp_frame_, rgb_buffer_)) {
        std::cerr << "mpp frame to rgb error" << std::endl;
        return;
      }
      if (mpp_frame_get_eos(output_frame)) {
        std::cout << "mpp frame get eos" << std::endl;
      }
    }
    ret = mpp_api_->enqueue(mpp_ctx_, MPP_PORT_OUTPUT, mpp_task_);
    if (ret != MPP_OK) {
      std::cerr << "mpp enqueue failed" << std::endl;
      return;
    }
    if(dest != nullptr){
      memcpy(dest, rgb_buffer_, WIDTH * HEIGHT * 3);
    }else{
      std::cerr << "dest is nullptr" << std::endl;
    }
  }else{
    std::cerr << "mpp_task_ is nullptr" << std::endl;
  }
}

void* capture_thread(void* arg){
  // 注册 SIGINT 信号处理函数
  signal(SIGINT, signal_handler);

  device_fd = open("/dev/video1", O_RDWR);
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

    // 将共享内存中的data指向映射的缓冲区
    shm_ptr->data[i] = (unsigned char *)buffers[i].start;
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

  while (running.load(std::memory_order_acquire)){
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

    // if (buf.bytesused <= WIDTH * HEIGHT * 2) {
    //   memcpy(shm_ptr->data, buffers[buf.index].start, buf.bytesused);
    // } else {
    //   std::cerr << "Warning! Buffer bytesused is beyond buffer arry size" << std::endl;
    //   return nullptr;
    // }
    // shm_ptr->new_frame_available.store(true, std::memory_order_release);  // 使用原子操作更新标志位

    // 写入到环形缓冲区
    int write_index = shm_ptr->write_index.load(std::memory_order_acquire);
    if(buf.bytesused <= WIDTH * HEIGHT * 2){
      // memcpy(shm_ptr->data[write_index], buffers[buf.index].start, buf.bytesused);
      shm_ptr->size[i] = buf.bytesused;
      shm_ptr->new_frame_available[write_index].store(true, std::memory_order_release);
      shm_ptr->write_index.store((write_index + 1) % 4, std::memory_order_release);
    }else{
      std::cerr << "Warning! Buffer bytesused is beyond buffer arry size" << std::endl;
      return nullptr;
    }


    // 将缓冲区重新排队
    if(ioctl(device_fd, VIDIOC_QBUF, &buf) == -1){
      std::cerr << "Failed to enqueue buffer 3" << std::endl;
      close(device_fd);
      return nullptr;
    }
    usleep(100000);
  }
  return nullptr;
}

void* display_thread(void* arg){
  while(running.load(std::memory_order_acquire)){
    // if(shm_ptr->new_frame_available.load(std::memory_order_acquire)){
    //   std::cout << "new frame available" << std::endl;
    //   // 处理共享内存数据
    //   shm_ptr->new_frame_available.store(false, std::memory_order_release);
    // }
    int read_index = shm_ptr->read_index.load(std::memory_order_acquire);
    if(shm_ptr->new_frame_available[read_index].load(std::memory_order_acquire)){
      std::cout << "new frame available" << std::endl;
      // 直接访问共享内存中的映射缓冲区域
      unsigned char* data = shm_ptr->data[read_index];
      size_t size = shm_ptr->size[read_index];
      // 处理共享内存数据
      if(data != nullptr){
        std::cout << "data is not null" << std::endl;
        std::cout << "data size: " << size << std::endl;
        uint8_t* dest = new uint8_t[WIDTH * HEIGHT * 3];
        decode(data, dest, size);
        for(int i=0; i < WIDTH * HEIGHT * 3; i++){
          std::cout << (int)dest[i] << " ";
        }
        std::cout << "--------------" <<std::endl;
        delete[] dest;
      }else{
        std::cout << "data is null" << std::endl;
      }

      shm_ptr->new_frame_available[read_index].store(false, std::memory_order_release);
      shm_ptr->read_index.store((read_index + 1) % 4, std::memory_order_release);
    }
    usleep(100000);
  }
  return nullptr;
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
  // shm_ptr->new_frame_available.store(false, std::memory_order_release);
  for(int i=0; i < 4; i++){
    shm_ptr->new_frame_available[i].store(false, std::memory_order_release);
    // memset(shm_ptr->data[i], 0, WIDTH * HEIGHT * 2);
    shm_ptr->data[i] = nullptr;
    shm_ptr->size[i] = 0;
  }
  shm_ptr->write_index.store(0, std::memory_order_release);
  shm_ptr->read_index.store(0, std::memory_order_release);

  // 初始化解码器
  rk_jpeg_decoder_init();

  // 创建线程
  pthread_t capture_thread_id;
  pthread_create(&capture_thread_id, nullptr, capture_thread, nullptr);
  pthread_t display_thread_id;
  pthread_create(&display_thread_id, nullptr, display_thread, nullptr);

  // 等待信号处理停止
  while (running.load(std::memory_order_acquire)) {
    // 主线程保持活跃，等待 Ctrl+C
    usleep(100000);  // sleep 100ms
  }

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
