#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "utils/mjpeg2yuv.h"

#define WIDTH 1280
#define HEIGHT 720
#define BUFFER_COUNT 2
// #define BUFFER_SIZE WIDTH * HEIGHT * 2

struct buffer{
  void *start;
  uint32_t length;
};

int xioctl(int fd, uint64_t request, void* arg) {
  int r = 0;

  do {
    r = ioctl(fd, request, arg);
    continue;
  } while (-1 == r && EINTR == errno);

  return r;
}

struct buffer buffers[BUFFER_COUNT];

int main(int argc, char** argv){
  
  // 初始化ros2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("video_capture_node");
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

  MjpegToYuvConverter* mjpeg2yuv = new MjpegToYuvConverter(u_int32_t(WIDTH),u_int32_t(HEIGHT));
  uint8_t* yuv_data = new uint8_t[WIDTH * HEIGHT * 3];
  if(!mjpeg2yuv->init()){
    std::cerr << "Failed to initialize mjpeg2yuv" << std::endl;
    return -1;
  }

  // 打开设备
  int fd = open("/dev/video1", O_RDWR);
  if(fd == -1){
    std::cerr << "Failed to open /dev/video1" << std::endl;
    return -1;
  }

  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;

  // 初始化设备
  if(-1 == xioctl(fd, static_cast<int>(VIDIOC_QUERYCAP), &cap)){
    std::cerr << "Failed to query device && Unable to initialize memory mapping" << std::endl;
    return -1;
  }

  if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
    std::cerr << "The device does not support video capture" << std::endl;
    return -1;
  }

  memset(&cropcap, 0, sizeof(cropcap));
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(0 == xioctl(fd, static_cast<int>(VIDIOC_CROPCAP), &cropcap)){
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect;
    if(-1 == xioctl(fd, static_cast<int>(VIDIOC_S_CROP), &crop)){
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
  fmt.fmt.pix.width = WIDTH;
  fmt.fmt.pix.height = HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;
  if(-1 == xioctl(fd, static_cast<int>(VIDIOC_S_FMT), &fmt)){
    std::cerr << "Failed to set format" << std::endl;
    return -1;
  }


  struct v4l2_streamparm stream_params;
  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(-1 == xioctl(fd, static_cast<int>(VIDIOC_G_PARM), &stream_params)){
    std::cerr << "Failed to get stream params" << std::endl;
    return -1;
  }
  if(!stream_params.parm.capture.capability && V4L2_CAP_TIMEPERFRAME){
    std::cerr << "The device does not support timeperframe" << std::endl;
    return -1;
  }
  stream_params.parm.capture.timeperframe.numerator = 1;
  stream_params.parm.capture.timeperframe.denominator = 30;

  if(xioctl(fd, static_cast<int>(VIDIOC_S_PARM), &stream_params) < 0){
    std::cerr << "Couldn't set camera framerate" << std::endl;
  }

  // 初始化mmp
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = BUFFER_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if(-1 == xioctl(fd, static_cast<int>(VIDIOC_REQBUFS), &req)){
    std::cerr << "Failed to request buffers" << std::endl;
    return -1;
  }

  for(int i=0; i < BUFFER_COUNT; i++){
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if(-1 == xioctl(fd, static_cast<int>(VIDIOC_QUERYBUF), &buf)){
      std::cerr << "Failed to query buffer" << std::endl;
      return -1;
    }

    buffers[i].start = reinterpret_cast<char *>(mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset));
    buffers[i].length = buf.length;

    if(MAP_FAILED == buffers[i].start){
      std::cerr << "Failed to mmap buffer" << std::endl;
      return -1;
    }
  }

  // 开始捕获
  unsigned int i;
  enum v4l2_buf_type type;

  for(int i=0; i < BUFFER_COUNT; i++){
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if(-1 == xioctl(fd, static_cast<int>(VIDIOC_QBUF), &buf)){
      std::cerr << "Unable to queue buffer" << std::endl;
      return -1;
    }
  }

  // 开始流
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(-1 == xioctl(fd, static_cast<int>(VIDIOC_STREAMON), &type)){
    std::cerr << "Unable to start stream" << std::endl;
    return -1;
  }

  const int period_ms = 1000.0 / 30;
  rclcpp::Rate loop_rate(30);
  while(rclcpp::ok()){
    fd_set fds;
    struct timeval tv;
    int r;
  
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
  
    /* Timeout. */
    tv.tv_sec = 5;
    tv.tv_usec = 0;
  
    r = select(fd + 1, &fds, NULL, NULL, &tv);
    try {
      if (-1 == r) {
        if (EINTR == errno) {
          return false;
        }
        std::cerr << "Something went wrong, exiting..." << errno << std::endl;
        throw errno;
      }
      if (0 == r) {
        std::cerr << "Select timeout, exiting..." << std::endl;
        throw "select timeout";
      }
    } catch (const std::exception &e) {
      return false;
    }

    struct v4l2_buffer buf;
    unsigned int i;
    int len;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if(-1 == xioctl(fd, static_cast<int>(VIDIOC_DQBUF), &buf)){
      switch(errno){
        case EAGAIN:
          continue;
        default:
          std::cerr << "Unable to retrieve frame with mmap" << std::endl;
          return -1;
      }
    }

    mjpeg2yuv->decode((uint8_t*)buffers[buf.index].start, buf.bytesused, yuv_data);
    sensor_msgs::msg::Image image_msg;
    image_msg.height = HEIGHT;
    image_msg.width = WIDTH;
    image_msg.encoding = "rgb8";
    image_msg.step = WIDTH * 3;
    image_msg.data.resize(WIDTH * HEIGHT * 3);
    memcpy(image_msg.data.data(), yuv_data, WIDTH * HEIGHT * 3);
    publisher->publish(image_msg);


    if(-1 == xioctl(fd, static_cast<int>(VIDIOC_QBUF), &buf)){
      std::cerr << "Unable to exchange buffer with the driver" << std::endl;
      return -1;
    }

    loop_rate.sleep();
  }



  // // 设置视频格式
  // struct v4l2_format fmt;
  // fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  // fmt.fmt.pix.width = WIDTH;
  // fmt.fmt.pix.height = HEIGHT;
  // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  // if(ioctl(fd, VIDIOC_S_FMT, &fmt) == -1){
  //   std::cerr << "Failed to set format" << std::endl;
  //   close(fd);
  //   return -1;
  // }



  // // struct v4l2_streamparm stream_params;
  // // memset(&stream_params, 0, sizeof(stream_params));
  // // stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  // // stream_params.parm.capture.timeperframe.numerator = 1;
  // // stream_params.parm.capture.timeperframe.denominator = 30;
  // // if(ioctl(fd, VIDIOC_S_PARM, &stream_params) == -1){
  // //   std::cerr << "Failed to set stream params" << std::endl;
  // //   close(fd);
  // //   return -1;
  // // }

  // // 请求缓冲区
  // struct v4l2_requestbuffers req;
  // memset(&req, 0, sizeof(req));
  // req.count = BUFFER_COUNT; // 请求4个缓冲区
  // req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  // req.memory = V4L2_MEMORY_MMAP; // 请求内存映射方式
  // if(ioctl(fd, VIDIOC_REQBUFS, &req) == -1){
  //   std::cerr << "Failed to request buffers" << std::endl;
  //   close(fd);
  //   return -1;
  // }

  // struct v4l2_buffer buf;
  // for(int i=0; i < BUFFER_COUNT ; i++){
  //   memset(&buf, 0, sizeof(buf));
  //   buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  //   buf.memory = V4L2_MEMORY_MMAP;
  //   buf.index = i;
  //   if(ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1){
  //     std::cerr << "Failed to query buffer" << std::endl;
  //     close(fd);
  //     return -1;
  //   }

  //   buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
  //   buffers[i].length = buf.bytesused;
  //   if(buffers[i].start == MAP_FAILED){
  //     std::cerr << "Failed to mmap buffer" << std::endl;
  //     close(fd);
  //     return -1;
  //   }
  // }

  // // 将缓冲区放入队列
  // for(int i=0; i < BUFFER_COUNT; i++){
  //   memset(&buf, 0, sizeof(buf));
  //   buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  //   buf.memory = V4L2_MEMORY_MMAP;
  //   buf.index = i;
  //   if(ioctl(fd, VIDIOC_QBUF, &buf) == -1){
  //     std::cerr << "Failed to enqueue buffer" << std::endl;
  //     close(fd);
  //     return -1;
  //   }
  // }

  // // 开始流采集
  // enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  // if(ioctl(fd, VIDIOC_STREAMON, &type) == -1){
  //   std::cerr << "Failed to start streaming" << std::endl;
  //   close(fd);
  //   return -1;
  // }

  // while (rclcpp::ok()){
  //   std::cout << buffers[0].length << std::endl;
  //   // 处理缓冲区数据
  //   if (buffers[0].start == MAP_FAILED) {
  //     std::cerr << "缓冲区映射失败，索引: " << buf.index << std::endl;
  //     close(fd);
  //     return -1;
  //   }
  //   if (buf.bytesused <= WIDTH * HEIGHT * 2) {
  //     mjpeg2yuv->decode((uint8_t*)buffers[0].start, buffers[0].length, yuv_data);
  //     sensor_msgs::msg::Image image_msg;
  //     image_msg.height = HEIGHT;
  //     image_msg.width = WIDTH;
  //     image_msg.encoding = "rgb8";
  //     image_msg.step = WIDTH * 3;
  //     image_msg.data.resize(WIDTH * HEIGHT * 3);
  //     memcpy(image_msg.data.data(), yuv_data, WIDTH * HEIGHT * 3);
  //     publisher->publish(image_msg);
  //   } else {
  //     std::cerr << "警告：缓冲区数据超过共享内存的大小!" << std::endl;
  //     return -1;
  //   }
  // }

  // 停止流采集
  if(ioctl(fd, VIDIOC_STREAMOFF, &type) == -1){
    std::cerr << "Failed to stop streaming" << std::endl;
    close(fd);
    return -1;
  }

  // 关闭设备
  rclcpp::shutdown(); 
  close(fd);
  return 0;
}