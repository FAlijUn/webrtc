#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <cstring>
#include <gst/gst.h>
#include <gst/app/app.h>

#define WIDTH 640
#define HEIGHT 480

typedef struct {
    void* start;
    size_t length;
} V4L2Buffer;

typedef struct {
    int fd;
    V4L2Buffer* buffers;
} CaptureArgs;

static GstElement* appsrc;

static void capture_frames(CaptureArgs* args) {
  int fd = args->fd;
  V4L2Buffer* buffers = args->buffers;

  while (true) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
      std::cerr << "Failed to dequeue buffer" << std::endl;
      break;
    }

    // 推送数据到appsrc
    GstBuffer* gst_buffer = gst_buffer_new_wrapped_full(
      GST_MEMORY_FLAG_READONLY,
      buffers[buf.index].start,
      buf.bytesused,
      0,
      buf.bytesused,
      nullptr,
      nullptr
    );

    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc, "push-buffer", gst_buffer, &ret);
    gst_buffer_unref(gst_buffer);

    if (ret != GST_FLOW_OK) {
      std::cerr << "Failed to push buffer: " << ret << std::endl;
    }

    if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
      std::cerr << "Failed to enqueue buffer" << std::endl;
      break;
    }
  }

  delete args;
}

int main() {
  int device_fd = open("/dev/video0", O_RDWR);
  if (device_fd == -1) {
    std::cerr << "Failed to open device" << std::endl;
    return -1;
  }

  // 配置V4L2格式
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = WIDTH;
  fmt.fmt.pix.height = HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  if (ioctl(device_fd, VIDIOC_S_FMT, &fmt) == -1) {
    std::cerr << "Failed to set format" << std::endl;
    close(device_fd);
    return -1;
  }

  // 请求缓冲区
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(device_fd, VIDIOC_REQBUFS, &req) == -1) {
    std::cerr << "Failed to request buffers" << std::endl;
    close(device_fd);
    return -1;
  }

  // 映射缓冲区
  V4L2Buffer* buffers = new V4L2Buffer[req.count];
  for (int i = 0; i < req.count; ++i) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (ioctl(device_fd, VIDIOC_QUERYBUF, &buf) == -1) {
        std::cerr << "Failed to query buffer" << std::endl;
        close(device_fd);
        return -1;
    }
    buffers[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, device_fd, buf.m.offset);
    buffers[i].length = buf.length;
  }

  // 入队所有缓冲区
  for (int i = 0; i < req.count; ++i) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (ioctl(device_fd, VIDIOC_QBUF, &buf) == -1) {
        std::cerr << "Failed to enqueue buffer" << std::endl;
        close(device_fd);
        return -1;
    }
  }

  // 启动视频流
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(device_fd, VIDIOC_STREAMON, &type) == -1) {
    std::cerr << "Failed to start streaming" << std::endl;
    close(device_fd);
    return -1;
  }

  // 初始化GStreamer
  gst_init(nullptr, nullptr);
  GstElement* pipeline = gst_pipeline_new("pipeline");
  appsrc = gst_element_factory_make("appsrc", "appsrc");
  GstElement* videoconvert = gst_element_factory_make("videoconvert", "convert");
  GstElement* sink = gst_element_factory_make("autovideosink", "sink");

  gst_bin_add_many(GST_BIN(pipeline), appsrc, videoconvert, sink, nullptr);
  gst_element_link_many(appsrc, videoconvert, sink, nullptr);

  // 设置caps格式为YUY2
  GstCaps* caps = gst_caps_new_simple("video/x-raw",
                                      "format", G_TYPE_STRING, "YUY2",
                                      "width", G_TYPE_INT, WIDTH,
                                      "height", G_TYPE_INT, HEIGHT,
                                      "framerate", GST_TYPE_FRACTION, 30, 1,
                                      nullptr);
  g_object_set(appsrc, "caps", caps, "do-timestamp", TRUE, nullptr);
  gst_caps_unref(caps);

  // 启动流水线
  gst_element_set_state(pipeline, GST_STATE_PLAYING);

  // 启动采集线程
  CaptureArgs* args = new CaptureArgs{device_fd, buffers};
  GThread* thread = g_thread_new("capture", (GThreadFunc)capture_frames, args);

  // 运行主循环
  GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
  g_main_loop_run(loop);

  // 清理资源
  g_main_loop_unref(loop);
  g_thread_join(thread);
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);
  close(device_fd);

  for (int i = 0; i < req.count; ++i) {
      munmap(buffers[i].start, buffers[i].length);
  }
  delete[] buffers;

  return 0;
}