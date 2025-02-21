#pragma once
#include <gst/gst.h>
#include <gst/app/app.h>
#include <gst/webrtc/webrtc.h>
#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <thread>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <cstring>

#include "shm.h"

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

class VideoStream{
private:
  SharedMemory *shm_;
  GstElement* appsrc_;
  GMainLoop* loop_;
  GstElement *pipeline_;
  GThread* thread_;
public:
  VideoStream(SharedMemory *shm): shm_(shm) {}

  void main(){
    gst_init(nullptr, nullptr);

    // 创建GStreamer管道
    GstElement* pipeline_ = gst_pipeline_new("pipeline");
    appsrc_ = gst_element_factory_make("appsrc", "appsrc");
    GstElement *videoconvert = gst_element_factory_make("videoconvert", "convert");
    GstElement *sink = gst_element_factory_make("autovideosink", "sink");

    if(!pipeline_){
      g_printerr("Pipeline could not be created.\n");
      return;
    }
    if(!appsrc_){
      g_printerr("Appsrc element could not be created.\n");
      return;
    }
    if(!videoconvert){
      g_printerr("Videoconvert element could not be created.\n");
      return;
    }
    if(!sink){
      g_printerr("Sink element could not be created.\n");
      return;
    }

    gst_bin_add_many(GST_BIN(pipeline_), appsrc_, videoconvert, sink, NULL);
    if(!gst_element_link_many(appsrc_, videoconvert, sink, NULL)){
      g_printerr("Elements could not be linked.\n");
      gst_object_unref(pipeline_);
      return;
    }

    // 设置caps格式为YUY2
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
                                        "format", G_TYPE_STRING, "YUY2",
                                        "width", G_TYPE_INT, WIDTH,
                                        "height", G_TYPE_INT, HEIGHT,
                                        "framerate", GST_TYPE_FRACTION, 30, 1,
                                        nullptr);
    g_object_set(appsrc_, "caps", caps, "do-timestamp", TRUE, nullptr);
    gst_caps_unref(caps);

    // 启动流水线
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    // 启动采集线程
    // thread_ = g_thread_new("capture", (GThreadFunc)capture_frames, this);
    thread_ = g_thread_new("capture", &capture_wrapper, this);
    // 运行主循环
    loop_ = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop_);
  }

  // 静态包装函数
  static gpointer capture_wrapper(gpointer data) {
    VideoStream* self = static_cast<VideoStream*>(data);
    self->capture_frames();
    return nullptr;
  }

  void capture_frames(){
    while(true){
      std::unique_lock<std::mutex> lock(shm_->mtx_);
      // 等待新数据就绪
      shm_->cv_.wait(lock, [this]{ return shm_->data_ready_; });
      // 读取数据
      // std::cout << "Read thread: shared_data = " << (int)shm_->shared_data_[0] << std::endl;
      GstBuffer* gst_buffer = gst_buffer_new_wrapped_full(
        GST_MEMORY_FLAG_READONLY,
        shm_->shared_data_,
        WIDTH * HEIGHT * 2,
        0,
        WIDTH * HEIGHT * 2,
        nullptr,
        nullptr
      );
      GstFlowReturn ret;
      g_signal_emit_by_name(appsrc_, "push-buffer", gst_buffer, &ret);
      gst_buffer_unref(gst_buffer);
      if(ret != GST_FLOW_OK){
        std::cerr << "Failed to push buffer: " << ret << std::endl;
      }
      // 更新状态
      shm_->data_ready_ = false;
      shm_->data_processed_ = true;
      lock.unlock();
      shm_->cv_.notify_one();
    }
  }

  ~VideoStream(){
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    gst_element_set_state(appsrc_, GST_STATE_NULL);
    gst_object_unref(appsrc_);
    g_thread_join(thread_);
    g_main_loop_unref(loop_);
  }

  // void read_data(){
  //   while(true){
  //     std::unique_lock<std::mutex> lock(shm_->mtx_);
      
  //     // 等待新数据就绪
  //     shm_->cv_.wait(lock, [this]{ return shm_->data_ready_; });

  //     // 读取数据
  //     std::cout << "Read thread: shared_data = " << (int)shm_->shared_data_[0] << std::endl;

  //     // 更新状态
  //     shm_->data_ready_ = false;
  //     shm_->data_processed_ = true;

  //     lock.unlock();
  //     shm_->cv_.notify_one();
  //   }
  // }
};