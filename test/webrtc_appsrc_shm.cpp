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

#include <gst/gst.h>
#include <gst/app/app.h>
#include <gst/app/gstappsrc.h>
#include <gst/webrtc/webrtc.h>
#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <thread>
#include <string>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;


#define WIDTH 640
#define HEIGHT 480

// 定义共享内存的内存结构
// struct SharedMemory{
//   unsigned char data[WIDTH * HEIGHT * 2];
//   std::atomic<bool> new_frame_available;
// };
struct SharedMemory{
  unsigned char* data[4]; // 指向缓冲区的指针数组,零拷贝
  size_t size[4];
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
  // fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
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
    shm_ptr->size[i] = buf.length;
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
    // std::cout << "proccessing buffer" <<  std::endl;
    // std::cout << "buffer bytesused: " << buf.bytesused << std::endl;

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
    // usleep(1000);
  }
}

void* display_thread(void* arg){
  while(true){
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
      }else{
        std::cout << "data is null" << std::endl;
      }

      shm_ptr->new_frame_available[read_index].store(false, std::memory_order_release);
      shm_ptr->read_index.store((read_index + 1) % 4, std::memory_order_release);
    }
    // usleep(1000);
  }
}

#define STUN_SERVER "stun://stun.l.google.com:19302"
#define SERVER_PORT 8000

GMainLoop *loop;
GstElement *pipeline, *webrtcbin;

void send_ice_candidate_message(websocket::stream<tcp::socket>& ws, guint mlineindex, gchar *candidate)
{
  std::cout << "Sending ICE candidate: mlineindex=" << mlineindex << ", candidate=" << candidate << std::endl;

  Json::Value ice_json;
  ice_json["candidate"] = candidate;
  ice_json["sdpMLineIndex"] = mlineindex;

  Json::Value msg_json;
  msg_json["type"] = "candidate";
  msg_json["ice"] = ice_json;

  Json::StreamWriterBuilder writer;
  std::string text = Json::writeString(writer, msg_json);
  ws.write(net::buffer(text));

  std::cout << "ICE candidate sent" << std::endl;
}

void on_answer_created(GstPromise *promise, gpointer user_data)
{
  std::cout << "Answer created" << std::endl;

  websocket::stream<tcp::socket>* ws = static_cast<websocket::stream<tcp::socket>*>(user_data);
  GstWebRTCSessionDescription *answer = NULL;
  const GstStructure *reply = gst_promise_get_reply(promise);
  gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, NULL);
  GstPromise *local_promise = gst_promise_new();
  g_signal_emit_by_name(webrtcbin, "set-local-description", answer, local_promise);

  Json::Value sdp_json;
  sdp_json["type"] = "answer";
  sdp_json["sdp"] = gst_sdp_message_as_text(answer->sdp);
  Json::StreamWriterBuilder writer;
  std::string text = Json::writeString(writer, sdp_json);
  ws->write(net::buffer(text));

  std::cout << "Local description set and answer sent: " << text << std::endl;

  gst_webrtc_session_description_free(answer);
}

void on_negotiation_needed(GstElement *webrtc, gpointer user_data)
{
  std::cout << "Negotiation needed" << std::endl;
}

void on_set_remote_description(GstPromise *promise, gpointer user_data)
{
  std::cout << "Remote description set, creating answer" << std::endl;

  websocket::stream<tcp::socket>* ws = static_cast<websocket::stream<tcp::socket>*>(user_data);
  GstPromise *answer_promise = gst_promise_new_with_change_func(on_answer_created, ws, NULL);

  g_signal_emit_by_name(webrtcbin, "create-answer", NULL, answer_promise);
}

void on_ice_candidate(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data)
{
  std::cout << "ICE candidate generated: mlineindex=" << mlineindex << ", candidate=" << candidate << std::endl;

  websocket::stream<tcp::socket>* ws = static_cast<websocket::stream<tcp::socket>*>(user_data);
  send_ice_candidate_message(*ws, mlineindex, candidate);
}

void on_need_data(GstElement* appsrc){
  int read_index = shm_ptr->read_index.load(std::memory_order_acquire);
  if(shm_ptr->new_frame_available[read_index].load(std::memory_order_acquire)){
    std::cout << "new frame available" << std::endl;
    // 直接访问共享内存中的映射缓冲区域
    unsigned char* g_data = shm_ptr->data[read_index];
    size_t size = shm_ptr->size[read_index];
    // 处理共享内存数据
    if(g_data != nullptr){
      GstBuffer *g_buffer;
      GstFlowReturn ret;
      GstMapInfo map;
      g_buffer = gst_buffer_new_allocate(NULL, size, NULL);
      gst_buffer_fill(g_buffer, 0, g_data, size);
      g_signal_emit_by_name(appsrc, "push-buffer", g_buffer, &ret);
      gst_buffer_unref(g_buffer);
    }else{
      std::cout << "data is null" << std::endl;
    }

    shm_ptr->new_frame_available[read_index].store(false, std::memory_order_release);
    shm_ptr->read_index.store((read_index + 1) % 4, std::memory_order_release);
  }
}

void handle_websocket_session(tcp::socket socket)
{
  try
  {
    websocket::stream<tcp::socket> ws{std::move(socket)};
    ws.accept();

    std::cout << "WebSocket connection accepted" << std::endl;

    GstStateChangeReturn ret;
    GError *error = NULL;

    pipeline = gst_pipeline_new("pipeline");
    // GstElement *v4l2src = gst_element_factory_make("v4l2src", "source");
    GstElement *appsrc = gst_element_factory_make("appsrc", "source");
    // GstElement *capsfilter1 = gst_element_factory_make("capsfilter", "capsfilter1");
    GstElement *videoconvert = gst_element_factory_make("videoconvert", "convert");
    GstElement *capsfilter2 = gst_element_factory_make("capsfilter", "capsfilter2");
    GstElement *queue = gst_element_factory_make("queue", "queue");
    GstElement *vp8enc = gst_element_factory_make("vp8enc", "encoder");
    GstElement *rtpvp8pay = gst_element_factory_make("rtpvp8pay", "pay");
    webrtcbin = gst_element_factory_make("webrtcbin", "sendrecv");

    if(!pipeline)
    {
      g_printerr("Pipeline could not be created.\n");
      return;
    }
    if(!appsrc)
    {
      g_printerr("V4L2 source element could not be created.\n");
      return;
    }
    if(!videoconvert)
    {
      g_printerr("Video convert element could not be created.\n");
      return;
    }
    if(!capsfilter2)
    {
      g_printerr("Caps filter element could not be created.\n");
      return;
    }
    if(!queue)
    {
      g_printerr("Queue element could not be created.\n");
      return;
    }
    if(!vp8enc)
    {
      g_printerr("VP8 encoder element could not be created.\n");
      return;
    }
    if(!rtpvp8pay)
    {
      g_printerr("RTP VP8 pay element could not be created.\n");
      return;
    }
    if(!webrtcbin)
    {
      g_printerr("WebRTC bin element could not be created.\n");
      return;
    }

    GstCaps *caps2 = gst_caps_new_simple("video/x-raw",
                                    "format", G_TYPE_STRING, "I420",
                                    "width", G_TYPE_INT, 640,
                                    "height", G_TYPE_INT, 480,
                                    NULL);

    g_object_set(capsfilter2, "caps", caps2, NULL);
    gst_caps_unref(caps2);
    g_object_set(webrtcbin, "stun-server", "stun://stun.l.google.com:19302", NULL);
    // g_object_set(v4l2src, "device", "/dev/video0", NULL);
    g_object_set(vp8enc, "deadline", 1, NULL);

    // 设置数据回调函数
    g_signal_connect(appsrc, "need-data", G_CALLBACK(on_need_data), NULL);
    g_object_set(appsrc, "stream-type", GST_APP_STREAM_TYPE_STREAM, NULL );

    gst_bin_add_many(GST_BIN(pipeline), appsrc, videoconvert, capsfilter2, queue, vp8enc, rtpvp8pay, webrtcbin, NULL);

    if (!gst_element_link_many(appsrc, videoconvert, capsfilter2, queue, vp8enc, rtpvp8pay, NULL))
    {
      g_printerr("Elements could not be linked.\n");
      gst_object_unref(pipeline);
      return;
    }

    GstPad *rtp_src_pad = gst_element_get_static_pad(rtpvp8pay, "src");
    GstPad *webrtc_sink_pad = gst_element_get_request_pad(webrtcbin, "sink_%u");
    gst_pad_link(rtp_src_pad, webrtc_sink_pad);
    gst_object_unref(rtp_src_pad);
    gst_object_unref(webrtc_sink_pad);

    g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(on_negotiation_needed), &ws);
    g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(on_ice_candidate), &ws);

    ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);

    if (ret == GST_STATE_CHANGE_FAILURE)
    {
      g_printerr("Unable to set the pipeline to the playing state.\n");
      gst_object_unref(pipeline);
      return;
    }

    std::cout << "GStreamer pipeline set to playing" << std::endl;

    for (;;)
    {
      beast::flat_buffer b_buffer;
      ws.read(b_buffer);

      auto text = beast::buffers_to_string(b_buffer.data());

      Json::Value obj;
      Json::CharReaderBuilder reader;      
      std::string errs;
      std::istringstream text_stream(text);
      if(!Json::parseFromStream(reader, text_stream, &obj, &errs))
      {
        std::cerr << "Error parsing JSON: " << errs << std::endl;
        continue;
      }

      std::string type = obj["type"].asString();

      if (type == "offer")
      {
        std::cout << "Received offer: " << text << std::endl;

        std::string sdp = obj["sdp"].asString();

        GstSDPMessage *sdp_message;
        gst_sdp_message_new_from_text(sdp.c_str(), &sdp_message);
        GstWebRTCSessionDescription *offer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp_message);
        GstPromise *promise = gst_promise_new_with_change_func(on_set_remote_description, &ws, NULL);
        g_signal_emit_by_name(webrtcbin, "set-remote-description", offer, promise);
        gst_webrtc_session_description_free(offer);

        std::cout << "Setting remote description" << std::endl;
      }
      else if (type == "candidate")
      {
        std::cout << "Received ICE candidate: " << text << std::endl;
        
        Json::Value ice = obj["ice"];
        std::string candidate = ice["candidate"].asString();
        guint sdpMLineIndex = ice["sdpMLineIndex"].asUInt();
        g_signal_emit_by_name(webrtcbin, "add-ice-candidate", sdpMLineIndex, candidate.c_str());

        std::cout << "Added ICE candidate" << std::endl;
      }
    }
  }
  catch (beast::system_error const& se)
  {
    if (se.code() != websocket::error::closed)
    {
      std::cerr << "Error: " << se.code().message() << std::endl;
    }
  }
  catch (std::exception const& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
  }
}

void start_server()
{
  try
  {
    net::io_context ioc{1};
    tcp::acceptor acceptor{ioc, tcp::endpoint{tcp::v4(), SERVER_PORT}};

    for (;;)
    {
      tcp::socket socket{ioc};
      acceptor.accept(socket);
      std::cout << "Accepted new TCP connection" << std::endl;
      std::thread{handle_websocket_session, std::move(socket)}.detach();
    }
  }
  catch (std::exception const& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
  }
}


void* webrtc_stream(void* arg){
  gst_init(nullptr, nullptr);
  loop = g_main_loop_new(NULL, FALSE);

  std::cout << "Starting WebRTC server" << std::endl;

  std::thread server_thread(start_server);
  g_main_loop_run(loop);

  server_thread.join();

  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);
  g_main_loop_unref(loop);

  std::cout << "WebRTC server stopped" << std::endl;

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
  }
  shm_ptr->write_index.store(0, std::memory_order_release);
  shm_ptr->read_index.store(0, std::memory_order_release);
 
  // 创建线程
  pthread_t capture_thread_id;
  pthread_create(&capture_thread_id, nullptr, capture_thread, nullptr);
  // pthread_t display_thread_id;
  // pthread_create(&display_thread_id, nullptr, display_thread, nullptr);
  pthread_t webrtc_thread_id;
  pthread_create(&webrtc_thread_id, nullptr, webrtc_stream, nullptr);


  pthread_join(capture_thread_id, nullptr);
  // pthread_join(display_thread_id, nullptr);
  pthread_join(webrtc_thread_id, nullptr);

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
