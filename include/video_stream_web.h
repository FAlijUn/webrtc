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

#define STUN_SERVER "stun://stun.l.google.com:19302"
#define SERVER_PORT 8000

#define WIDTH 640
#define HEIGHT 480




class VideoStreamWeb{
private:
  SharedMemory *shm_;
  GstElement* appsrc_;
  GMainLoop* loop_;
  GstElement *pipeline_;
  static GstElement *webrtcbin_;
  GThread* thread_;
  std::thread server_thread_;
public:
  VideoStreamWeb(SharedMemory *shm): shm_(shm) {}

  void init(){
    gst_init(nullptr, nullptr);
    loop_ = g_main_loop_new(nullptr, FALSE);
    server_thread_ = std::thread(&VideoStreamWeb::start_server, this);
    g_main_loop_run(loop_);
    server_thread_.join();
  }

  void start_server(){
    try
    {
      net::io_context ioc{1};
      tcp::acceptor acceptor{ioc, tcp::endpoint{tcp::v4(), SERVER_PORT}};

      for (;;)
      {
        tcp::socket socket{ioc};
        acceptor.accept(socket);
        std::cout << "Accepted new TCP connection" << std::endl;
        // std::thread{&VideoStreamWeb::handle_websocket_session, this, std::move(socket)}.detach();
        std::thread{&VideoStreamWeb::handle_websocket_session, this, std::move(socket)}.detach();
      }
    }
    catch (std::exception const& e)
    {
      std::cerr << "Exception: " << e.what() << std::endl;
    }
  }

  void handle_websocket_session(tcp::socket socket){
    
    try{

      websocket::stream<tcp::socket> ws{std::move(socket)};
      ws.accept();

      // 创建GStreamer管道
      GstElement* pipeline_ = gst_pipeline_new("pipeline");
      appsrc_ = gst_element_factory_make("appsrc", "appsrc");
      GstElement *jpeg_dec = gst_element_factory_make("jpegdec", "jpeg_dec");
      GstElement *queue = gst_element_factory_make("queue", "queue");
      GstElement *videoconvert = gst_element_factory_make("videoconvert", "convert");
      GstElement *capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
      GstElement *vp8enc = gst_element_factory_make("vp8enc", "encoder");
      GstElement *rtpvp8pay = gst_element_factory_make("rtpvp8pay", "pay");
      webrtcbin_ = gst_element_factory_make("webrtcbin", "sendrecv");
      // GstElement *sink = gst_element_factory_make("autovideosink", "sink");

      if(!pipeline_){
        g_printerr("Pipeline could not be created.\n");
        return;
      }
      if(!appsrc_){
        g_printerr("Appsrc element could not be created.\n");
        return;
      }
      if(!jpeg_dec){
        g_printerr("Jpegdec element could not be created.\n");
        return;
      }
      if(!queue){
        g_printerr("Queue element could not be created.\n");
        return;
      }
      if(!videoconvert){
        g_printerr("Videoconvert element could not be created.\n");
        return;
      }
      if(!capsfilter){
        g_printerr("Caps filter element could not be created.\n");
        return;
      }
      if(!vp8enc){
        g_printerr("VP8 encoder element could not be created.\n");
        return;
      }
      if(!rtpvp8pay){
        g_printerr("RTP VP8 pay element could not be created.\n");
        return;
      }
      if(!webrtcbin_){
        g_printerr("WebRTC bin element could not be created.\n");
        return;
      }

      GstCaps* appsrc_caps = gst_caps_new_simple("image/jpeg",
        "width", G_TYPE_INT, WIDTH,
        "height", G_TYPE_INT, HEIGHT,
        "framerate", GST_TYPE_FRACTION, 30, 1,
        nullptr);
      g_object_set(appsrc_, "caps", appsrc_caps, "is-live", TRUE, "format", GST_FORMAT_TIME, nullptr);
      gst_caps_unref(appsrc_caps);

      GstCaps *video_caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "I420",
        "width", G_TYPE_INT, WIDTH,
        "height", G_TYPE_INT, HEIGHT,
        NULL);
      
      g_object_set(capsfilter, "caps", video_caps, NULL);
      gst_caps_unref(video_caps);

      g_object_set(webrtcbin_, "stun-server", "stun://stun.l.google.com:19302", NULL);
      g_object_set(vp8enc, "deadline", 1, NULL);
      // g_object_set(sink, "sync", FALSE, NULL);

      gst_bin_add_many(GST_BIN(pipeline_), appsrc_, jpeg_dec, queue, videoconvert, capsfilter, vp8enc, rtpvp8pay, webrtcbin_, NULL);
      if(!gst_element_link_many(appsrc_, jpeg_dec, queue, videoconvert, capsfilter, vp8enc, rtpvp8pay, NULL)){
        g_printerr("Elements could not be linked.\n");
        gst_object_unref(pipeline_);
        return;
      }

      GstPad *rtp_src_pad = gst_element_get_static_pad(rtpvp8pay, "src");
      GstPad *webrtc_sink_pad = gst_element_get_request_pad(webrtcbin_, "sink_%u");
      gst_pad_link(rtp_src_pad, webrtc_sink_pad);
      gst_object_unref(rtp_src_pad);
      gst_object_unref(webrtc_sink_pad);

      g_signal_connect(webrtcbin_, "on-negotiation-needed", G_CALLBACK(on_negotiation_needed), &ws);
      g_signal_connect(webrtcbin_, "on-ice-candidate", G_CALLBACK(on_ice_candidate), &ws);

      // 设置caps格式为YUY2
      // GstCaps* caps = gst_caps_new_simple("video/x-raw",
      //                                     "format", G_TYPE_STRING, "MJPEG",
      //                                     "width", G_TYPE_INT, WIDTH,
      //                                     "height", G_TYPE_INT, HEIGHT,
      //                                     "framerate", GST_TYPE_FRACTION, 30, 1,
      //                                     nullptr);

      GstStateChangeReturn ret;
      // 启动流水线
      ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);

      // 启动采集线程
      // thread_ = g_thread_new("capture", (GThreadFunc)capture_frames, this);
      thread_ = g_thread_new("capture", &capture_wrapper, this);

      if(ret == GST_STATE_CHANGE_FAILURE){
        g_printerr("Failed to start pipeline.\n");
        gst_object_unref(pipeline_);
        return;
      }
      std::cout << "GStreamer pipeline set to playing" << std::endl;

      while(true){
        beast::flat_buffer buffer;
        ws.read(buffer);
        auto text = beast::buffers_to_string(buffer.data());

        Json::Value obj;
        Json::CharReaderBuilder builder;
        std::string errs;
        std::istringstream text_stream(text);
        if(!Json::parseFromStream(builder, text_stream, &obj, &errs)){
          std::cerr << "Failed to parse JSON: " << errs << std::endl;
          return;
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
          g_signal_emit_by_name(webrtcbin_, "set-remote-description", offer, promise);
          gst_webrtc_session_description_free(offer);

          std::cout << "Setting remote description" << std::endl;
        }
        else if (type == "candidate")
        {
          std::cout << "Received ICE candidate: " << text << std::endl;
          
          Json::Value ice = obj["ice"];
          std::string candidate = ice["candidate"].asString();
          guint sdpMLineIndex = ice["sdpMLineIndex"].asUInt();
          g_signal_emit_by_name(webrtcbin_, "add-ice-candidate", sdpMLineIndex, candidate.c_str());

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

  

  // 静态包装函数
  static gpointer capture_wrapper(gpointer data) {
    VideoStreamWeb* self = static_cast<VideoStreamWeb*>(data);
    self->capture_frames();
    return nullptr;
  }



  void capture_frames(){
    std::cout << "capture_frame" << std::endl;
    while(true){
      std::unique_lock<std::mutex> lock(shm_->mtx_);
      // 等待新数据就绪
      shm_ -> gst_cv_.wait(lock, [this]{ return shm_->gst_data_ready_; });
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
      shm_->gst_data_ready_ = false;
      shm_->gst_data_processed_ = true;
      lock.unlock();
      shm_->gst_cv_.notify_all();
    }
  }

  static void send_ice_candidate_message(websocket::stream<tcp::socket>& ws, guint mlineindex, gchar *candidate);
  static void on_answer_created(GstPromise *promise, gpointer user_data);
  static void on_negotiation_needed(GstElement *webrtc, gpointer user_data);
  static void on_set_remote_description(GstPromise *promise, gpointer user_data);
  static void on_ice_candidate(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data);

  ~VideoStreamWeb(){
    if(server_thread_.joinable()){
      server_thread_.join();
    }
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

GstElement *VideoStreamWeb::webrtcbin_ = nullptr;

void VideoStreamWeb::send_ice_candidate_message(websocket::stream<tcp::socket>& ws, guint mlineindex, gchar *candidate)
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

  void VideoStreamWeb::on_answer_created(GstPromise *promise, gpointer user_data)
  {
    std::cout << "Answer created" << std::endl;

    websocket::stream<tcp::socket>* ws = static_cast<websocket::stream<tcp::socket>*>(user_data);
    GstWebRTCSessionDescription *answer = NULL;
    const GstStructure *reply = gst_promise_get_reply(promise);
    gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, NULL);
    GstPromise *local_promise = gst_promise_new();
    g_signal_emit_by_name(webrtcbin_, "set-local-description", answer, local_promise);

    Json::Value sdp_json;
    sdp_json["type"] = "answer";
    sdp_json["sdp"] = gst_sdp_message_as_text(answer->sdp);
    Json::StreamWriterBuilder writer;
    std::string text = Json::writeString(writer, sdp_json);
    ws->write(net::buffer(text));

    std::cout << "Local description set and answer sent: " << text << std::endl;

    gst_webrtc_session_description_free(answer);
  }

  void VideoStreamWeb::on_negotiation_needed(GstElement *webrtc, gpointer user_data)
  {
    std::cout << "Negotiation needed" << std::endl;
  }

  void VideoStreamWeb::on_set_remote_description(GstPromise *promise, gpointer user_data)
  {
    std::cout << "Remote description set, creating answer" << std::endl;

    websocket::stream<tcp::socket>* ws = static_cast<websocket::stream<tcp::socket>*>(user_data);
    GstPromise *answer_promise = gst_promise_new_with_change_func(on_answer_created, ws, NULL);

    g_signal_emit_by_name(webrtcbin_, "create-answer", NULL, answer_promise);
  }

  void VideoStreamWeb::on_ice_candidate(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data)
  {
    std::cout << "ICE candidate generated: mlineindex=" << mlineindex << ", candidate=" << candidate << std::endl;

    websocket::stream<tcp::socket>* ws = static_cast<websocket::stream<tcp::socket>*>(user_data);
    send_ice_candidate_message(*ws, mlineindex, candidate);
  }
