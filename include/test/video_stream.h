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
#include <atomic>

#include "shm.h"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using wsts = beast::websocket::stream<beast::tcp_stream>;

#define STUN_SERVER "stun://stun.l.google.com:19302"
#define SERVER_PORT 8000

#define WIDTH 640
#define HEIGHT 480

class VideoStream{
public:
  VideoStream(SharedMemory *shm, const std::string& host, const std::string& port)
  :shm_(shm), host_(host), port_(port), io_context_(), ws_(io_context_){}

  void init(){
    std::cout << "video stream init" << std::endl;
    gst_init(nullptr, nullptr);
    loop_ = g_main_loop_new(nullptr, FALSE);
    if(!init_wbrtc_stream()){
      is_exit_.store(true);
    }
    connect_thread_ = std::thread(&VideoStream::connect_to_signaling_server, this);
    connect_thread_.detach();
    thread_ = g_thread_new("capture", &capture_wrapper, this);
    g_main_loop_run(loop_);
  }

  void stop(){
    is_exit_.store(true);
    running_.store(false);
  }

  ~VideoStream(){
    is_exit_.store(true);
    running_.store(false);
    if(listen_thread_.joinable()){
      listen_thread_.join();
    }
    if(connect_thread_.joinable()){
      connect_thread_.join();
    }
    ws_.close(beast::websocket::close_code::normal);
    g_main_loop_quit(loop_);
    g_thread_join(thread_);
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    gst_deinit();
  }

  static void on_set_remote_description(GstPromise *promise, gpointer user_data);
  static void on_answer_created(GstPromise *promise, gpointer user_data);
  static void on_negotiation_needed(GstElement *webrtc, gpointer user_data);
  static void on_ice_candidate(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data);
  static void send_ice_candidate_message(wsts& ws, guint mlineindex, gchar *candidate);

private:
  void connect_to_signaling_server()
  {
    std::cout << "start to connect signaling server";
    while(!is_exit_.load() && !running_.load()){
      try{
        // 解析主机名和端口
        tcp::resolver resolver{io_context_};
        auto const results = resolver.resolve(host_, port_); // 使用实际端口号

        // 连接底层 TCP 套接字
        beast::get_lowest_layer(ws_).connect(results);

        ws_.handshake(host_, "/ws");
        // 启动后台线程监听信令消息
        // std::thread([this]() { listen_for_signaling_messages(); }).detach();
      }catch(...){
        std::cerr << "there is no connect with " << host_ << ":" << port_ << std::endl;
        usleep(1000000);
        continue;
      }
      running_.store(true);
      listen_thread_ = std::thread([this]() { listen_for_signaling_messages(); });
      listen_thread_.detach();
    }
  }

  static gpointer capture_wrapper(gpointer data) {
    VideoStream* self = static_cast<VideoStream*>(data);
    self->capture_frames();
    return nullptr;
  }

  void capture_frames(){
    std::cout << "start capture frames" << std::endl;
    while(!is_exit_.load()){
      if(running_.load())
      {
        std::shared_lock<std::shared_mutex> lock(shm_->mtx_);
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
        if(ret != GST_FLOW_OK){
          std::cerr << "Failed to push buffer: " << ret << std::endl;
        }
        gst_buffer_unref(gst_buffer);
      }
      usleep(33333);
    }
    std::cout << "Capture thread exiting" << std::endl;
  }

  bool init_wbrtc_stream(){
    pipeline_ = gst_pipeline_new("pipeline");
    appsrc_ = gst_element_factory_make("appsrc", "appsrc");
    GstElement *jpeg_dec = gst_element_factory_make("jpegdec", "jpeg_dec");
    GstElement *queue = gst_element_factory_make("queue", "queue");
    GstElement *videoconvert = gst_element_factory_make("videoconvert", "convert");
    GstElement *capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
    GstElement *enc = gst_element_factory_make("mppvp8enc", "encoder");
    GstElement *rtppay = gst_element_factory_make("rtpvp8pay", "pay");
    webrtcbin_ = gst_element_factory_make("webrtcbin", "sendrecv");

    if(!pipeline_){
      std::cerr << "Pipeline could not be created" << std::endl;
      return false;
    }
    if(!appsrc_){
      std::cerr << "Appsrc element could not be created" << std::endl;
      return false;
    }
    if(!jpeg_dec){
      std::cerr << "Jpegdec element could not be created" << std::endl;
      return false;
    }
    if(!queue){
      std::cerr << "Queue element could not be created" << std::endl;
      return false;
    }
    if(!videoconvert){
      std::cerr << "Videoconvert element could not be created" << std::endl;
      return false;
    }
    if(!capsfilter){
      std::cerr << "Caps filter element could not be created" << std::endl;
      return false;
    }
    if(!enc){
      std::cerr << "Encoder element could not be created" << std::endl;
      return false;
    }
    if(!rtppay){
      std::cerr << "RTP pay element could not be created" << std::endl;
      return false;
    }
    if(!webrtcbin_){
      std::cerr << "Webrtcbin element could not be created" << std::endl;
      return false;
    }

    GstCaps* appsrc_caps = gst_caps_new_simple("image/jpeg",
      "width", G_TYPE_INT, WIDTH,
      "height", G_TYPE_INT, HEIGHT,
      nullptr);
    g_object_set(appsrc_, "caps", appsrc_caps, "is-live", TRUE, "format", GST_FORMAT_TIME,"max-buffers", 200, nullptr);
    gst_caps_unref(appsrc_caps);

    GstCaps *video_caps = gst_caps_new_simple("video/x-raw",
      "format", G_TYPE_STRING, "I420",
      "width", G_TYPE_INT, WIDTH,
      "height", G_TYPE_INT, HEIGHT,
      nullptr);
    
    g_object_set(capsfilter, "caps", video_caps, nullptr);
    gst_caps_unref(video_caps);

    g_object_set(webrtcbin_, "stun-server", STUN_SERVER, nullptr);

    gst_bin_add_many(GST_BIN(pipeline_), appsrc_, jpeg_dec, queue, videoconvert, capsfilter, enc, rtppay, webrtcbin_, nullptr);

    if(!gst_element_link_many(appsrc_, jpeg_dec, queue, videoconvert, capsfilter, enc, rtppay, webrtcbin_, nullptr)){
      std::cerr << "Elements could not be linked" << std::endl;
      gst_object_unref(pipeline_);
      return false;
    }

    GstPad *rtp_src_pad = gst_element_get_static_pad(rtppay, "src");
    GstPad *webrtc_sink_pad = gst_element_request_pad_simple(webrtcbin_, "sink_%u");
    gst_pad_link(rtp_src_pad, webrtc_sink_pad);
    gst_object_unref(rtp_src_pad);
    gst_object_unref(webrtc_sink_pad);

    g_signal_connect(webrtcbin_, "on-negotiation-needed", G_CALLBACK(on_negotiation_needed), &ws_);
    g_signal_connect(webrtcbin_, "on-ice-candidate", G_CALLBACK(on_ice_candidate), &ws_);
  
    GstStateChangeReturn ret;
    ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if(ret == GST_STATE_CHANGE_FAILURE){
      std::cerr << "Failed to start pipeline" << std::endl;
      gst_object_unref(pipeline_);
      return false;
    }

    std::cout << "GStreamer pipeline set to playing" << std::endl;
    return true;
  }

  void listen_for_signaling_messages(){
    std::cout << "listen signaling message" << std::endl;
    try{
      while(!is_exit_.load()){
        if(!running_.load()){
          usleep(100000);
          continue;
        }
        beast::flat_buffer buffer;
        ws_.read(buffer);
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
        if(type == "offer"){
          std::cout << "Received offer" << std::endl;
          std::string sdp = obj["sdp"].asString();

          GstSDPMessage *sdp_message;
          gst_sdp_message_new_from_text(sdp.c_str(), &sdp_message);
          GstWebRTCSessionDescription *offer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp_message);
          GstPromise *promise = gst_promise_new_with_change_func(on_set_remote_description, &ws_, NULL);
          g_signal_emit_by_name(webrtcbin_, "set-remote-description", offer, promise);
          gst_webrtc_session_description_free(offer);

          std::cout << "Setting remote description" << std::endl;
        }else if (type == "candidate"){
          std::cout << "Received ICE candidate" << std::endl;

          Json::Value ice = obj["ice"];
          std::string candidate = ice["candidate"].asString();
          guint sdpMLineIndex = ice["sdpMLineIndex"].asUInt();
          g_signal_emit_by_name(webrtcbin_, "add-ice-candidate", sdpMLineIndex, candidate.c_str());
          
          std::cout << "Added ICE candidate" << std::endl;
        }
      }
    }catch(...){
      std::cerr << "Error in listen_for_signaling_messages: " << std::endl;
    }
    std::cout << "Signaling thread exiting" << std::endl;
  }

private:
  std::string host_;
  std::string port_;
  net::io_context io_context_;
  wsts ws_;
  SharedMemory *shm_;
  GstElement* appsrc_;
  GMainLoop* loop_;
  GstElement *pipeline_;
  GThread* thread_;
  std::thread connect_thread_;
  std::thread listen_thread_;
  std::atomic<bool> is_exit_{false};
  std::atomic<bool> running_{false};
  static GstElement *webrtcbin_;
};

GstElement *VideoStream::webrtcbin_ = nullptr;

void VideoStream::on_set_remote_description(GstPromise *promise, gpointer user_data){
  std::cout << "Remote description set, creating answer" << std::endl;

  wsts* ws = static_cast<wsts*>(user_data);
  GstPromise *answer_promise = gst_promise_new_with_change_func(on_answer_created, ws, NULL);
  g_signal_emit_by_name(webrtcbin_, "create-answer", nullptr, answer_promise);
}

void VideoStream::on_answer_created(GstPromise *promise, gpointer user_data){
  std::cout << "Answer created" << std::endl;

  wsts* ws = static_cast<wsts*>(user_data);
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

  std::cout << "Local description set and answer sent" << std::endl;

  gst_webrtc_session_description_free(answer);
}

void VideoStream::on_negotiation_needed(GstElement *webrtc, gpointer user_data){
  std::cout << "Negotiation needed" << std::endl;
}

void VideoStream::on_ice_candidate(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data){
  std::cout << "Sending ICE candidate" << std::endl;

  wsts* ws = static_cast<wsts*>(user_data);
  send_ice_candidate_message(*ws, mlineindex, candidate);
}

void VideoStream::send_ice_candidate_message(wsts& ws, guint mlineindex, gchar *candidate){
  std::cout << "Sending ICE candidate" << std::endl;

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