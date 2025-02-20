#include <gst/gst.h>
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
#include <gst/gst.h>
#include <gst/app/app.h>



namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;


#define STUN_SERVER "stun://stun.l.google.com:19302"
#define SERVER_PORT 8000

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
GMainLoop *loop;
GstElement *pipeline, *webrtcbin;
int device_fd;

static void capture_frames(CaptureArgs* args) {
  int fd = args->fd;
  V4L2Buffer* buffers = args->buffers;
  GstClockTime timestamp = 0;

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

    GST_BUFFER_PTS(gst_buffer) = timestamp;
    GST_BUFFER_DURATION(gst_buffer) = gst_util_uint64_scale(1, GST_SECOND, 30);
    timestamp += GST_BUFFER_DURATION(gst_buffer);

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

void handle_websocket_session(tcp::socket socket)
{
  try
  {
    websocket::stream<tcp::socket> ws{std::move(socket)};
    ws.accept();

    std::cout << "WebSocket connection accepted" << std::endl;

    device_fd = open("/dev/video0", O_RDWR);
    if (device_fd == -1) {
      std::cerr << "Failed to open device" << std::endl;
      return;
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
      return;
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
      return;
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
          return;
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
        return;
      }
    }
  
    // 启动视频流
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(device_fd, VIDIOC_STREAMON, &type) == -1) {
      std::cerr << "Failed to start streaming" << std::endl;
      close(device_fd);
      return;
    }
  
    GstStateChangeReturn ret;
    GError *error = NULL;

    pipeline = gst_pipeline_new("pipeline");
    appsrc = gst_element_factory_make("appsrc", "appsrc");
    GstElement *videoconvert = gst_element_factory_make("videoconvert", "convert");
    GstElement *capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
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
      g_printerr("Appsrc element could not be created.\n");
      return;
    }
    if(!videoconvert)
    {
      g_printerr("Videoconvert element could not be created.\n");
      return;
    }
    if(!capsfilter)
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

    GstCaps *caps1 = gst_caps_new_simple("video/x-raw",
                                    "format", G_TYPE_STRING, "YUY2",
                                    "width", G_TYPE_INT, 640,
                                    "height", G_TYPE_INT, 480,
                                    "framerate", GST_TYPE_FRACTION, 30, 1,
                                    NULL);

    g_object_set(appsrc, "caps", caps1, "is-live", TRUE, "format", GST_FORMAT_TIME, NULL);
    gst_caps_unref(caps1);

    GstCaps *caps2 = gst_caps_new_simple("video/x-raw",
                                      "format", G_TYPE_STRING, "I420",
                                      "width", G_TYPE_INT, 640,
                                      "height", G_TYPE_INT, 480,
                                      NULL);

    g_object_set(capsfilter, "caps", caps2, NULL);
    gst_caps_unref(caps2);


    g_object_set(webrtcbin, "stun-server", "stun://stun.l.google.com:19302", NULL);
    g_object_set(vp8enc, "deadline", 1, NULL);

    gst_bin_add_many(GST_BIN(pipeline), appsrc, videoconvert, capsfilter, queue, vp8enc, rtpvp8pay, webrtcbin, NULL);
    
    if (!gst_element_link_many(appsrc, videoconvert, capsfilter, queue, vp8enc, rtpvp8pay, NULL))
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

    // 启动采集线程
    CaptureArgs* args = new CaptureArgs{device_fd, buffers};
    GThread* thread = g_thread_new("capture", (GThreadFunc)capture_frames, args);

  
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
      g_printerr("Unable to set the pipeline to the playing state.\n");
      gst_object_unref(pipeline);
      return;
    }

    std::cout << "GStreamer pipeline set to playing" << std::endl;

    for (;;)
    {
      beast::flat_buffer buffer;
      ws.read(buffer);

      auto text = beast::buffers_to_string(buffer.data());

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

int main(int argc, char *argv[])
{
  gst_init(&argc, &argv);
  std::cout << "Starting WebRTC server" << std::endl;
  loop = g_main_loop_new(NULL, FALSE);

  std::thread server_thread(start_server);
  g_main_loop_run(loop);
  server_thread.join();

  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);
  g_main_loop_unref(loop);
  close(device_fd);
  std::cout << "WebRTC server stopped" << std::endl;
  return 0;
}