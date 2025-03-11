#include <gst/gst.h>
#include <thread>

int tutorial_main (int argc, char *argv[])
{
  GstElement *pipeline;
  GstBus *bus;
  GstMessage *msg;

  /* Initialize GStreamer */
  gst_init (&argc, &argv); // 初始化所有内部结构，检查所有可用的插件

  /* Build the pipeline */
  pipeline =
      gst_parse_launch
      ("playbin uri=https://gstreamer.freedesktop.org/data/media/sintel_trailer-480p.webm",
      NULL);
  
  // gst_parse_launch，媒体将source元素，生产者传播到sink汇元素，所有相互连接的元素的集合称为“管道”

  /* Start playing */
  gst_element_set_state (pipeline, GST_STATE_PLAYING);
  // playbin 构建一个playbin的单个元素组成的管道
  std::this_thread::sleep_for(std::chrono::seconds(2));
  gst_element_set_state (pipeline, GST_STATE_PAUSED);

  /* Wait until error or EOS */
  bus = gst_element_get_bus (pipeline);
  msg =gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

  /* See next tutorial for proper error message handling/parsing */
  if (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_ERROR) {
    g_printerr ("An error occurred! Re-run with the GST_DEBUG=*:WARN "
        "environment variable set for more details.\n");
  }

  // 执行将在媒体到达其末端（EOS）或遇到错误时结束（尝试关闭视频窗口，或拔掉网线）

  /* Free resources */
  gst_message_unref (msg);
  gst_object_unref (bus);
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (pipeline);
  return 0;
}

int
main (int argc, char *argv[]){
  return tutorial_main (argc, argv);
}

