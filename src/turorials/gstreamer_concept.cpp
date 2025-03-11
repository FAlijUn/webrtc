#include <gst/gst.h>

// 手动构建一条pipeline：初始化每一个element并将它们连接起来

int tutorial_main (int argc, char *argv[])
{
  GstElement *pipeline, *source, *sink;
  GstBus *bus;
  GstMessage *msg;
  GstStateChangeReturn ret;

  /* Initialize GStreamer */
  gst_init (&argc, &argv);

  /* Create the elements */
  source = gst_element_factory_make ("videotestsrc", "source");
  sink = gst_element_factory_make ("autovideosink", "sink");
  // videotestsrc是一个按照制定pattern生成测试视频source element(它将生产数据)
  // autovideosink是一个在窗口中播放它接收到的图像的sink element(它将消费数据)
  // GStreamer包含有很多视频sink element，具体取决于操作系统，它们能够处理不同的图像格式
  // autovideosink将自动选择其中一个并实例话化，所以用户不需要担心平台兼容性问题

  /* Create the empty pipeline */
  pipeline = gst_pipeline_new ("test-pipeline");
  // 创建一个pipeline，GStreamer中的所有元素在使用之前通常必须包含在一条pipeline中
  // pipeline将负责一些时钟和消息功能
  // 一条pipeline也是一个特殊的bin，被用来包含其他elements


  if (!pipeline || !source || !sink) {
    g_printerr ("Not all elements could be created.\n");
    return -1;
  }

  /* Build the pipeline */
  gst_bin_add_many (GST_BIN (pipeline), source, sink, NULL);
  // gst_bin_add_many()来将elements添加到pipeline中(注意对pipeline的GST_BIN()映射)
  // 这个函数接受一个要被添加到pipeline中的element列表，因为不确定列表长度所以需要以NULL结尾
  // 添加单个element可以使用get_bin_add()

  if (gst_element_link (source, sink) != TRUE) {
    g_printerr ("Elements could not be linked.\n");
    gst_object_unref (pipeline);
    return -1;
  }

  /* Modify the source's properties */
  g_object_set (source, "pattern", 0, NULL);

  /* Start playing */
  ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr ("Unable to set the pipeline to the playing state.\n");
    gst_object_unref (pipeline);
    return -1;
  }

  /* Wait until error or EOS */
  bus = gst_element_get_bus (pipeline);
  msg =
      gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE,
        (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
  // GstMessage是一个非常通用的结构，
  // 它可以传递几乎任何类型的信息
  // 同时，GStreamer为每种消息提供了一系列解析函数

  /* Parse message */
  if (msg != NULL) {
    GError *err;
    gchar *debug_info;

    switch (GST_MESSAGE_TYPE (msg)) {
      case GST_MESSAGE_ERROR:
        gst_message_parse_error (msg, &err, &debug_info);
        g_printerr ("Error received from element %s: %s\n",
            GST_OBJECT_NAME (msg->src), err->message);
        g_printerr ("Debugging information: %s\n",
            debug_info ? debug_info : "none");
        g_clear_error (&err);
        g_free (debug_info);
        break;
      case GST_MESSAGE_EOS:
        g_print ("End-Of-Stream reached.\n");
        break;
      default:
        /* We should not reach here because we only asked for ERRORs and EOS */
        g_printerr ("Unexpected message received.\n");
        break;
    }
    gst_message_unref (msg);
  }
  // GStreamer bus它是负责将element生成的GstMessages按顺序交付给应用程序和应用程序线程
  // 因此GStreamer实际是在其他的线程中处理媒体流的对象

  /* Free resources */
  gst_object_unref (bus);
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (pipeline);
  return 0;
}

int main (int argc, char *argv[])
{
  return tutorial_main (argc, argv);
}

