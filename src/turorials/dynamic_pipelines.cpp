#include <gst/gst.h>

/* Structure to contain all our information, so we can pass it to callbacks */
// 大部分应用程序需要使用回调函数，为了便于处理将所有数据组织成一个结构体
typedef struct _CustomData {
  GstElement *pipeline;
  GstElement *source;
  GstElement *convert;
  GstElement *resample;
  GstElement *sink;

  GstElement *videoconvert;
  GstElement *videosink;
} CustomData;

/* Handler for the pad-added signal */
static void pad_added_handler (GstElement *src, GstPad *pad, CustomData *data);

int main(int argc, char *argv[]) {
  CustomData data;
  GstBus *bus;
  GstMessage *msg;
  GstStateChangeReturn ret;
  gboolean terminate = FALSE;

  /* Initialize GStreamer */
  gst_init (&argc, &argv);

  /* Create the elements */
  data.source = gst_element_factory_make ("uridecodebin", "source");
  // Playback中的bin插件它在内部实例化所有需要的elements(source, demuxers和decoders)
  // 将URI解码成裸音频流和/或裸音频流 
  data.convert = gst_element_factory_make ("audioconvert", "convert");
  // audioconvert是一个非常有用的插件，它能转换不同的音频格式
  data.resample = gst_element_factory_make ("audioresample", "resample");
  // audioresample是一个非常有用的插件，它能够转换不同的音频采样率
  data.sink = gst_element_factory_make ("autoaudiosink", "sink");

  data.videoconvert = gst_element_factory_make ("videoconvert", "videoconvert");
  data.videosink = gst_element_factory_make ("autovideosink", "videosink");


  /* Create the empty pipeline */
  data.pipeline = gst_pipeline_new ("test-pipeline");

  if (!data.pipeline || !data.source || !data.convert || !data.resample || !data.sink || !data.videoconvert || !data.videosink) {
    g_printerr ("Not all elements could be created.\n");
    return -1;
  }

  /* Build the pipeline. Note that we are NOT linking the source at this
   * point. We will do it later. */
  gst_bin_add_many (GST_BIN (data.pipeline), data.source, data.convert, data.resample, data.sink, data.videoconvert, data.videosink, NULL);
  if (!gst_element_link_many (data.convert, data.resample, data.sink, NULL)) {
    g_printerr ("Elements could not be linked.\n");
    gst_object_unref (data.pipeline);
    return -1;
  }
  if (!gst_element_link_many (data.videoconvert, data.videosink, NULL)) {
    g_printerr ("Elements could not be linked.\n");
    gst_object_unref (data.pipeline);
    return -1;
  }

  /* Set the URI to play */
  g_object_set (data.source, "uri", "https://www.freedesktop.org/software/gstreamer-sdk/data/media/sintel_trailer-480p.webm", NULL);

  /* Connect to the pad-added signal */
  g_signal_connect (data.source, "pad-added", G_CALLBACK (pad_added_handler), &data);
  // GSignals是GStreamer的一个重点，
  // 将在某些事件发生的时候以回调的方式通知
  // 这些信号以名字属性区分，每个GObject都有它自己的信号
  // 向回调传递一个用户数据指针data
  // 当source element最终获取到足够的信息从而开始生成数据的时候
  // 它将创建source pads并且出发pad-added信号，这时将调用信号连接的回调函数 

  /* Start playing */
  ret = gst_element_set_state (data.pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr ("Unable to set the pipeline to the playing state.\n");
    gst_object_unref (data.pipeline);
    return -1;
  }
  // 无法从NULL直接改为PLAYING，
  // 必须经过READY和PAUSED两个中间态
  // 假如你将pipeline设置为PLAYING, GStreamer将自动进行中间转换

  /* Listen to the bus */
  bus = gst_element_get_bus (data.pipeline);
  do {
    msg = gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE,
      (GstMessageType)(GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    /* Parse message */
    if (msg != NULL) {
      GError *err;
      gchar *debug_info;

      switch (GST_MESSAGE_TYPE (msg)) {
        case GST_MESSAGE_ERROR:
          gst_message_parse_error (msg, &err, &debug_info);
          g_printerr ("Error received from element %s: %s\n", GST_OBJECT_NAME (msg->src), err->message);
          g_printerr ("Debugging information: %s\n", debug_info ? debug_info : "none");
          g_clear_error (&err);
          g_free (debug_info);
          terminate = TRUE;
          break;
        case GST_MESSAGE_EOS:
          g_print ("End-Of-Stream reached.\n");
          terminate = TRUE;
          break;
        case GST_MESSAGE_STATE_CHANGED:
          /* We are only interested in state-changed messages from the pipeline */
          if (GST_MESSAGE_SRC (msg) == GST_OBJECT (data.pipeline)) {
            GstState old_state, new_state, pending_state;
            gst_message_parse_state_changed (msg, &old_state, &new_state, &pending_state);
            g_print ("Pipeline state changed from %s to %s:\n",
                gst_element_state_get_name (old_state), gst_element_state_get_name (new_state));
          }
          break;
        default:
          /* We should not reach here */
          g_printerr ("Unexpected message received.\n");
          break;
      }
      gst_message_unref (msg);
    }
  } while (!terminate);

  /* Free resources */
  gst_object_unref (bus);
  gst_element_set_state (data.pipeline, GST_STATE_NULL);
  gst_object_unref (data.pipeline);
  return 0;
}

/* This function will be called by the pad-added signal */
static void pad_added_handler (GstElement *src, GstPad *new_pad, CustomData *data) {
  // GstPad *sink_pad = gst_element_get_static_pad (data->convert, "sink");
  GstPad *sink_pad = nullptr;
  // 从CustomData中获取autoaudioconvertelement，
  // 并使用gst_element_get_static_pad()获取它的sink pad
  // 要与new_pad连接的pad
  GstPadLinkReturn ret;
  GstCaps *new_pad_caps = NULL;
  GstStructure *new_pad_struct = NULL;
  const gchar *new_pad_type = NULL;

  g_print ("Received new pad '%s' from '%s':\n", GST_PAD_NAME (new_pad), GST_ELEMENT_NAME (src));

  /* If our converter is already linked, we have nothing to do here */
  if (gst_pad_is_linked (sink_pad)) {
    g_print ("We are already linked. Ignoring.\n");
    goto exit;
  }
  // 避免我们尝试将new_pad与一个已经连接了的element连接

  /* Check the new pad's type */
  new_pad_caps = gst_pad_get_current_caps (new_pad);
  // gst_pad_get_current_caps()检查当前pad的capabilities(当前输出的数据类型)
  // 用户可以使用gst_pad_query_caps()获取当前pad支持的所有caps
  new_pad_struct = gst_caps_get_structure (new_pad_caps, 0);
  // 使用gst_caps_get_structure()检索pad的第一个GstStructure
  new_pad_type = gst_structure_get_name (new_pad_struct);
  // if (!g_str_has_prefix (new_pad_type, "audio/x-raw")) {
  //   g_print ("It has type '%s' which is not raw audio. Ignoring.\n", new_pad_type);
  //   goto exit;
  // }

  // /* Attempt the link */
  // ret = gst_pad_link (new_pad, sink_pad);
  // if (GST_PAD_LINK_FAILED (ret)) {
  //   g_print ("Type is '%s' but link failed.\n", new_pad_type);
  // } else {
  //   g_print ("Link succeeded (type '%s').\n", new_pad_type);
  // }
  // gst_pad_link()尝试连接两个pads，连接顺序必须是source->sink
  // 并且两个pads必须属于同一个bin/pipeline中的elements
  if (g_str_has_prefix (new_pad_type, "audio/x-raw")) {
    sink_pad = gst_element_get_static_pad (data->convert, "sink");
  } else if (g_str_has_prefix (new_pad_type, "video/x-raw")) {
    sink_pad = gst_element_get_static_pad (data->videoconvert, "sink");
  } else {
    g_print ("It has type '%s' which is not raw audio or video. Ignoring.\n", new_pad_type);
    goto exit;
  }

  if (gst_pad_is_linked (sink_pad)) {
    g_print ("We are already linked. Ignoring.\n");
    goto exit;
  }

  ret = gst_pad_link (new_pad, sink_pad);
  if (GST_PAD_LINK_FAILED (ret)) {
    g_print ("Type is '%s' but link failed.\n", new_pad_type);
  } else {
    g_print ("Link succeeded (type '%s').\n", new_pad_type);
  }

exit:
  /* Unreference the new pad's caps, if we got them */
  if (new_pad_caps != NULL)
    gst_caps_unref (new_pad_caps);

  /* Unreference the sink pad */
  gst_object_unref (sink_pad);
}