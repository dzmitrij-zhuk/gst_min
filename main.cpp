#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <glib-object.h>
#include <glib.h>
#include <gst/gst.h>
#include <gst/gstbin.h>
#include <gst/gstcaps.h>
#include <gst/gstcompat.h>
#include <gst/gstdebugutils.h>
#include <gst/gststreams.h>
#include <gst/gstutils.h>
#include <gstreamer-1.0/gst/gstelement.h>
#include <gstreamer-1.0/gst/gstpad.h>
#include <thread>

#include "log.h"
GstElement *pipeline;

std::atomic<bool> stop_dumping = false;

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data) {
  GMainLoop *loop = (GMainLoop *)data;

  switch (GST_MESSAGE_TYPE(msg)) {

  case GST_MESSAGE_EOS:
    LOG_INFO("Bus: end of stream");
    g_main_loop_quit(loop);
    break;

  case GST_MESSAGE_ERROR: {
    gchar *debug;
    GError *error;

    gst_message_parse_error(msg, &error, &debug);

    LOG_ERROR(std::format("Bus: error with msg <{}>, debug <{}>", error->message, debug));
    g_error_free(error);
    g_free(debug);

    g_main_loop_quit(loop);
    break;
  }
  case GST_MESSAGE_STATE_CHANGED: {
    GstState old_state, new_state;

    gst_message_parse_state_changed(msg, &old_state, &new_state, NULL);
    LOG_INFO(std::format("Element {} changed state from {} to {}.",
                                                        GST_OBJECT_NAME(msg->src), 
                                                        gst_element_state_get_name(old_state),
                                                        gst_element_state_get_name(new_state)));
    break;
  }
  default:
    break;
  }

  return TRUE;
}

void dump(const boost::system::error_code &, boost::asio::steady_timer *t) {
  if (!stop_dumping) {
    LOG_INFO("Dumping pipeline");
    gst_debug_bin_to_dot_file(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL,
                              "test");
    t->expires_at(t->expiry() + boost::asio::chrono::seconds(5));
    t->async_wait(std::bind(dump, std::placeholders::_1, t));
  } else {
    LOG_INFO("Exiting the timer");
  }
}

void dumper_timer() {
  LOG_INFO("Starting the dumper");
  boost::asio::io_context io;
  boost::asio::steady_timer t(io, std::chrono::seconds(5));
  t.async_wait(std::bind(dump, std::placeholders::_1, &t));
  io.run();
}

/*******************************************************************/

static void onPadAdded(GstElement *element, GstPad *pad, GstElement *sink) {
    GstPad *sink_pad = gst_element_get_static_pad(sink, "sink");
    gst_pad_link(pad, sink_pad);
    gst_object_unref(sink_pad);
}

GstElement* createAndAddItem(const gchar* plugin, const gchar* name){
    auto res = gst_element_factory_make(plugin, name);
    if (!res){
      LOG_ERROR(std::format("Item <{}> with name <{}> can't be created!", plugin, name));
    }
    if(!gst_bin_add(GST_BIN(pipeline), res)){
      LOG_ERROR(std::format("Item <{}> with name <{}> can't be added to pipline!", plugin, name));
    }
    return res;
}

/*******************************************************************/

int main(int argc, char *argv[]) {
  Logger::getInstance().setLogFile("logfile.log");
  GMainLoop *loop;
  GstBus *bus;
  guint bus_watch_id;
  gst_init(&argc, &argv);
  loop = g_main_loop_new(NULL, FALSE);
  LOG_INFO("Logger and gst initialized");

  pipeline = gst_pipeline_new("pipeline");
  if (!pipeline){
      LOG_ERROR("Item plugin can't be created!");
      return(-1);
  }
  auto source = createAndAddItem("souphttpsrc", "source");
  auto queue_src = createAndAddItem("queue2", "queue_src");
  auto decodebin = createAndAddItem("decodebin", "decodebin");
  ////// VIDEO
  auto queue_video_decodebin = createAndAddItem("queue", "queue_video_decodebin");
  auto videotestsrc = createAndAddItem("videotestsrc", "videotestsrc");
  auto fallbackswitch_video = createAndAddItem("fallbackswitch", "fallbackswitch_video");
  auto video_convert = createAndAddItem("videoconvert", "video_convert");
  auto queue_video_encoder = createAndAddItem("queue2", "queue_video_encoder");
  auto x264enc = createAndAddItem("x264enc", "x264enc");
  auto h264parse = createAndAddItem("h264parse", "h264parse");
  auto queue_video_output = createAndAddItem("queue", "queue_video_output");
  //////////////////
  ////// AUDIO
  auto queue_audio_decodebin = createAndAddItem("queue", "queue_audio_decodebin");
  auto audio_convert = createAndAddItem("audioconvert", "audio_convert");
  auto audio_resample = createAndAddItem("audioresample", "audio_resample");
  auto faac = createAndAddItem("faac", "faac");
  auto aacparse = createAndAddItem("aacparse", "aacparse");
  auto queue_audio_output = createAndAddItem("queue", "queue_audio_output");
  //////////////////
  auto mpegtsmux = createAndAddItem("mpegtsmux", "mpegtsmux");
  auto hlssink = createAndAddItem("hlssink", "hlssink");
  LOG_INFO("All item created");



  ////// CONFIGURE
  bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  bus_watch_id = gst_bus_add_watch(bus, bus_call, loop);
  gst_object_unref(bus);
  
  g_object_set(source, "location", "http://10.10.20.215:8080/hls/encoding/754/1510/playlist.m3u8", NULL);
  g_signal_connect(decodebin, "pad-added", G_CALLBACK(onPadAdded), queue_video_decodebin);
  g_signal_connect(decodebin, "pad-added", G_CALLBACK(onPadAdded), queue_audio_decodebin);
  g_object_set(fallbackswitch_video, "timeout", 10000000000, NULL);
  //////////////////



  ////// LINK
  if(!gst_element_link_many(source, queue_src, decodebin, NULL)){
      LOG_ERROR("Items <source, queue_src, decodebin> can't be linked!");
      return -1;
  }
  if(!gst_element_link_many(queue_video_decodebin, fallbackswitch_video, video_convert, queue_video_encoder, x264enc, h264parse, queue_video_output, NULL)){
      LOG_ERROR("Items <queue_video_decodebin, video_convert, queue_video_encoder, x264enc, h264parse, queue_video_output> can't be linked!");
      return -1;
  }
  if(!gst_element_link(videotestsrc, fallbackswitch_video)){
      LOG_ERROR("Items <source, queue_src, decodebin> can't be linked!");
      return -1;
  }
  if(!gst_element_link_many(queue_audio_decodebin, audio_convert, audio_resample, faac, aacparse, queue_audio_output, NULL)){
      LOG_ERROR("Items <queue_audio_decodebin, audio_convert, audio_resample, faac, aacparse, queue_audio_output> can't be linked!");
      return -1;
  }
  if(!gst_element_link(queue_video_output, mpegtsmux)){
      LOG_ERROR("Items <source, queue_src, decodebin> can't be linked!");
      return -1;
  }
  if(!gst_element_link(queue_audio_output, mpegtsmux)){
      LOG_ERROR("Items <source, queue_src, decodebin> can't be linked!");
      return -1;
  }
  if(!gst_element_link(mpegtsmux, hlssink)){
      LOG_ERROR("Items <source, queue_src, decodebin> can't be linked!");
      return -1;
  }
  //////////////////
  LOG_INFO("All items linked");

  LOG_INFO("Start pipeline");
  gst_element_set_state(pipeline, GST_STATE_PLAYING);

  std::thread dumper(dumper_timer);
  LOG_INFO("Start main loop");
  g_main_loop_run(loop);
  LOG_INFO("Stop main loop");

  stop_dumping = true;
  LOG_INFO("Stop pipeline");
  gst_element_set_state(pipeline, GST_STATE_NULL);
  dumper.join();
  gst_object_unref(GST_OBJECT(pipeline));
  g_source_remove(bus_watch_id);
  g_main_loop_unref(loop);

  return 0;
}