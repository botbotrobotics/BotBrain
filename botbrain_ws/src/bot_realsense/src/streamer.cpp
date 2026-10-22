#include "bot_realsense/streamer.hpp"

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/video/video.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <rclcpp/logging.hpp>

#include "depth_video.hpp"

namespace bot_realsense {
namespace {

template <auto Free>
struct Deleter {
  template <class T>
  void operator()(T* p) const {
    Free(p);
  }
};
template <class T, auto Free>
using Owned = std::unique_ptr<T, Deleter<Free>>;

void destroy_source(GSource* s) {
  g_source_destroy(s);
  g_source_unref(s);
}
void release_pool(GstBufferPool* p) {
  gst_buffer_pool_set_active(p, FALSE);
  gst_object_unref(p);
}

template <class T>
using GstRef = Owned<T, gst_object_unref>;
using CapsRef = Owned<GstCaps, gst_caps_unref>;
using MessageRef = Owned<GstMessage, gst_message_unref>;
using ErrorRef = Owned<GError, g_error_free>;
using SourceRef = Owned<GSource, destroy_source>;
using PoolRef = Owned<GstBufferPool, release_pool>;
using ServerRef = Owned<GstRTSPServer, g_object_unref>;
using LoopRef = Owned<GMainLoop, g_main_loop_unref>;
using ContextRef = Owned<GMainContext, g_main_context_unref>;

// One RTSP mount. Its encoder pipeline (a gst-rtsp-server media) exists only between media-configure and unprepared.
struct Mount {
  // Set by the Streamer constructor, read-only afterwards.
  std::string name;
  std::string launch;
  GstVideoInfo info{};
  PoolRef pool;  // bounds frames in flight: no free buffer means the encoder is busy
  int fps_in = 0, fps_out = 0;
  rclcpp::Logger logger = rclcpp::get_logger("bot_realsense");

  int skip_acc = 0;  // owned by the pushing thread

  std::atomic<bool> wanted{false};
  std::atomic<uint64_t> pushed{0}, dropped{0};

  std::mutex mu;  // guards the fields below; never held across a push or a GStreamer state change
  const GstRTSPMedia* media = nullptr;  // identity of the live media, not a ref
  GstRef<GstElement> src;
  GstClockTime last_pts = GST_CLOCK_TIME_NONE;
  std::condition_variable unprepared;
};

// gst-rtsp-server tears media down on its own threads, maybe after the Streamer is gone: closures own their mount.
using MountRef = std::shared_ptr<Mount>;

Mount& mount_of(gpointer closure_data) { return **static_cast<MountRef*>(closure_data); }

void release_mount_ref(gpointer closure_data, GClosure*) {
  std::unique_ptr<MountRef> owned(static_cast<MountRef*>(closure_data));
}

void connect_mount(gpointer instance, const char* signal, GCallback handler, const MountRef& m) {
  g_signal_connect_data(instance, signal, handler, std::make_unique<MountRef>(m).release(), release_mount_ref,
                        GConnectFlags(0));
}

bool element_exists(const char* name) { return GstRef<GstElementFactory>(gst_element_factory_find(name)) != nullptr; }

// The nvvideo4linux2 plugin ships on every Jetson, including the NVENC-less Orin Nano, so only a real encode proves NVENC.
bool nvenc_encodes() {
  if (!element_exists("nvv4l2h264enc") || !element_exists("nvvidconv")) return false;
  GError* err = nullptr;
  GstRef<GstElement> p(gst_parse_launch(
      "videotestsrc num-buffers=2 ! video/x-raw,format=NV12,width=320,height=240 ! nvvidconv ! "
      "video/x-raw(memory:NVMM),format=NV12 ! nvv4l2h264enc ! fakesink",
      &err));
  ErrorRef owned_err(err);
  if (!p) return false;
  gst_element_set_state(p.get(), GST_STATE_PLAYING);
  GstRef<GstBus> bus(gst_element_get_bus(p.get()));
  MessageRef msg(
      gst_bus_timed_pop_filtered(bus.get(), 5 * GST_SECOND, GstMessageType(GST_MESSAGE_EOS | GST_MESSAGE_ERROR)));
  gst_element_set_state(p.get(), GST_STATE_NULL);
  return msg && GST_MESSAGE_TYPE(msg.get()) == GST_MESSAGE_EOS;
}

std::string resolve_encoder(const std::string& want) {
  if (want != "auto" && want != "nvenc" && want != "x264")
    throw std::runtime_error("stream_encoder must be auto, nvenc or x264, got '" + want + "'");
  for (const char* e : {"appsrc", "rtph264pay"})
    if (!element_exists(e)) throw std::runtime_error(std::string("GStreamer element missing: ") + e);
  if (want != "x264" && nvenc_encodes()) return "nvv4l2h264enc";
  if (want == "nvenc") throw std::runtime_error("stream_encoder nvenc: nvv4l2h264enc cannot encode on this device");
  if (!element_exists("x264enc")) throw std::runtime_error("GStreamer element missing: x264enc");
  return "x264enc";
}

std::string launch_description(const std::string& encoder, const std::string& caps, int bitrate, int gop) {
  std::string enc;
  if (encoder == "nvv4l2h264enc")
    enc = "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! nvv4l2h264enc control-rate=constant_bitrate bitrate=" +
          std::to_string(bitrate) + " profile=Baseline iframeinterval=" + std::to_string(gop) +
          " idrinterval=" + std::to_string(gop) + " insert-sps-pps=true";
  else
    // threads=1: zerolatency's sliced threads make GStreamer 1.20's rtph264pay repeat SPS/PPS before every
    // IDR slice, which strict decoders read as extra frames.
    enc = "x264enc pass=cbr tune=zerolatency speed-preset=ultrafast threads=1 bitrate=" +
          std::to_string(std::max(1, bitrate / 1000)) + " key-int-max=" + std::to_string(gop) +
          " ! video/x-h264,profile=constrained-baseline";
  return "( appsrc name=src is-live=true format=time max-buffers=2 leaky-type=downstream caps=\"" + caps + "\" ! " +
         enc + " ! rtph264pay name=pay0 pt=96 config-interval=-1 )";
}

void init_mount(Mount& m, const char* name, const std::string& encoder, int w, int h, int fps_in, int fps_out,
                int bitrate, double gop_s, const char* colorimetry, const rclcpp::Logger& logger) {
  m.name = name;
  m.fps_in = fps_in;
  m.fps_out = std::min(fps_out, fps_in);
  m.logger = logger;
  std::string caps = "video/x-raw,format=NV12,width=" + std::to_string(w) + ",height=" + std::to_string(h) +
                     ",framerate=" + std::to_string(m.fps_out) + "/1";
  if (colorimetry) caps += std::string(",colorimetry=") + colorimetry;
  m.launch = launch_description(encoder, caps, bitrate, std::max(1, int(std::lround(gop_s * m.fps_out))));

  CapsRef c(gst_caps_from_string(caps.c_str()));
  gst_video_info_from_caps(&m.info, c.get());
  m.pool.reset(gst_buffer_pool_new());
  GstStructure* config = gst_buffer_pool_get_config(m.pool.get());
  gst_buffer_pool_config_set_params(config, c.get(), guint(GST_VIDEO_INFO_SIZE(&m.info)), 4, 4);
  gst_buffer_pool_set_config(m.pool.get(), config);
  if (!gst_buffer_pool_set_active(m.pool.get(), TRUE)) throw std::runtime_error("cannot allocate stream buffers");
}

void on_media_unprepared(GstRTSPMedia* media, gpointer user) {
  Mount& m = mount_of(user);
  GstRef<GstElement> src;
  {
    std::lock_guard<std::mutex> lock(m.mu);
    if (m.media != media) return;
    src = std::move(m.src);
    m.media = nullptr;
    m.wanted = false;
  }
  m.unprepared.notify_all();
  RCLCPP_INFO(m.logger, "stream %s: last client left, encoder stopped (%lu frames, %lu dropped busy)",
              m.name.c_str(), static_cast<unsigned long>(m.pushed.load()),
              static_cast<unsigned long>(m.dropped.load()));
}

// Shared media: called once per encoder pipeline, when its first client asks for the mount.
void on_media_configure(GstRTSPMediaFactory*, GstRTSPMedia* media, gpointer user) {
  Mount& m = mount_of(user);
  GstRef<GstElement> bin(gst_rtsp_media_get_element(media));
  GstRef<GstElement> src(gst_bin_get_by_name_recurse_up(GST_BIN(bin.get()), "src"));
  connect_mount(media, "unprepared", G_CALLBACK(on_media_unprepared), *static_cast<MountRef*>(user));
  GstRef<GstElement> previous;
  {
    std::lock_guard<std::mutex> lock(m.mu);
    previous = std::move(m.src);
    m.media = media;
    m.src = std::move(src);
    m.last_pts = GST_CLOCK_TIME_NONE;
    m.pushed = 0;
    m.dropped = 0;
    m.wanted = true;
  }
  RCLCPP_INFO(m.logger, "stream %s: client attached, encoder starting", m.name.c_str());
}

// gst-rtsp-server keys shared media by path and query, so a query would start a second encoder on the same mount.
GstRTSPStatusCode reject_query(GstRTSPClient*, GstRTSPContext* ctx, gpointer) {
  return ctx->uri && ctx->uri->query ? GST_RTSP_STS_NOT_FOUND : GST_RTSP_STS_OK;
}

// The pool watch recomputes its timeout only when our context iterates; sessions arrive on other threads, so wake it.
void on_new_session(GstRTSPClient*, GstRTSPSession*, gpointer ctx) {
  g_main_context_wakeup(static_cast<GMainContext*>(ctx));
}

void release_context(gpointer ctx, GClosure*) { g_main_context_unref(static_cast<GMainContext*>(ctx)); }

void on_client_connected(GstRTSPServer*, GstRTSPClient* client, gpointer ctx) {
  g_signal_connect(client, "pre-describe-request", G_CALLBACK(reject_query), nullptr);
  g_signal_connect(client, "pre-setup-request", G_CALLBACK(reject_query), nullptr);
  g_signal_connect_data(client, "new-session", G_CALLBACK(on_new_session),
                        g_main_context_ref(static_cast<GMainContext*>(ctx)), release_context, GConnectFlags(0));
}

// Dispatched by the pool's watch only when a session has stopped sending keep-alives past its timeout.
gboolean expire_sessions(GstRTSPSessionPool* pool, gpointer) {
  gst_rtsp_session_pool_cleanup(pool);
  return G_SOURCE_CONTINUE;
}

GstRTSPFilterResult remove_session(GstRTSPSessionPool*, GstRTSPSession*, gpointer) { return GST_RTSP_FILTER_REMOVE; }
GstRTSPFilterResult remove_client(GstRTSPServer*, GstRTSPClient*, gpointer) { return GST_RTSP_FILTER_REMOVE; }

gboolean quit_loop(gpointer loop) {
  g_main_loop_quit(static_cast<GMainLoop*>(loop));
  return G_SOURCE_REMOVE;
}

// The capture moment on the pipeline's running time, so RTP timestamps follow capture rather than our scheduling.
// An age outside [0, 1 s] means the stamp is on another clock (sim time); then the push moment is used.
GstClockTime capture_running_time(GstElement* src, uint64_t stamp_ns) {
  GstRef<GstClock> clock(gst_element_get_clock(src));
  if (!clock) return GST_CLOCK_TIME_NONE;
  const GstClockTime now = gst_clock_get_time(clock.get()), base = gst_element_get_base_time(src);
  if (now < base) return GST_CLOCK_TIME_NONE;
  const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
  int64_t age = int64_t(wall) - int64_t(stamp_ns);
  if (age < 0 || age > int64_t(GST_SECOND)) age = 0;
  const GstClockTime running = now - base;
  return running > GstClockTime(age) ? running - GstClockTime(age) : 0;
}

template <class Fill>
void send(Mount& m, uint64_t stamp_ns, Fill&& fill) {
  GstRef<GstElement> src;
  GstClockTime pts = GST_CLOCK_TIME_NONE;
  {
    std::lock_guard<std::mutex> lock(m.mu);
    if (!m.src) return;
    pts = capture_running_time(m.src.get(), stamp_ns);
    if (pts == GST_CLOCK_TIME_NONE) return;
    if (m.last_pts != GST_CLOCK_TIME_NONE && pts <= m.last_pts) pts = m.last_pts + 1;
    m.last_pts = pts;
    src.reset(GST_ELEMENT(gst_object_ref(m.src.get())));
  }
  GstBuffer* buf = nullptr;
  GstBufferPoolAcquireParams params{};
  params.flags = GST_BUFFER_POOL_ACQUIRE_FLAG_DONTWAIT;
  if (gst_buffer_pool_acquire_buffer(m.pool.get(), &buf, &params) != GST_FLOW_OK) {
    ++m.dropped;
    return;
  }
  GstMapInfo map;
  gst_buffer_map(buf, &map, GST_MAP_WRITE);
  fill(map.data);
  gst_buffer_unmap(buf, &map);
  GST_BUFFER_PTS(buf) = pts;
  GST_BUFFER_DURATION(buf) = GST_SECOND / GstClockTime(m.fps_out);
  gst_app_src_push_buffer(GST_APP_SRC(src.get()), buf);  // takes the buffer
  ++m.pushed;
}

template <class Fill>
void push(Mount& m, uint64_t stamp_ns, Fill&& fill) {
  if (!m.wanted.load(std::memory_order_relaxed)) return;
  m.skip_acc += m.fps_out;
  if (m.skip_acc < m.fps_in) return;
  m.skip_acc -= m.fps_in;
  send(m, stamp_ns, std::forward<Fill>(fill));
}

// A media awaiting its first frame holds its client's thread and blocks teardown; a blank frame lets it finish.
void send_blank(Mount& m) {
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch()).count();
  send(m, uint64_t(now), [&m](uint8_t* dst) {
    const auto chroma = size_t(GST_VIDEO_INFO_PLANE_OFFSET(&m.info, 1));
    std::memset(dst, 16, chroma);
    std::memset(dst + chroma, 128, size_t(GST_VIDEO_INFO_SIZE(&m.info)) - chroma);
  });
}

void yuyv_to_nv12(const uint8_t* yuyv, size_t stride, const GstVideoInfo& info, uint8_t* dst) {
  const int w = GST_VIDEO_INFO_WIDTH(&info), h = GST_VIDEO_INFO_HEIGHT(&info);
  uint8_t* yp = dst + GST_VIDEO_INFO_PLANE_OFFSET(&info, 0);
  uint8_t* uvp = dst + GST_VIDEO_INFO_PLANE_OFFSET(&info, 1);
  const auto ys = size_t(GST_VIDEO_INFO_PLANE_STRIDE(&info, 0)), uvs = size_t(GST_VIDEO_INFO_PLANE_STRIDE(&info, 1));
  for (int y = 0; y < h; ++y) {
    const uint8_t* s = yuyv + size_t(y) * stride;
    uint8_t* yd = yp + size_t(y) * ys;
    for (size_t x = 0; x < size_t(w); ++x) yd[x] = s[2 * x];
    if (y % 2 == 0) {  // Y0 U Y1 V: the odd bytes are already NV12's interleaved UV
      uint8_t* uv = uvp + size_t(y / 2) * uvs;
      for (size_t x = 0; x < size_t(w); ++x) uv[x] = s[2 * x + 1];
    }
  }
}

void validate(const StreamConfig& c) {
  if (c.mount_base.empty()) throw std::runtime_error("stream mount_base is empty");
  if (c.rtsp_port < 1 || c.rtsp_port > 65535) throw std::runtime_error("stream_rtsp_port out of range");
  if (c.color_w <= 0 || c.color_h <= 0 || c.color_w % 2 || c.color_h % 2)
    throw std::runtime_error("colour stream size must be positive and even");
  if (c.depth_w <= 0 || c.depth_h <= 0 || c.depth_w % 4 || c.depth_h % 4)
    throw std::runtime_error("depth size must be positive and a multiple of 4 (stream is decimated 2x2)");
  if (c.color_fps_in <= 0 || c.color_fps_out <= 0 || c.depth_fps_in <= 0 || c.depth_fps_out <= 0)
    throw std::runtime_error("stream fps must be positive");
  if (c.color_bitrate <= 0 || c.depth_bitrate <= 0 || !(c.gop_s > 0))
    throw std::runtime_error("stream bitrate and gop must be positive");
  if (!(c.depth_min_m > 0 && c.depth_min_m < c.depth_max_m))
    throw std::runtime_error("depth_min_m must be positive and below depth_max_m");
}

}  // namespace

// Reverse member destruction releases every GSource before its context.
struct Streamer::Impl {
  // Set by the constructor, read-only afterwards.
  rclcpp::Logger logger;
  StreamConfig cfg;
  std::string encoder;
  MountRef color = std::make_shared<Mount>(), depth = std::make_shared<Mount>();

  // Owned by the depth pushing thread.
  GreyLut lut{};
  float lut_scale = 0;

  ContextRef ctx;  // run by loop_thread: accepts RTSP connections, expires sessions
  LoopRef loop;
  ServerRef server;
  SourceRef session_watch;
  SourceRef listen;  // owned by the thread calling start()/stop(); set while started
  std::thread loop_thread;

  Impl(const rclcpp::Logger& lg, StreamConfig config);
  ~Impl();
  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;
  Impl(Impl&&) = delete;
  Impl& operator=(Impl&&) = delete;
  void start();
  void stop();
};

Streamer::Impl::Impl(const rclcpp::Logger& lg, StreamConfig config) : logger(lg), cfg(std::move(config)) {
  validate(cfg);
  gst_init(nullptr, nullptr);
  encoder = resolve_encoder(cfg.encoder);
  init_mount(*color, "color", encoder, cfg.color_w, cfg.color_h, cfg.color_fps_in, cfg.color_fps_out,
             cfg.color_bitrate, cfg.gop_s, nullptr, logger);
  // Full range so a <video>/canvas reader gets the exact luma values the browser maps back to metres.
  init_mount(*depth, "depth", encoder, cfg.depth_w / 2, cfg.depth_h / 2, cfg.depth_fps_in, cfg.depth_fps_out,
             cfg.depth_bitrate, cfg.gop_s, "1:4:5:4", logger);

  ctx.reset(g_main_context_new());
  loop.reset(g_main_loop_new(ctx.get(), FALSE));
  server.reset(gst_rtsp_server_new());
  gst_rtsp_server_set_address(server.get(), "127.0.0.1");
  gst_rtsp_server_set_service(server.get(), std::to_string(cfg.rtsp_port).c_str());
  // gst-rtsp-server serves all clients from one thread by default; one awaiting its first frame would stall the rest.
  Owned<GstRTSPThreadPool, g_object_unref> threads(gst_rtsp_server_get_thread_pool(server.get()));
  gst_rtsp_thread_pool_set_max_threads(threads.get(), 8);
  g_signal_connect(server.get(), "client-connected", G_CALLBACK(on_client_connected), ctx.get());
  Owned<GstRTSPMountPoints, g_object_unref> mounts(gst_rtsp_server_get_mount_points(server.get()));
  for (const MountRef& m : {color, depth}) {
    GstRTSPMediaFactory* f = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(f, m->launch.c_str());
    gst_rtsp_media_factory_set_shared(f, TRUE);
    connect_mount(f, "media-configure", G_CALLBACK(on_media_configure), m);
    gst_rtsp_mount_points_add_factory(mounts.get(), ("/" + cfg.mount_base + "/" + m->name).c_str(), f);  // takes f
  }

  // gst-rtsp-server never expires sessions itself: a hung client with its connection open keeps its encoder forever.
  Owned<GstRTSPSessionPool, g_object_unref> pool(gst_rtsp_server_get_session_pool(server.get()));
  session_watch.reset(gst_rtsp_session_pool_create_watch(pool.get()));
  g_source_set_callback(session_watch.get(), G_SOURCE_FUNC(expire_sessions), nullptr, nullptr);
  g_source_attach(session_watch.get(), ctx.get());
  loop_thread = std::thread([this] { g_main_loop_run(loop.get()); });
}

Streamer::Impl::~Impl() {
  stop();
  // Quit from inside the loop: a quit issued before g_main_loop_run starts would be lost.
  SourceRef quit(g_idle_source_new());
  g_source_set_callback(quit.get(), quit_loop, loop.get(), nullptr);
  g_source_attach(quit.get(), ctx.get());
  loop_thread.join();
}

void Streamer::Impl::start() {
  if (listen) return;
  GError* err = nullptr;
  listen.reset(gst_rtsp_server_create_source(server.get(), nullptr, &err));
  ErrorRef owned_err(err);
  if (!listen)
    throw std::runtime_error("RTSP server on 127.0.0.1:" + std::to_string(cfg.rtsp_port) + ": " +
                             (err ? err->message : "unknown error"));
  g_source_attach(listen.get(), ctx.get());
  RCLCPP_INFO(logger, "RTSP rtsp://127.0.0.1:%d/%s/{color,depth}, encoder %s", cfg.rtsp_port, cfg.mount_base.c_str(),
              encoder.c_str());
}

void Streamer::Impl::stop() {
  if (!listen) return;
  listen.reset();
  // Dropping the sessions and closing the clients makes gst-rtsp-server unprepare their media on its client threads.
  Owned<GstRTSPSessionPool, g_object_unref> pool(gst_rtsp_server_get_session_pool(server.get()));
  g_list_free_full(gst_rtsp_session_pool_filter(pool.get(), remove_session, nullptr), g_object_unref);
  g_list_free_full(gst_rtsp_server_client_filter(server.get(), remove_client, nullptr), g_object_unref);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  for (Mount* m : {color.get(), depth.get()}) {
    std::unique_lock<std::mutex> lock(m->mu);
    while (!m->unprepared.wait_for(lock, std::chrono::milliseconds(250), [m] { return m->media == nullptr; }) &&
           std::chrono::steady_clock::now() < deadline) {
      lock.unlock();
      send_blank(*m);
      lock.lock();
    }
    if (m->media) RCLCPP_ERROR(logger, "stream %s: encoder still running 5 s after stop()", m->name.c_str());
  }
}

Streamer::Streamer(const rclcpp::Logger& logger, StreamConfig cfg)
    : impl_(std::make_unique<Impl>(logger, std::move(cfg))) {}

Streamer::~Streamer() = default;

void Streamer::start() { impl_->start(); }
void Streamer::stop() { impl_->stop(); }

bool Streamer::color_wanted() const { return impl_->color->wanted; }
bool Streamer::depth_wanted() const { return impl_->depth->wanted; }

void Streamer::push_color_yuyv(const uint8_t* data, int stride, uint64_t stamp_ns) {
  Mount& m = *impl_->color;
  push(m, stamp_ns, [&](uint8_t* dst) { yuyv_to_nv12(data, size_t(stride), m.info, dst); });
}

void Streamer::push_depth_z16(const uint16_t* data, int stride, float depth_scale_m, uint64_t stamp_ns) {
  Impl& s = *impl_;
  push(*s.depth, stamp_ns, [&](uint8_t* dst) {
    if (depth_scale_m != s.lut_scale) {
      fill_grey_lut(s.lut, depth_scale_m, s.cfg.depth_min_m, s.cfg.depth_max_m);
      s.lut_scale = depth_scale_m;
    }
    const GstVideoInfo& info = s.depth->info;
    depth_to_grey(data, size_t(stride), s.cfg.depth_w, s.cfg.depth_h, s.lut, dst + GST_VIDEO_INFO_PLANE_OFFSET(&info, 0),
                  size_t(GST_VIDEO_INFO_PLANE_STRIDE(&info, 0)));
    std::memset(dst + GST_VIDEO_INFO_PLANE_OFFSET(&info, 1), 128,
                size_t(GST_VIDEO_INFO_PLANE_STRIDE(&info, 1)) * size_t(GST_VIDEO_INFO_COMP_HEIGHT(&info, 1)));
  });
}

std::string Streamer::encoder_name() const { return impl_->encoder; }

}  // namespace bot_realsense
