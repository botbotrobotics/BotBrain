#include "bot_realsense/realsense_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pthread.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace bot_realsense {
namespace {

using LifecycleState = lifecycle_msgs::msg::State;
using LifecycleTransition = lifecycle_msgs::msg::Transition;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using namespace std::chrono_literals;

constexpr int64_t kRetryPeriodNs = 2'000'000'000, kRetryMaxNs = 30'000'000'000;

int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

rs2::device find_device(const rs2::context& context, const std::string& serial) {
  if (!serial.empty()) {
    for (auto&& device : context.query_devices(RS2_PRODUCT_LINE_ANY)) {
      if (device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) && serial == device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
        return device;
      }
    }
    return {};
  }
  for (const int line : {RS2_PRODUCT_LINE_D400, RS2_PRODUCT_LINE_D500}) {
    const rs2::device_list devices = context.query_devices(line);
    if (devices.size() > 0) return devices[0];
  }
  return {};
}

rs2::stream_profile find_profile(const rs2::sensor& sensor, rs2_stream stream, rs2_format format, int w, int h,
                                 int fps) {
  std::set<std::tuple<int, int, int>> available;
  for (auto&& profile : sensor.get_stream_profiles()) {
    const auto video = profile.as<rs2::video_stream_profile>();
    if (!video || video.stream_type() != stream || video.format() != format) continue;
    if (video.width() == w && video.height() == h && video.fps() == fps) return profile;
    available.emplace(video.width(), video.height(), video.fps());
  }
  std::string modes;
  for (const auto& [mode_w, mode_h, mode_fps] : available) {
    modes += " " + std::to_string(mode_w) + "x" + std::to_string(mode_h) + "@" + std::to_string(mode_fps);
  }
  throw std::runtime_error(std::string("camera has no ") + rs2_stream_to_string(stream) + " " +
                           rs2_format_to_string(format) + " mode " + std::to_string(w) + "x" + std::to_string(h) +
                           "@" + std::to_string(fps) + "; available:" + (modes.empty() ? " none" : modes));
}

CameraInfo make_camera_info(const rs2_intrinsics& in, const std::string& frame_id) {
  CameraInfo info;
  info.header.frame_id = frame_id;
  info.width = in.width;
  info.height = in.height;
  const bool fisheye = in.model == RS2_DISTORTION_KANNALA_BRANDT4;
  info.distortion_model = fisheye ? "equidistant" : "plumb_bob";
  info.d.assign(in.coeffs, in.coeffs + (fisheye ? 4 : 5));
  info.k = {in.fx, 0, in.ppx, 0, in.fy, in.ppy, 0, 0, 1};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {in.fx, 0, in.ppx, 0, 0, in.fy, in.ppy, 0, 0, 0, 1, 0};
  return info;
}

geometry_msgs::msg::TransformStamped make_transform(const rclcpp::Time& stamp, const std::string& parent,
                                                    const std::string& child, const tf2::Vector3& t,
                                                    const tf2::Quaternion& q) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = t.x();
  tf.transform.translation.y = t.y();
  tf.transform.translation.z = t.z();
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  return tf;
}

}  // namespace

RealsenseNode::RealsenseNode(const rclcpp::NodeOptions& options)
    : LifecycleNode("camera", options), retry_(kRetryPeriodNs, kRetryMaxNs) {
  // read_only: values apply only at configure, and the browser decodes depth with the live depth_min_m/depth_max_m.
  rcl_interfaces::msg::ParameterDescriptor read_only;
  read_only.read_only = true;
  const auto declare = [&](const char* name, const rclcpp::ParameterValue& value) {
    declare_parameter(name, value, read_only);
  };
  rcl_interfaces::msg::ParameterDescriptor serial = read_only;
  serial.dynamic_typing = true;  // launch files YAML-parse an all-digit serial into an integer
  declare_parameter("serial_no", rclcpp::ParameterValue(std::string()), serial);
  declare("camera_name", rclcpp::ParameterValue(std::string(get_name())));
  declare("tf_prefix", rclcpp::ParameterValue(std::string()));
  declare("initial_reset", rclcpp::ParameterValue(false));
  declare("depth_width", rclcpp::ParameterValue(848));
  declare("depth_height", rclcpp::ParameterValue(480));
  declare("depth_fps", rclcpp::ParameterValue(30));
  declare("color_width", rclcpp::ParameterValue(640));
  declare("color_height", rclcpp::ParameterValue(480));
  declare("color_fps", rclcpp::ParameterValue(30));
  declare("color_rate", rclcpp::ParameterValue(0.0));
  declare("depth_rate", rclcpp::ParameterValue(0.0));
  declare("align_depth", rclcpp::ParameterValue(true));
  declare("frame_timeout_s", rclcpp::ParameterValue(5.0));
  declare("stream_enable", rclcpp::ParameterValue(true));
  declare("stream_rtsp_port", rclcpp::ParameterValue(8560));
  declare("stream_encoder", rclcpp::ParameterValue(std::string("auto")));
  declare("stream_color_fps", rclcpp::ParameterValue(15));
  declare("stream_depth_fps", rclcpp::ParameterValue(15));
  declare("stream_color_bitrate", rclcpp::ParameterValue(1000000));
  declare("stream_depth_bitrate", rclcpp::ParameterValue(2000000));
  declare("stream_gop_s", rclcpp::ParameterValue(1.0));
  declare("depth_min_m", rclcpp::ParameterValue(0.3));
  declare("depth_max_m", rclcpp::ParameterValue(8.0));
  declare("auto_exposure_priority", rclcpp::ParameterValue(false));
  watchdog_ = create_wall_timer(500ms, [this] { watchdog_tick(); });
}

RealsenseNode::~RealsenseNode() {
  try {
    stop_capture();
    release();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "shutdown: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "shutdown: unknown exception");
  }
}

RealsenseNode::Params RealsenseNode::read_params() {
  const auto integer = [this](const char* name) {
    const int64_t v = get_parameter(name).as_int();
    if (v < INT32_MIN || v > INT32_MAX) throw std::invalid_argument(std::string(name) + " is out of range");
    return static_cast<int>(v);
  };
  const auto real = [this](const char* name) { return get_parameter(name).as_double(); };
  const auto flag = [this](const char* name) { return get_parameter(name).as_bool(); };
  const auto text = [this](const char* name) { return get_parameter(name).as_string(); };

  Params p;
  const rclcpp::Parameter serial = get_parameter("serial_no");
  p.serial_no = serial.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ? std::to_string(serial.as_int())
                                                                             : serial.as_string();
  if (!p.serial_no.empty() && p.serial_no.front() == '_') p.serial_no.erase(0, 1);  // realsense-ros "_<serial>"
  p.camera_name = text("camera_name");
  p.tf_prefix = text("tf_prefix");
  p.initial_reset = flag("initial_reset");
  p.depth_w = integer("depth_width");
  p.depth_h = integer("depth_height");
  p.depth_fps = integer("depth_fps");
  p.color_w = integer("color_width");
  p.color_h = integer("color_height");
  p.color_fps = integer("color_fps");
  p.color_rate = real("color_rate");
  p.depth_rate = real("depth_rate");
  p.frame_timeout_s = real("frame_timeout_s");
  p.align_depth = flag("align_depth");
  p.stream_enable = flag("stream_enable");
  p.auto_exposure_priority = flag("auto_exposure_priority");

  StreamConfig& s = p.stream;
  s.mount_base = get_name();
  s.rtsp_port = integer("stream_rtsp_port");
  s.encoder = text("stream_encoder");
  s.color_w = p.color_w;
  s.color_h = p.color_h;
  s.color_fps_in = p.color_fps;
  s.color_fps_out = integer("stream_color_fps");
  s.color_bitrate = integer("stream_color_bitrate");
  s.depth_w = p.depth_w;
  s.depth_h = p.depth_h;
  s.depth_fps_in = p.depth_fps;
  s.depth_fps_out = integer("stream_depth_fps");
  s.depth_bitrate = integer("stream_depth_bitrate");
  s.gop_s = real("stream_gop_s");
  s.depth_min_m = static_cast<float>(real("depth_min_m"));
  s.depth_max_m = static_cast<float>(real("depth_max_m"));

  if (p.camera_name.empty()) throw std::invalid_argument("camera_name must not be empty");
  if (p.color_rate < 0 || p.depth_rate < 0) throw std::invalid_argument("color_rate and depth_rate must be >= 0");
  if (!(p.frame_timeout_s > 0)) throw std::invalid_argument("frame_timeout_s must be > 0");
  if (s.rtsp_port < 1 || s.rtsp_port > 65535) throw std::invalid_argument("stream_rtsp_port must be 1..65535");
  if (s.color_bitrate <= 0 || s.depth_bitrate <= 0 || s.gop_s <= 0) {
    throw std::invalid_argument("stream bitrates and stream_gop_s must be > 0");
  }
  if (!(s.depth_min_m > 0 && s.depth_min_m < s.depth_max_m)) {
    throw std::invalid_argument("need 0 < depth_min_m < depth_max_m");
  }
  return p;
}

void RealsenseNode::open_device() {
  context_.emplace(R"({"format-conversion":"basic"})");
  device_ = find_device(*context_, params_.serial_no);
  if (!device_) {
    throw std::runtime_error(params_.serial_no.empty() ? "no RealSense D400 or D500 camera found"
                                                       : "RealSense camera " + params_.serial_no + " not found");
  }
  serial_ = device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

  if (params_.initial_reset) hardware_reset(serial_);

  for (auto&& sensor : device_.query_sensors()) {
    if (sensor.is<rs2::depth_sensor>()) {
      stereo_ = sensor;
      continue;
    }
    for (auto&& profile : sensor.get_stream_profiles()) {
      if (profile.stream_type() == RS2_STREAM_COLOR) {
        rgb_ = sensor;
        break;
      }
    }
  }
  if (!stereo_ || !rgb_) throw std::runtime_error("camera " + serial_ + " lacks a depth or colour sensor");

  depth_profile_ = find_profile(stereo_, RS2_STREAM_DEPTH, RS2_FORMAT_Z16, params_.depth_w, params_.depth_h,
                                params_.depth_fps);
  color_profile_ = find_profile(rgb_, RS2_STREAM_COLOR, RS2_FORMAT_YUYV, params_.color_w, params_.color_h,
                                params_.color_fps);
  depth_scale_ = stereo_.as<rs2::depth_sensor>().get_depth_scale();

  for (rs2::sensor* sensor : {&stereo_, &rgb_}) {
    if (sensor->supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) sensor->set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.f);
  }
  if (rgb_.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY)) {
    rgb_.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, params_.auto_exposure_priority ? 1.f : 0.f);
  }

  device_removed_ = false;
  context_->set_devices_changed_callback([this, device = device_](rs2::event_information& info) {
    if (info.was_removed(device)) device_removed_ = true;
  });

  RCLCPP_INFO(get_logger(), "%s %s firmware %s on USB %s: depth Z16 %dx%d@%d (scale %g m), colour YUYV %dx%d@%d",
              device_.get_info(RS2_CAMERA_INFO_NAME), serial_.c_str(),
              device_.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION),
              device_.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
                  ? device_.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
                  : "?",
              params_.depth_w, params_.depth_h, params_.depth_fps, depth_scale_, params_.color_w, params_.color_h,
              params_.color_fps);
}

void RealsenseNode::hardware_reset(const std::string& serial) {
  // librealsense runs this callback on its own thread, possibly late, so its state must outlive this stack frame.
  struct Arrival {
    std::mutex mutex;
    std::condition_variable arrived;
    rs2::device device;
  };
  const auto arrival = std::make_shared<Arrival>();
  context_->set_devices_changed_callback([arrival, serial](rs2::event_information& info) {
    for (auto&& device : info.get_new_devices()) {
      if (serial != device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) continue;
      {
        std::lock_guard<std::mutex> lock(arrival->mutex);
        arrival->device = device;
      }
      arrival->arrived.notify_all();
    }
  });
  RCLCPP_INFO(get_logger(), "Resetting camera %s", serial.c_str());
  device_.hardware_reset();
  device_ = rs2::device();
  {
    std::unique_lock<std::mutex> lock(arrival->mutex);
    arrival->arrived.wait_for(lock, 20s, [&] { return static_cast<bool>(arrival->device); });
    device_ = arrival->device;
  }
  context_->set_devices_changed_callback([](rs2::event_information&) {});
  if (!device_) throw std::runtime_error("camera " + serial + " did not come back within 20 s of hardware reset");
}

RealsenseNode::ImageOutput RealsenseNode::make_output(const std::string& image_topic, const std::string& info_topic,
                                                      const rs2_intrinsics& in, const std::string& frame_id,
                                                      const char* encoding, uint32_t bytes_per_pixel, double rate_hz) {
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  ImageOutput out;
  out.image = create_publisher<Image>("~/" + image_topic, qos);
  out.info = create_publisher<CameraInfo>("~/" + info_topic, qos);
  out.info_msg = make_camera_info(in, frame_id);
  Image& image = out.image_msg;
  image.header.frame_id = frame_id;
  image.width = out.info_msg.width;
  image.height = out.info_msg.height;
  image.encoding = encoding;
  image.step = image.width * bytes_per_pixel;
  image.data.resize(size_t(image.step) * image.height);
  out.rate = RateLimiter(rate_hz);
  return out;
}

void RealsenseNode::create_outputs(const rs2_intrinsics& depth, const rs2_intrinsics& color,
                                   const rs2_extrinsics& color_to_depth) {
  const std::string base = params_.tf_prefix + params_.camera_name;
  const std::string depth_optical = base + "_depth_optical_frame";
  const std::string color_optical = base + "_color_optical_frame";
  color_out_ = make_output("color/image_raw", "color/camera_info", color, color_optical, "rgb8", 3, params_.color_rate);
  depth_out_ =
      make_output("depth/image_rect_raw", "depth/camera_info", depth, depth_optical, "16UC1", 2, params_.depth_rate);
  if (params_.align_depth) {
    // Its frames are picked by the colour worker, so it needs no rate limit of its own.
    aligned_out_ = make_output("aligned_depth_to_color/image_raw", "aligned_depth_to_color/camera_info", color,
                               color_optical, "16UC1", 2, 0.0);
  }

  // realsense-ros convention: _link sits at the depth origin with ROS axes (x forward, y left, z up); the optical
  // frames use camera axes (x right, y down, z forward). Extrinsics are in camera axes, rotation column-major.
  tf2::Quaternion optical;
  optical.setRPY(-M_PI / 2, 0, -M_PI / 2);
  const float* r = color_to_depth.rotation;
  tf2::Quaternion color_rotation;
  tf2::Matrix3x3(r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]).getRotation(color_rotation);
  color_rotation = optical * color_rotation * optical.inverse();
  const float* t = color_to_depth.translation;
  const tf2::Vector3 zero(0, 0, 0);
  const rclcpp::Time stamp = now();
  tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  tf_broadcaster_->sendTransform({
      make_transform(stamp, base + "_link", base + "_depth_frame", zero, tf2::Quaternion::getIdentity()),
      make_transform(stamp, base + "_depth_frame", depth_optical, zero, optical),
      make_transform(stamp, base + "_link", base + "_color_frame", tf2::Vector3(t[2], -t[0], -t[1]), color_rotation),
      make_transform(stamp, base + "_color_frame", color_optical, zero, optical),
  });
}

RealsenseNode::CallbackReturn RealsenseNode::on_configure(const rclcpp_lifecycle::State&) {
  try {
    params_ = read_params();
    open_device();
    depth_period_ns_ = 1'000'000'000 / params_.depth_fps;
    const auto depth = depth_profile_.as<rs2::video_stream_profile>().get_intrinsics();
    const auto color = color_profile_.as<rs2::video_stream_profile>().get_intrinsics();
    create_outputs(depth, color, color_profile_.get_extrinsics_to(depth_profile_));
    if (params_.align_depth) aligner_.emplace(depth, depth_profile_.get_extrinsics_to(color_profile_), color);
    if (params_.stream_enable) streamer_ = std::make_unique<Streamer>(get_logger(), params_.stream);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "configure failed: %s", e.what());
    release();
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

RealsenseNode::CallbackReturn RealsenseNode::on_activate(const rclcpp_lifecycle::State& state) {
  for (ImageOutput* out : {&color_out_, &depth_out_, &aligned_out_}) {
    if (!out->image) continue;
    out->image->on_activate();
    out->info->on_activate();
  }
  try {
    if (streamer_) streamer_->start();
    start_capture();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "activate failed: %s", e.what());
    on_deactivate(state);
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

RealsenseNode::CallbackReturn RealsenseNode::on_deactivate(const rclcpp_lifecycle::State&) {
  stop_capture();
  for (ImageOutput* out : {&color_out_, &depth_out_, &aligned_out_}) {
    if (!out->image) continue;
    out->image->on_deactivate();
    out->info->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

RealsenseNode::CallbackReturn RealsenseNode::on_cleanup(const rclcpp_lifecycle::State&) {
  release();
  return CallbackReturn::SUCCESS;
}

RealsenseNode::CallbackReturn RealsenseNode::on_shutdown(const rclcpp_lifecycle::State&) {
  stop_capture();
  release();
  return CallbackReturn::SUCCESS;
}

RealsenseNode::CallbackReturn RealsenseNode::on_error(const rclcpp_lifecycle::State&) {
  stop_capture();
  release();
  return CallbackReturn::SUCCESS;
}

void RealsenseNode::start_capture() {
  depth_slot_.open();
  color_slot_.open();
  aligned_pick_ns_ = 0;
  last_depth_frame_ns_ = steady_ns();  // the stall timeout also covers sensor start-up
  last_color_frame_ns_ = steady_ns();
  depth_thread_ = std::thread([this] { depth_loop(); });
  color_thread_ = std::thread([this] { color_loop(); });
  pthread_setname_np(depth_thread_.native_handle(), "rs_depth");
  pthread_setname_np(color_thread_.native_handle(), "rs_color");
  stereo_.open(depth_profile_);
  rgb_.open(color_profile_);
  stereo_.start([this](rs2::frame frame) { depth_slot_.put(std::move(frame)); });
  rgb_.start([this](rs2::frame frame) { color_slot_.put(std::move(frame)); });
}

void RealsenseNode::stop_capture() {
  // Streamer first, while frames still flow: gst-rtsp-server cannot tear down a media still waiting to preroll.
  if (streamer_) streamer_->stop();
  // stop() waits for running callbacks; both calls throw if never started/opened or once the camera is gone.
  for (rs2::sensor* sensor : {&stereo_, &rgb_}) {
    if (!*sensor) continue;
    try {
      sensor->stop();
    } catch (const rs2::error& e) {
      RCLCPP_DEBUG(get_logger(), "sensor stop: %s", e.what());
    }
    try {
      sensor->close();
    } catch (const rs2::error& e) {
      RCLCPP_DEBUG(get_logger(), "sensor close: %s", e.what());
    }
  }
  depth_slot_.close();
  color_slot_.close();
  if (depth_thread_.joinable()) depth_thread_.join();
  if (color_thread_.joinable()) color_thread_.join();
}

void RealsenseNode::release() {
  // The removal callback holds a device ref; releasing the last one inside ~context deadlocks librealsense.
  if (context_) context_->set_devices_changed_callback([](rs2::event_information&) {});
  streamer_.reset();
  aligner_.reset();
  tf_broadcaster_.reset();
  color_out_ = ImageOutput();
  depth_out_ = ImageOutput();
  aligned_out_ = ImageOutput();
  depth_profile_ = rs2::stream_profile();
  color_profile_ = rs2::stream_profile();
  stereo_ = rs2::sensor();
  rgb_ = rs2::sensor();
  device_ = rs2::device();
  context_.reset();
}

// Recovery runs through lifecycle transitions so bot_state_machine sees every one of them.
void RealsenseNode::watchdog_tick() {
  if (!rclcpp::ok()) return;
  const uint8_t state = get_current_state().id();
  const int64_t now_ns = steady_ns();
  if (state == LifecycleState::PRIMARY_STATE_ACTIVE) {
    const auto timeout_ns = static_cast<int64_t>(params_.frame_timeout_s * 1e9);
    const bool removed = device_removed_;
    if (!removed && now_ns - last_depth_frame_ns_ <= timeout_ns && now_ns - last_color_frame_ns_ <= timeout_ns) return;
    if (removed) {
      RCLCPP_ERROR(get_logger(), "camera %s was disconnected; deactivating and cleaning up until it is back",
                   serial_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "camera %s delivered no frames for %g s; deactivating, cleaning up and reconfiguring",
                   serial_.c_str(), params_.frame_timeout_s);
    }
    trigger_transition(LifecycleTransition::TRANSITION_DEACTIVATE);
    trigger_transition(LifecycleTransition::TRANSITION_CLEANUP);
    retry_.arm(steady_ns());
    waiting_logged_ = false;
    return;
  }
  if (!retry_.armed()) return;
  if (state != LifecycleState::PRIMARY_STATE_UNCONFIGURED) {  // someone else took over
    retry_.disarm();
    return;
  }
  if (!retry_.due(now_ns)) return;
  if (!camera_present()) {
    if (!waiting_logged_) RCLCPP_WARN(get_logger(), "waiting for camera %s to come back", serial_.c_str());
    waiting_logged_ = true;
    retry_.camera_absent(now_ns);
    return;
  }
  RCLCPP_INFO(get_logger(), "camera %s is present; configuring and activating", serial_.c_str());
  if (trigger_transition(LifecycleTransition::TRANSITION_CONFIGURE).id() == LifecycleState::PRIMARY_STATE_INACTIVE &&
      trigger_transition(LifecycleTransition::TRANSITION_ACTIVATE).id() == LifecycleState::PRIMARY_STATE_ACTIVE) {
    RCLCPP_INFO(get_logger(), "camera %s recovered", serial_.c_str());
    retry_.disarm();
    return;
  }
  if (get_current_state().id() == LifecycleState::PRIMARY_STATE_INACTIVE) {
    trigger_transition(LifecycleTransition::TRANSITION_CLEANUP);
  }
  retry_.bring_up_failed(steady_ns());
}

bool RealsenseNode::camera_present() const {
  try {
    return static_cast<bool>(find_device(rs2::context(), serial_));
  } catch (const rs2::error&) {
    return false;
  }
}

int64_t RealsenseNode::frame_stamp_ns(const rs2::frame& frame) {
  // Global time is the host's system clock in ms, i.e. the ROS clock; a bare hardware clock has its own epoch.
  if (frame.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK) return now().nanoseconds();
  return static_cast<int64_t>(frame.get_timestamp() * 1e6);
}

bool RealsenseNode::wanted(const ImageOutput& out) {
  return out.image->get_subscription_count() > 0 || out.info->get_subscription_count() > 0;
}

template <class Fill>
void RealsenseNode::publish(ImageOutput& out, const rclcpp::Time& stamp, Fill&& fill) {
  if (out.image->get_subscription_count() > 0) {
    out.image_msg.header.stamp = stamp;
    fill(out.image_msg.data.data());
    out.image->publish(out.image_msg);
  }
  if (out.info->get_subscription_count() > 0) {
    out.info_msg.header.stamp = stamp;
    out.info->publish(out.info_msg);
  }
}

void RealsenseNode::process_depth(const rs2::frame& frame, int64_t stamp_ns, const rs2::frame& previous,
                                  int64_t previous_ns) {
  const auto z16_of = [](const rs2::frame& f) { return static_cast<const uint16_t*>(f.get_data()); };
  const auto stride_of = [](const rs2::frame& f) { return f.as<rs2::video_frame>().get_stride_in_bytes(); };
  const uint16_t* z16 = z16_of(frame);
  const int stride = stride_of(frame);
  if (wanted(depth_out_) && depth_out_.rate.allow(stamp_ns)) {
    publish(depth_out_, rclcpp::Time(stamp_ns, RCL_ROS_TIME), [&](uint8_t* data) {
      depth_to_mm(z16, stride, params_.depth_w, params_.depth_h, depth_scale_, reinterpret_cast<uint16_t*>(data));
    });
  }
  int64_t pick_ns = aligned_pick_ns_.load(std::memory_order_acquire);
  if (aligner_ && pick_ns != 0) {
    const DepthPick choice = pick_depth_for(pick_ns, previous ? previous_ns : 0, stamp_ns, depth_period_ns_);
    // The exchange fails if the colour worker posted a newer pick meanwhile; that one is served next frame.
    if (choice != DepthPick::kWait && choice != DepthPick::kSkip &&
        aligned_pick_ns_.compare_exchange_strong(pick_ns, 0)) {
      const rs2::frame& source = choice == DepthPick::kPrevious ? previous : frame;
      publish(aligned_out_, rclcpp::Time(pick_ns, RCL_ROS_TIME), [&](uint8_t* data) {
        aligner_->align(z16_of(source), stride_of(source), depth_scale_, reinterpret_cast<uint16_t*>(data));
      });
    } else if (choice == DepthPick::kSkip) {
      aligned_pick_ns_.compare_exchange_strong(pick_ns, 0);
    }
  }
  if (streamer_ && streamer_->depth_wanted()) {
    streamer_->push_depth_z16(z16, stride, depth_scale_, static_cast<uint64_t>(stamp_ns));
  }
}

void RealsenseNode::process_color(const rs2::frame& frame, int64_t stamp_ns) {
  const auto* yuyv = static_cast<const uint8_t*>(frame.get_data());
  const int stride = frame.as<rs2::video_frame>().get_stride_in_bytes();
  const bool colour = wanted(color_out_);
  const bool aligned = aligner_ && wanted(aligned_out_);
  // Aligned depth takes the picked colour frames' stamps: RTAB-Map pairs colour with aligned depth by exact stamp.
  if ((colour || aligned) && color_out_.rate.allow(stamp_ns)) {
    if (colour) {
      publish(color_out_, rclcpp::Time(stamp_ns, RCL_ROS_TIME),
              [&](uint8_t* data) { yuyv_to_rgb8(yuyv, stride, params_.color_w, params_.color_h, data); });
    }
    if (aligned) aligned_pick_ns_.store(stamp_ns, std::memory_order_release);
  }
  if (streamer_ && streamer_->color_wanted()) {
    streamer_->push_color_yuyv(yuyv, stride, static_cast<uint64_t>(stamp_ns));
  }
}

void RealsenseNode::depth_loop() {
  rs2::frame previous;
  int64_t previous_ns = 0;
  while (rs2::frame frame = depth_slot_.take()) {
    last_depth_frame_ns_.store(steady_ns(), std::memory_order_relaxed);
    try {
      const int64_t stamp_ns = frame_stamp_ns(frame);
      process_depth(frame, stamp_ns, previous, previous_ns);
      previous = std::move(frame);
      previous_ns = stamp_ns;
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "depth frame dropped: %s", e.what());
    }
  }
}

void RealsenseNode::color_loop() {
  while (const rs2::frame frame = color_slot_.take()) {
    last_color_frame_ns_.store(steady_ns(), std::memory_order_relaxed);
    try {
      process_color(frame, frame_stamp_ns(frame));
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "colour frame dropped: %s", e.what());
    }
  }
}

}  // namespace bot_realsense

RCLCPP_COMPONENTS_REGISTER_NODE(bot_realsense::RealsenseNode)
