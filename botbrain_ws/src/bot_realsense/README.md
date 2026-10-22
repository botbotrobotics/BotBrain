<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_realsense

**One lifecycle node per Intel RealSense D400 camera: ROS topics for SLAM, Nav2 and YOLO, plus H.264 video for the browser.**

`bot_realsense` owns the camera and serves everyone from one capture. It replaces `realsense2_camera`, the Python JPEG node and JPEG-over-rosbridge.

- **ROS topics** with the names `realsense2_camera` used, so RTAB-Map, `depthimage_to_laserscan` and YOLO need no changes. Each topic is computed only while it has subscribers, at most at its configured rate.
- **Browser video**: colour and grey-coded depth are encoded once as H.264 and served over RTSP on `127.0.0.1`. MediaMTX (`config/mediamtx.yml`) pulls a stream only while a browser watches it and sends it to every viewer over WebRTC (WHEP). An encoder runs only while its stream has a viewer.

## Package Purpose

- **Capture**: librealsense per-sensor API with native formats (`format-conversion: basic`): depth Z16, colour YUYV. No `rs2::pipeline`, no librealsense colourisation or decimation.
- **Hand-off**: librealsense callbacks only drop the newest frame into a one-frame slot. A worker thread per sensor does the work, so a slow consumer drops frames instead of queueing them.
- **On demand**: no subscribers and no viewers means no conversion and no encoding, only capture.
- **Lifecycle**: plugs into `bot_state_machine` like any BotBrain lifecycle node.
- **Recovery**: if the camera is unplugged, or delivers no frames for `frame_timeout_s`, the node deactivates and cleans up through its own lifecycle transitions, so the state machine sees them. When the camera is back it configures and activates again.

Tested with the D435i. The D455 and D555 are expected to work if they offer the configured modes. Configure fails with the list of available modes when they do not.

---

## Lifecycle

| Transition | Effect |
|---|---|
| configure | Opens the camera by serial (optional `initial_reset`), reads intrinsics, extrinsics and depth scale, creates publishers, publishes static TF, builds the RTSP server (no encoding yet). Invalid parameters or a missing camera or mode give FAILURE with a log line. |
| activate | Opens and starts both sensors, starts the worker threads and the RTSP server. |
| deactivate | Stops and closes both sensors (the camera goes idle and its `/dev/video*` nodes are closed), joins the workers, stops the RTSP server. |
| cleanup | Releases the camera, publishers and streamer. |

The node never calls `exit()`, and it resets the hardware only when `initial_reset` is true. With `serial_no` set, the camera is looked up on every RealSense product line. Without it, the first D400 is used, or else the first D500.

While the node is active, a timer checks two things: that the camera is still connected, and that both sensors delivered a frame within `frame_timeout_s`. If either check fails, the node logs an error and runs deactivate and cleanup. It then checks every 2 s whether the camera is back. Once it is, the node runs configure and activate. A failed bring-up backs off, up to 30 s between attempts.

## Topics

Topics are relative to the node, `/<namespace>/<node>/...`. Images are published only while subscribed, with reliable keep-last-1 QoS.

| Topic | Type | Notes |
|---|---|---|
| `color/image_raw` | `sensor_msgs/Image` `rgb8` | YUYV converted with librealsense's BT.601 maths |
| `color/camera_info` | `sensor_msgs/CameraInfo` | |
| `depth/image_rect_raw` | `sensor_msgs/Image` `16UC1` | millimetres |
| `depth/camera_info` | `sensor_msgs/CameraInfo` | |
| `aligned_depth_to_color/image_raw` | `sensor_msgs/Image` `16UC1` | depth registered to the colour image, only with `align_depth` |
| `aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | colour intrinsics |
| `/tf_static` | | `<tf_prefix><camera_name>_link` → `_depth_frame` → `_depth_optical_frame`, `_link` → `_color_frame` → `_color_optical_frame` (realsense-ros convention) |

Header stamps are the frame capture time on the host clock (librealsense global time). Depth images use the `_depth_optical_frame` frame id; colour and aligned images use `_color_optical_frame`.

Aligned depth is published once for each colour frame that is published, under that colour frame's exact stamp, so RTAB-Map gets exactly matching pairs. It is built from the depth frame nearest in time. The colour worker picks these frames at `color_rate`, and the topic follows colour's picks even when only aligned depth has subscribers. The depth topic keeps its own `depth_rate`.

Aligned depth is computed by our own aligner. It matches `rs2::align`'s NEON implementation within 1 mm on more than 99.99% of valid pixels of live D435i frames. On the Orin NX it runs in about 70% of the time `rs2::align` takes. Because it is our own code, it allocates nothing per frame.

## Video streams

- `rtsp://127.0.0.1:<stream_rtsp_port>/<node>/color`: colour, H.264 constrained baseline, CBR, IDR every `stream_gop_s`.
- `rtsp://127.0.0.1:<stream_rtsp_port>/<node>/depth`: depth decimated 2x2 into grey luma, with full range signalled in the SPS.
  - 0 means no depth. Valid depth z maps to t = (zmin^-0.5 - z^-0.5) / (zmin^-0.5 - zmax^-0.5), and Y = 16 + round(219 t).
  - The browser inverts this mapping and colours the depth itself.
- The encoder is `nvv4l2h264enc` where NVENC exists (Orin NX/AGX) and `x264enc` otherwise (Orin Nano).

Browsers reach the streams through MediaMTX at `http://<robot>:8889/<node>/<color|depth>/whep`.

## Parameters

All parameters are read at configure and are read-only at runtime: `ros2 param set` is refused rather than silently ignored. Set them from the launch file.

| Name | Default | Meaning |
|---|---|---|
| `serial_no` | `""` | camera serial; `""` = first D400 found. An integer or a `_`-prefixed string is accepted too |
| `camera_name` | node name | TF frame base name |
| `tf_prefix` | `""` | prepended to frame names verbatim |
| `initial_reset` | `false` | hardware reset once at configure |
| `depth_width` / `depth_height` / `depth_fps` | 848 / 480 / 30 | depth mode (Z16) |
| `color_width` / `color_height` / `color_fps` | 640 / 480 / 30 | colour mode (YUYV) |
| `color_rate` / `depth_rate` | 0.0 | max publish rate in Hz, 0 = every frame; aligned depth follows `color_rate` |
| `frame_timeout_s` | 5.0 | seconds without frames from either sensor, while active, before the node recovers |
| `align_depth` | `true` | offer the aligned depth topic |
| `stream_enable` | `true` | serve the RTSP streams |
| `stream_rtsp_port` | 8560 | RTSP port, one per camera node |
| `stream_encoder` | `auto` | `auto`, `nvenc` or `x264` |
| `stream_color_fps` / `stream_depth_fps` | 15 / 15 | encoded frames per second |
| `stream_color_bitrate` / `stream_depth_bitrate` | 1000000 / 2000000 | bits per second |
| `stream_gop_s` | 1.0 | IDR interval in seconds |
| `depth_min_m` / `depth_max_m` | 0.3 / 8.0 | grey depth range |
| `auto_exposure_priority` | `false` | colour sensor; `false` keeps the frame rate in low light |

## Running

```bash
ros2 run bot_realsense bot_realsense_node --ros-args -r __node:=front_camera -p camera_name:=front_camera
ros2 lifecycle set /front_camera configure
ros2 lifecycle set /front_camera activate
```

The node is also a component, `bot_realsense::RealsenseNode`. In BotBrain, `bot_localization/launch/realsense.launch.py` launches it, and `bot_state_machine` drives its transitions.

## Build and test

```bash
colcon build --packages-select bot_realsense      # Release unless CMAKE_BUILD_TYPE is set
colcon test --packages-select bot_realsense && colcon test-result --verbose
```

Dependencies:
- librealsense2 2.55 or newer, for the context settings that select `format-conversion: basic`. Humble's `ros-humble-librealsense2` is newer than that.
- GStreamer 1.20 or newer with gst-rtsp-server, plus the base, good and ugly plugins.
- On Jetson, the L4T multimedia plugins provide NVENC.
- In BotBrain, `docker/bot_realsense.Dockerfile` adds these to the base image. Compose builds it as `camera_base` on first use, and the `builder_base` and `localization` services run on it.

Unit tests cover the pure parts:
- the rate limiter;
- YUYV to RGB;
- millimetre conversion;
- the aligner against `rs2::align` on a software device;
- the frame hand-off slot;
- the depth grey coding and decimation;
- the recovery retry schedule and the colour/depth frame pairing.
