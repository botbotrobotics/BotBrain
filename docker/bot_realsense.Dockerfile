# Base image plus what bot_realsense needs to build and run: librealsense headers and CMake
# config, gst-rtsp-server, and the GStreamer plugins for x264 encoding and RTP.
FROM botbotrobotics/botbrain:base

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-humble-librealsense2 \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstrtspserver-1.0-dev \
        gstreamer1.0-rtsp \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-ugly \
    && rm -rf /var/lib/apt/lists/*
