#!/usr/bin/env python3
# file: jetson_suite.launch.py

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration


def _load_robot_config():
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']
    return config


def generate_launch_description():
    # --- Launch args ---
    jtop_exec_arg = DeclareLaunchArgument(
        'jtop_exec',
        default_value='ros2_jtop_node.py',
        description='Executable for the JTOP lifecycle node'
    )
    netmode_exec_arg = DeclareLaunchArgument(
        'netmode_exec',
        default_value='network_mode_publisher.py',
        description='Executable for the Network Mode lifecycle node'
    )
    wifi_services_exec_arg = DeclareLaunchArgument(
        'wifi_services_exec',
        default_value='wifi_services.py',
        description='Executable for the Wi-Fi services lifecycle node'
    )


    jtop_exec = LaunchConfiguration('jtop_exec')
    netmode_exec = LaunchConfiguration('netmode_exec')
    wifi_services_exec = LaunchConfiguration('wifi_services_exec')

    # --- Load configuration from robot_config.yaml ---
    robot_config = _load_robot_config()
    robot_name = robot_config['robot_name']
    network_interface = robot_config.get('network_interface', 'eno1')
    wifi_interface = robot_config.get('wifi_interface', 'wlP1p1s0')
    wifi_ssid = robot_config.get('wifi_ssid', '')
    wifi_password = robot_config.get('wifi_password', '')

    # --- Lifecycle nodes ---
    jtop_node = LifecycleNode(
        package='bot_jetson_stats',
        executable=jtop_exec,
        name='jtop_publisher',
        namespace=robot_name,
        output='screen',
        emulate_tty=True,
    )

    netmode_node = LifecycleNode(
        package='bot_jetson_stats',
        executable=netmode_exec,
        name='network_mode_lifecycle_publisher',
        namespace=robot_name,
        output='screen',
        emulate_tty=True,
    )

    wifi_services_node = LifecycleNode(
        package='bot_jetson_stats',
        executable=wifi_services_exec,
        name='wifi_services',
        namespace=robot_name,
        output='screen',
        emulate_tty=True,
        parameters=[
            # --- Required parameters ---
            {"iface": wifi_interface},
            {"default_timeout": 30.0},
            {"force_rescan": False},
            {"use_sudo": False},

            # --- Watchdog / Network Monitoring parameters ---
            {"status_file": "/opt/network_status/network_mode_status.txt"},
            {"watchdog_enabled": True},
            {"check_interval": 5.0},

            # --- Wi-Fi connection defaults ---
            {"wifi_ssid": wifi_ssid},
            {"wifi_pass": wifi_password},
            {"wifi_iface": wifi_interface},

            # --- 4G / Ethernet parameters ---
            {"fourg_iface": "enx344b50000000"},
            {"robot_iface": network_interface},
            {"robot_conn_name": "go2"},
        ],
    )

    return LaunchDescription([
        jtop_exec_arg,
        netmode_exec_arg,
        wifi_services_exec_arg,
        jtop_node,
        netmode_node,
        wifi_services_node,
    ])