'use client';

import { useState, useEffect } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { LaserScan } from '@/interfaces/ros/LaserScan';
import * as ROSLIB from 'roslib';

export default function useCustomLaserScan(
  customTopicName: string = '/scan',
  useDummy: boolean = false
): LaserScan {
  const { connection } = useRobotConnection();

  const [laserScan, setLaserScan] = useState<LaserScan>({
    angle_increment: 0,
    angle_max: 0,
    angle_min: 0,
    header: {
      frame_id: '',
      seq: 0,
      stamp: {
        nanosec: 0,
        sec: 0,
      },
    },
    range_max: 0,
    scan_time: 0,
    time_increment: 0,
    range_min: 0,
    intensities: [],
    ranges: [],
  });

  useEffect(() => {
    if (!connection.ros || !connection.online) return;

    // Make sure the topic name starts with a slash
    const topicName = customTopicName.startsWith('/')
      ? customTopicName
      : `/${customTopicName}`;

    // Create the topic with proper prefixing for dummy mode
    const fullTopicName = `${useDummy ? '/dummy' : ''}${topicName}`;

    // Create a direct ROSLIB Topic
    const topic = new ROSLIB.Topic({
      ros: connection.ros,
      name: fullTopicName,
      messageType: 'sensor_msgs/LaserScan',
      compression: 'none',
      throttle_rate: 0,
      queue_size: 1,
    });

    // Subscribe to the topic
    topic.subscribe((message: any) => {
      setLaserScan(message as LaserScan);
    });

    // Cleanup: unsubscribe when component unmounts or topic changes
    return () => {
      topic.unsubscribe();
    };
  }, [connection.ros, connection.online, customTopicName, useDummy]);

  return laserScan;
} 