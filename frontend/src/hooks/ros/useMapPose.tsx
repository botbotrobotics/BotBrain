'use client';

import { useState, useEffect, useRef } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

export interface MapPose {
  x: number;
  y: number;
  z: number;
  theta: number;
  timestamp?: number;
  frameId?: string;
}

// Re-export as RobotPose for compatibility
export type RobotPose = MapPose;

interface UseMapPoseOptions {
  enabled?: boolean;
  topicName?: string;
}

export default function useMapPose(options?: UseMapPoseOptions) {
  const { enabled = true, topicName = '/map_odom' } = options || {};
  const { connection } = useRobotConnection();
  const [pose, setPose] = useState<MapPose | null>(null);
  const receivedDataRef = useRef(false);
  const fallbackTopicRef = useRef<ROSLIB.Topic<unknown> | null>(null);

  useEffect(() => {
    if (!connection.ros || !connection.online || !enabled) return;

    // Reset received flag on new subscription
    receivedDataRef.current = false;

    console.log(`Subscribing to ${topicName} for map-relative robot position`);

    const poseListener = new ROSLIB.Topic({
      ros: connection.ros!,
      name: topicName,
      messageType: 'geometry_msgs/PoseStamped'
    });

    const handleMessage = (message: any) => {
      try {
        const poseData = message.pose;
        if (!poseData) return;

        receivedDataRef.current = true;

        // Extract position
        const x = poseData.position.x || 0;
        const y = poseData.position.y || 0;
        const z = poseData.position.z || 0;

        // Convert quaternion to yaw angle
        const q = poseData.orientation;
        const theta = Math.atan2(
          2 * (q.w * q.z + q.x * q.y),
          1 - 2 * (q.y * q.y + q.z * q.z)
        );

        setPose({
          x,
          y,
          z,
          theta,
          timestamp: Date.now(),
          frameId: message.header?.frame_id
        });
      } catch (err) {
        console.error('Error processing map pose message:', err);
      }
    };

    poseListener.subscribe(handleMessage);

    // Fallback to /odom if primary topic fails
    const fallbackTimeout = setTimeout(() => {
      if (!receivedDataRef.current) {
        console.log(`No data received from ${topicName}, trying /odom fallback`);

        const odomTopic = new ROSLIB.Topic({
          ros: connection.ros!,
          name: '/odom',
          messageType: 'nav_msgs/Odometry'
        });

        fallbackTopicRef.current = odomTopic;

        odomTopic.subscribe((message: any) => {
          try {
            const position = message.pose.pose.position;
            const orientation = message.pose.pose.orientation;

            const theta = Math.atan2(
              2 * (orientation.w * orientation.z + orientation.x * orientation.y),
              1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
            );

            setPose({
              x: position.x,
              y: position.y,
              z: position.z,
              theta,
              timestamp: Date.now(),
              frameId: message.header?.frame_id || 'odom'
            });
          } catch (err) {
            console.error('Error processing odometry message:', err);
          }
        });
      }
    }, 5000);

    return () => {
      clearTimeout(fallbackTimeout);
      poseListener.unsubscribe();
      if (fallbackTopicRef.current) {
        fallbackTopicRef.current.unsubscribe();
        fallbackTopicRef.current = null;
      }
    };
  }, [connection.ros, connection.online, enabled, topicName]);

  return pose;
}