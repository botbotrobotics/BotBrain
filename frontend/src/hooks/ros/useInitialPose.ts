'use client';

import { useEffect, useRef, useCallback } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

interface InitialPose {
  x: number;
  y: number;
  theta: number;
}

export default function useInitialPose(topicName: string = '/initialpose') {
  const { connection } = useRobotConnection();
  const topicRef = useRef<ROSLIB.Topic<unknown> | null>(null);

  const publishInitialPose = useCallback((pose: InitialPose) => {
    if (!connection.online || !connection.ros) {
      console.warn('Cannot publish initial pose: ROS not connected');
      return false;
    }

    if (!topicRef.current) {
      topicRef.current = new ROSLIB.Topic({
        ros: connection.ros,
        name: topicName,
        messageType: 'geometry_msgs/PoseWithCovarianceStamped'
      });
    }

    const quaternion = {
      x: 0,
      y: 0,
      z: Math.sin(pose.theta / 2),
      w: Math.cos(pose.theta / 2)
    };

    // In roslib 2.x, messages are plain objects (no `new ROSLIB.Message()` needed)
    const message = {
      header: {
        stamp: {
          sec: Math.floor(Date.now() / 1000),
          nsec: (Date.now() % 1000) * 1000000
        },
        frame_id: 'map'
      },
      pose: {
        pose: {
          position: {
            x: pose.x,
            y: pose.y,
            z: 0.0
          },
          orientation: quaternion
        },
        covariance: [
          0.25, 0, 0, 0, 0, 0,
          0, 0.25, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0.06853892326654787
        ]
      }
    };

    try {
      topicRef.current.publish(message);
      console.log('Published initial pose:', pose);
      return true;
    } catch (error) {
      console.error('Error publishing initial pose:', error);
      return false;
    }
  }, [connection.ros, connection.online, topicName]);

  useEffect(() => {
    return () => {
      if (topicRef.current) {
        topicRef.current.unadvertise();
        topicRef.current = null;
      }
    };
  }, []);

  return {
    publishInitialPose,
    isConnected: connection.online
  };
}