'use client';

import { useState, useEffect, useRef, useCallback } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';
import { Odometry } from '@/interfaces/ros/Odometry';
import { formatDecimal } from '@/utils/ros/roslib-utils';

export default function useRobotSpeed(useDummy: boolean = false): {
  speed: number;
  maxSpeed: number;
} {
  const { connection } = useRobotConnection();
  const maxSpeed = 2; // Max speed is 2 m/s for go2
  const odomTopicRef = useRef<any>(null);
  const connectionTimerRef = useRef<NodeJS.Timeout | null>(null);

  const [speed, setSpeed] = useState(0);

  // Clean up function for subscriptions
  const cleanupSubscriptions = useCallback(() => {
    if (odomTopicRef.current) {
      odomTopicRef.current.unsubscribe();
      odomTopicRef.current = null;
    }

    if (connectionTimerRef.current) {
      clearTimeout(connectionTimerRef.current);
      connectionTimerRef.current = null;
    }
  }, []);

  useEffect(() => {
    cleanupSubscriptions();

    if (connection.ros && connection.online) {
      // Wait for stable connection before subscribing
      connectionTimerRef.current = setTimeout(() => {
        if (connection.ros) {
          const topicFactory = new ROSTopicFactory(connection.ros, useDummy);

          // Use odometry topic which provides velocity in twist.twist.linear
          const odomTopic = topicFactory.createAndSubscribeTopic<Odometry>(
            'odometry',
            (msg) => {
              // Extract velocity from odometry twist message
              if (msg.twist?.twist?.linear) {
                const vx = msg.twist.twist.linear.x || 0;
                const vy = msg.twist.twist.linear.y || 0;
                const vz = msg.twist.twist.linear.z || 0;

                // Calculate the magnitude of the velocity vector (Euclidean norm)
                const totalSpeed = Math.sqrt(vx * vx + vy * vy + vz * vz);
                const formattedSpeed = Number.parseFloat(
                  formatDecimal(totalSpeed, 1)
                );
                setSpeed(formattedSpeed);
              }
            }
          );

          odomTopicRef.current = odomTopic;
        }
      }, 1000);
    } else {
      // Reset to default when disconnected
      setSpeed(0);
    }

    return () => {
      cleanupSubscriptions();
    };
  }, [connection.ros, connection.online, useDummy, cleanupSubscriptions]);

  return { speed, maxSpeed };
}
