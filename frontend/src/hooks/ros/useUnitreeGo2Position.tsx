'use client';

import { useState, useEffect } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

export interface RobotPose {
  x: number;
  y: number;
  z: number;
  theta: number;
  timestamp?: number;
}

interface UseUnitreeGo2PositionOptions {
  enabled?: boolean;
}

export default function useUnitreeGo2Position(options?: UseUnitreeGo2PositionOptions) {
  const { enabled = true } = options || {};
  const { connection } = useRobotConnection();

  const [pose, setPose] = useState<RobotPose | null>(null);

  useEffect(() => {
    if (!connection.ros || !connection.online || !enabled) return;

    // List available topics to help debug (only once)
    const listTopics = () => {
      try {
        const topicsClient = new ROSLIB.Service({
          ros: connection.ros!,
          name: '/rosapi/topics',
          serviceType: 'rosapi/Topics'
        });

        topicsClient.callService({}, (result: any) => {
          if (result && result.topics) {
            console.log('Available ROS topics:', result.topics);
            const unitreeTopics = result.topics.filter((topic: string) => 
              topic.toLowerCase().includes('sport') || 
              topic.toLowerCase().includes('odom') ||
              topic.toLowerCase().includes('state') ||
              topic.toLowerCase().includes('unitree') ||
              topic.toLowerCase().includes('go2')
            );
            console.log('Unitree-related topics:', unitreeTopics);
          }
        }, (error: any) => {
          // Silently fail if rosapi is not available
          console.debug('rosapi service not available');
        });
      } catch (err) {
        // Silently fail
        console.debug('Could not create rosapi service client');
      }
    };

    // Only list topics once after a short delay
    const timeoutId = setTimeout(listTopics, 1000);

    // Try multiple possible topic names for Unitree Go2
    const topics = [
      { name: '/sportmodestate', type: 'unitree_go/msg/SportModeState' },
      { name: '/rt/sportmodestate', type: 'unitree_go/msg/SportModeState' },
      { name: '/api/sport/state', type: 'unitree_api/msg/SportModeState' },
      // Additional possible topics based on different SDKs
      { name: '/go2/sportmodestate', type: 'unitree_go/msg/SportModeState' },
      { name: '/unitree/sportmodestate', type: 'unitree_go/msg/SportModeState' }
    ];

    const subscriptions: ROSLIB.Topic<unknown>[] = [];
    let activeTopicName: string | null = null;

    topics.forEach(({ name, type }) => {
      try {
        // Try with empty message type first to allow auto-detection
        const topic = new ROSLIB.Topic({
          ros: connection.ros!,
          name: name,
          messageType: type
        });

        const handleMessage = (message: any) => {
          // If we already have an active topic and this isn't it, ignore
          if (activeTopicName && activeTopicName !== name) return;
          
          try {
            // Extract position from SportModeState
            let x = 0, y = 0, z = 0, theta = 0;
            
            if (message.position && Array.isArray(message.position)) {
              // Position is float32[3] array
              x = message.position[0] || 0;
              y = message.position[1] || 0;
              z = message.position[2] || 0;
            }
            
            // Try to get yaw from IMU state if available
            if (message.imu_state && message.imu_state.rpy) {
              // rpy[2] is yaw
              theta = message.imu_state.rpy[2] || 0;
            } else if (message.yaw) {
              theta = message.yaw;
            }

            setPose({ x, y, z, theta, timestamp: Date.now() });
            
            // Set this as the active topic if it's the first one
            if (!activeTopicName) {
              activeTopicName = name;
              console.log(`Using topic ${name} for robot position`);
              
              // Unsubscribe from other topics
              subscriptions.forEach((sub) => {
                if (sub !== topic) {
                  try {
                    sub.unsubscribe();
                  } catch (e) {
                    // Ignore unsubscribe errors
                  }
                }
              });
            }
          } catch (err) {
            // Silently fail for this topic and let others try
          }
        };

        topic.subscribe(handleMessage);
        subscriptions.push(topic);
      } catch (err) {
        // Failed to create topic, skip this one
      }
    });

    // Also try standard odometry topics as fallback
    try {
      const odomTopic = new ROSLIB.Topic({
        ros: connection.ros!,
        name: '/odom',
        messageType: 'nav_msgs/Odometry'
      });

      odomTopic.subscribe((message: any) => {
        // If we already have an active topic and this isn't /odom, ignore
        if (activeTopicName && activeTopicName !== '/odom') return;
        
        try {
          const position = message.pose.pose.position;
          const orientation = message.pose.pose.orientation;
          
          // Convert quaternion to yaw
          const theta = Math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
          );

          setPose({
            x: position.x,
            y: position.y,
            z: position.z,
            theta,
            timestamp: Date.now()
          });
          
          // Set this as the active topic if it's the first one
          if (!activeTopicName) {
            activeTopicName = '/odom';
            console.log('Using topic /odom for robot position');
          }
        } catch (err) {
          // Silently fail
        }
      });

      subscriptions.push(odomTopic);
    } catch (err) {
      // Failed to create odometry topic
    }

    return () => {
      clearTimeout(timeoutId);
      subscriptions.forEach(topic => topic.unsubscribe());
    };
  }, [connection.ros, connection.online, enabled]);

  return pose;
}