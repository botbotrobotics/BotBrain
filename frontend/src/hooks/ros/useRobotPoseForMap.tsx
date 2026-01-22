'use client';

import { useState, useEffect } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

export interface RobotPose {
  x: number;
  y: number;
  theta: number;
}

interface UseRobotPoseOptions {
  topicName?: string;
  messageType?: string;
  enabled?: boolean;
}

export default function useRobotPoseForMap(options?: UseRobotPoseOptions) {
  const { 
    topicName = '/odom', 
    messageType = 'nav_msgs/Odometry',
    enabled = true 
  } = options || {};
  const { connection } = useRobotConnection();

  const [pose, setPose] = useState<RobotPose | null>(null);

  useEffect(() => {
    if (!connection.ros || !connection.online || !enabled) return;

    const topic = new ROSLIB.Topic({
      ros: connection.ros,
      name: topicName,
      messageType: messageType
    });

    const handleMessage = (message: any) => {
      try {
        let position, orientation;
        
        // Handle different message types
        if (messageType === 'nav_msgs/Odometry') {
          // Odometry message structure
          position = message.pose.pose.position;
          orientation = message.pose.pose.orientation;
        } else if (messageType === 'geometry_msgs/PoseWithCovarianceStamped') {
          // AMCL pose message structure
          position = message.pose.pose.position;
          orientation = message.pose.pose.orientation;
        } else if (messageType === 'geometry_msgs/PoseStamped') {
          // Simple pose stamped structure
          position = message.pose.position;
          orientation = message.pose.orientation;
        } else {
          console.warn('Unknown message type:', messageType);
          return;
        }
        
        // Convert quaternion to yaw (theta)
        const theta = Math.atan2(
          2 * (orientation.w * orientation.z + orientation.x * orientation.y),
          1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        );

        setPose({
          x: position.x,
          y: position.y,
          theta
        });
        
        console.log(`Received pose from ${topicName}:`, { x: position.x, y: position.y, theta });
      } catch (err) {
        console.error('Error processing pose message:', err, message);
      }
    };

    topic.subscribe(handleMessage);
    
    console.log(`Subscribed to ${topicName} with message type ${messageType}`);

    return () => {
      topic.unsubscribe();
    };
  }, [connection.ros, connection.online, topicName, messageType, enabled]);

  return pose;
}