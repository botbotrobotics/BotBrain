'use client';

import { useState, useEffect } from 'react';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { Odometry } from '@/interfaces/ros/Odometry';
import { Pose, Vector3 } from 'roslib';

export default function useOdometry(useDummy: boolean = false): Odometry {
  const { connection } = useRobotConnection();

  const [odometry, setOdometry] = useState<Odometry>({
    header: {
      frame_id: '',
      seq: 0,
      stamp: {
        nanosec: 0,
        sec: 0,
      },
    },
    child_frame_id: '',
    pose: {
      pose: new Pose(),
      covariance: [],
    },
    twist: {
      twist: {
        linear: new Vector3(),
        angular: new Vector3(),
      },
      covariance: [],
    },
  });

  useEffect(() => {
    if (!connection.ros || !connection.online) return;

    // Debug logs to verify connection
    console.debug('Setting up odometry subscription, connection:', 
      connection.online ? 'online' : 'offline');

    const topicFactory: ROSTopicFactory = new ROSTopicFactory(
      connection.ros,
      useDummy
    );
    
    try {
      const odometryTopic = topicFactory.createAndSubscribeTopic<Odometry>(
        'odometry',
        (msg) => {
          // Validate odometry data before updating state
          if (msg && msg.pose && msg.pose.pose) {
            const { position, orientation } = msg.pose.pose;
            
            // Basic validation to ensure data integrity
            if (position && orientation && 
                typeof position.x === 'number' && 
                typeof position.y === 'number' && 
                typeof position.z === 'number' &&
                typeof orientation.x === 'number' && 
                typeof orientation.y === 'number' && 
                typeof orientation.z === 'number' && 
                typeof orientation.w === 'number') {
              
              // Log the first few messages to help debugging
              if (msg.header.seq < 5) {
                console.debug('Odometry data received:', {
                  seq: msg.header.seq,
                  position: `x: ${position.x.toFixed(2)}, y: ${position.y.toFixed(2)}, z: ${position.z.toFixed(2)}`,
                });
              }
              
              // Set the odometry state with valid data
              setOdometry(msg);
            } else {
              console.warn('Received invalid odometry data (NaN or missing fields):', msg);
            }
          } else {
            console.warn('Received malformed odometry message:', msg);
          }
        }
      );

      // Cleanup: unsubscribe from the topic when the component is unmounted or the connection changes
      return () => {
        console.debug('Unsubscribing from odometry topic');
        odometryTopic.unsubscribe();
      };
    } catch (error) {
      console.error('Error setting up odometry subscription:', error);
      return () => {}; // Return empty cleanup function
    }
  }, [connection.ros, connection.online, useDummy]);

  return odometry;
}
