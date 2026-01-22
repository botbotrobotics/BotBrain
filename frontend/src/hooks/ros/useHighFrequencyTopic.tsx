'use client';

import { useState, useEffect, useRef } from 'react';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

/**
 * Custom hook for subscribing to high-frequency ROS topics (like camera streams)
 * with optimized performance settings for low-latency real-time data.
 * 
 * @param topicKey The topic key to subscribe to
 * @param useDummy Whether to use dummy data
 * @returns The latest message from the topic
 */
export default function useHighFrequencyTopic<T>(
  topicKey: keyof typeof import('@/utils/ros/topics-and-services').topicsMessages,
  useDummy: boolean = false
): T | null {
  const { connection } = useRobotConnection();
  const [message, setMessage] = useState<T | null>(null);
  const topicRef = useRef<ROSLIB.Topic<unknown> | null>(null);
  
  // Track if component is mounted to avoid state updates after unmount
  const isMounted = useRef<boolean>(true);
  
  useEffect(() => {
    isMounted.current = true;
    return () => { isMounted.current = false; };
  }, []);

  useEffect(() => {
    if (!connection.ros || !connection.online) return;

    const topicFactory = new ROSTopicFactory(connection.ros, useDummy);
    
    // Use the specialized high-frequency topic subscription
    const topic = topicFactory.createAndSubscribeHighFrequencyTopic<T>(
      topicKey,
      (msg) => {
        if (isMounted.current) {
          // Use immediate state update for low latency
          setMessage(msg);
        }
      }
    );
    
    topicRef.current = topic;

    // Cleanup on unmount or connection change
    return () => {
      if (topicRef.current) {
        topicRef.current.unsubscribe();
        topicRef.current = null;
      }
    };
  }, [connection.ros, connection.online, topicKey, useDummy]);

  return message;
} 