'use client';

import { useState, useEffect, useRef } from 'react';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

interface CompressedImage {
  format: string;
  data: number[];
}

/**
 * Custom hook for camera streams with aggressive optimization for
 * real-time performance on poor connections.
 * 
 * @param useDummy Whether to use dummy data
 * @returns The latest compressed image data
 */
export default function useCameraStream(useDummy: boolean = false): {
  imageData: string | null;
  isLoading: boolean;
  error: string | null;
} {
  const { connection } = useRobotConnection();
  const [imageData, setImageData] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const topicRef = useRef<ROSLIB.Topic<unknown> | null>(null);
  
  // Frame rate control
  const lastFrameTime = useRef<number>(0);
  const targetFPS = 15; // Target 15fps max on poor connections
  const frameInterval = 1000 / targetFPS;

  useEffect(() => {
    if (!connection.ros || !connection.online) {
      setIsLoading(true);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const topicFactory = new ROSTopicFactory(connection.ros, useDummy);
      
      // Create topic with specialized settings for camera stream
      const topic = topicFactory.createAndSubscribeHighFrequencyTopic<CompressedImage>(
        'camera',
        (msg) => {
          const now = Date.now();
          // Limit processing rate to target FPS to prevent overwhelm on poor connections
          if (now - lastFrameTime.current < frameInterval) {
            return;
          }
          
          try {
            lastFrameTime.current = now;
            
            // Process image data
            if (msg && msg.data && msg.format) {
              // Convert array to Uint8Array
              const uint8Array = new Uint8Array(msg.data);
              
              // Convert to base64 for display
              const base64 = btoa(
                uint8Array.reduce((data, byte) => data + String.fromCharCode(byte), '')
              );
              
              setImageData(`data:image/${msg.format};base64,${base64}`);
              setIsLoading(false);
            }
          } catch (err) {
            console.error('Error processing camera frame:', err);
            // Don't set error state for individual frames to avoid flickering
          }
        }
      );
      
      topicRef.current = topic;
      
    } catch (err) {
      console.error('Error setting up camera stream:', err);
      setError('Failed to connect to camera stream');
      setIsLoading(false);
    }

    return () => {
      if (topicRef.current) {
        topicRef.current.unsubscribe();
        topicRef.current = null;
      }
    };
  }, [connection.ros, connection.online, useDummy]);

  return { imageData, isLoading, error };
} 