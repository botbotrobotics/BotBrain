'use client';

import { useState, useEffect, useRef } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { OccupancyGrid } from '@/interfaces/ros/OccupancyGrid';
import * as ROSLIB from 'roslib';

interface UseOccupancyGridOptions {
  topicName?: string;
  enabled?: boolean;
  throttleMs?: number; // Throttle updates (default: 0 = no throttle)
}

export default function useOccupancyGrid(options?: UseOccupancyGridOptions) {
  const { topicName = '/map', enabled = true, throttleMs = 0 } = options || {};
  const { connection } = useRobotConnection();

  const [occupancyGrid, setOccupancyGrid] = useState<OccupancyGrid | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [retryCount, setRetryCount] = useState(0);
  const lastUpdateRef = useRef<number>(0);
  const hasReceivedDataRef = useRef(false);

  useEffect(() => {
    if (!connection.ros || !connection.online || !enabled) {
      setIsLoading(false);
      return;
    }

    setIsLoading(true);
    setError(null);
    hasReceivedDataRef.current = false;

    const topic = new ROSLIB.Topic({
      ros: connection.ros,
      name: topicName,
      messageType: 'nav_msgs/OccupancyGrid'
    });

    const handleMessage = (message: any) => {
      try {
        // Apply throttling if configured
        if (throttleMs > 0) {
          const now = Date.now();
          if (now - lastUpdateRef.current < throttleMs) {
            return; // Skip this update
          }
          lastUpdateRef.current = now;
        }

        setOccupancyGrid({
          header: message.header,
          info: message.info,
          data: message.data
        });
        hasReceivedDataRef.current = true;
        setIsLoading(false);
        setError(null);
      } catch (err) {
        console.error('Error processing OccupancyGrid message:', err);
        setError('Failed to process map data');
        setIsLoading(false);
      }
    };

    topic.subscribe(handleMessage);

    const timeout = setTimeout(() => {
      if (!hasReceivedDataRef.current) {
        setError(`No map data received from ${topicName}`);
        setIsLoading(false);
        // Trigger retry after a delay
        if (retryCount < 3) {
          setTimeout(() => {
            setRetryCount(prev => prev + 1);
          }, 2000);
        }
      }
    }, 15000);

    return () => {
      topic.unsubscribe();
      clearTimeout(timeout);
    };
  }, [connection.ros, connection.online, topicName, enabled, retryCount, throttleMs]);

  const retry = () => {
    setError(null);
    setIsLoading(true);
    setRetryCount(prev => prev + 1);
  };

  return { occupancyGrid, isLoading, error, retry };
}