'use client';

import { useState, useEffect, useRef } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import {
  ROSTopicFactory,
  topicsMessages,
} from '@/utils/ros/topics-and-services';

export interface ChartDataPoint {
  timestamp: number;
  value: number;
}

export default function useChartData(
  topic: keyof typeof topicsMessages,
  refreshTime: number,
  maxPoints: number = 20,
  updateData: boolean = true
) {
  const [data, setData] = useState<ChartDataPoint[]>([]);
  const { connection } = useRobotConnection();
  const intervalRef = useRef<NodeJS.Timeout | null>(null);

  const handleRosData = (message: any) => {
    const now = Date.now();
    let value = 0;

    // Extract relevant data based on topic type
    switch (topic) {
      case 'battery':
        value = message.percentage * 100 || message.value || 0;
        break;
      case 'temperature':
        // Average of all motor temperatures
        value = message.data || message.temperature || 0;
        break;
      case 'jointStates':
        // First joint position
        value = message.position?.[0] || 0;
        break;
      case 'robotStatus':
        // First motor RPM
        value = message.rpm?.[0] || 0;
        break;
      case 'odometry':
        // Linear velocity in x direction
        value = message.twist?.twist?.linear?.x || 0;
        break;
      case 'sportModeState':
        value = message.mode;
        break;
      case 'laserScan':
        // Average of range data (simplified)
        if (message.ranges && message.ranges.length > 0) {
          const validRanges = message.ranges.filter(
            (r: number) => r >= message.range_min && r <= message.range_max
          );
          value =
            validRanges.length > 0
              ? validRanges.reduce((sum: number, r: number) => sum + r, 0) /
                validRanges.length
              : 0;
        }
        break;
      default:
        value = 0;
    }

    setData((prevData) => {
      const newData = [...prevData, { timestamp: now, value }];

      if (newData.length > maxPoints) {
        return newData.slice(newData.length - maxPoints);
      }

      return newData;
    });
  };

  useEffect(() => {
    if (!updateData) return;

    // Clear existing interval
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
    }

    let topicSubscription: any = null;

    // Only process data if connected to robot
    if (connection.online && connection.ros) {
      try {
        // Get the correct topic key for our topics-and-services utility
        const topicFactory = new ROSTopicFactory(connection.ros);
        topicSubscription = topicFactory.createAndSubscribeTopic(
          topic,
          handleRosData
        );
      } catch (error) {
        console.error('Error subscribing to ROS topic:', error);
      }
    }

    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
      if (topicSubscription) {
        topicSubscription.unsubscribe();
      }
    };
  }, [updateData, refreshTime, topic, connection.online, connection.ros]);

  return data;
}
