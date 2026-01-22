'use client';

import { useState, useEffect } from 'react';
import * as ROSLIB from 'roslib';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';

export interface NavPlan {
  header: {
    frame_id: string;
    seq: number;
    stamp: {
      sec: number;
      nanosec: number;
    };
  };
  poses: Array<{
    header: {
      frame_id: string;
      seq: number;
      stamp: {
        sec: number;
        nanosec: number;
      };
    };
    pose: {
      position: {
        x: number;
        y: number;
        z: number;
      };
      orientation: {
        x: number;
        y: number;
        z: number;
        w: number;
      };
    };
  }>;
}

export default function useRosNavPlan(topicName: string = '/plan') {
  const { connection } = useRobotConnection();
  const [navPlan, setNavPlan] = useState<NavPlan | null>(null);
  const [isConnected, setIsConnected] = useState(false);

  useEffect(() => {
    if (!connection.ros || !connection.online) {
      setIsConnected(false);
      setNavPlan(null);
      return;
    }

    console.debug('Setting up nav plan subscription to:', topicName);

    // Create a direct ROSLIB topic for custom topic path
    const planTopic = new ROSLIB.Topic({
      ros: connection.ros,
      name: topicName,
      messageType: 'nav_msgs/msg/Path',
      compression: 'cbor',
      throttle_rate: 0,
      queue_size: 1,
    });

    planTopic.subscribe((msg: unknown) => {
      const navMsg = msg as NavPlan;
      console.debug('Received nav plan with', navMsg.poses.length, 'poses');
      setNavPlan(navMsg);
      setIsConnected(true);
    });

    return () => {
      console.debug('Cleaning up nav plan subscription');
      planTopic.unsubscribe();
    };
  }, [connection.ros, connection.online, topicName]);

  return { navPlan, isConnected };
}