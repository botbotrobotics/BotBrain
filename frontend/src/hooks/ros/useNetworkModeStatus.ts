'use client';

import { useEffect, useRef, useState } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

export type NetworkMode = 'wifi' | '4g' | 'hotspot' | 'offline';

export interface NetworkModeStatus {
  mode: NetworkMode;
  ssid?: string;
  interface?: string;
  timestamp?: Date;
  rawMessage?: string;
}

export default function useNetworkModeStatus(): NetworkModeStatus {
  const { connection } = useRobotConnection();
  const [status, setStatus] = useState<NetworkModeStatus>({
    mode: 'offline',
  });
  const topicRef = useRef<ROSLIB.Topic<unknown> | null>(null);

  useEffect(() => {
    if (!connection.online || !connection.ros) {
      setStatus({ mode: 'offline' });
      if (topicRef.current) {
        topicRef.current.unsubscribe();
        topicRef.current = null;
      }
      return;
    }

    // Subscribe immediately when connection is available
    if (!topicRef.current && connection.ros) {
      // Create a custom topic for network mode status
      topicRef.current = new ROSLIB.Topic({
        ros: connection.ros,
        name: '/network_mode_status',
        messageType: 'std_msgs/String',
        compression: 'cbor',
        throttle_rate: 0,
        queue_size: 1,
      });

      const handleMessage = (message: any) => {
        if (!message?.data) return;
        
        const data = message.data as string;
        setStatus(parseNetworkStatus(data));
      };

      topicRef.current.subscribe(handleMessage);
    }

    return () => {
      if (topicRef.current) {
        topicRef.current.unsubscribe();
        topicRef.current = null;
      }
    };
  }, [connection.online, connection.ros]);

  return status;
}

function parseNetworkStatus(data: string): NetworkModeStatus {
  const status: NetworkModeStatus = {
    mode: 'offline',
    rawMessage: data,
  };

  // Parse timestamp if present
  const timestampMatch = data.match(/^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})/);
  if (timestampMatch) {
    status.timestamp = new Date(timestampMatch[1]);
  }

  // Parse the status description
  if (data.includes('WiFi:')) {
    status.mode = 'wifi';
    const ssidMatch = data.match(/WiFi:\s*(.+)$/);
    if (ssidMatch) {
      status.ssid = ssidMatch[1].trim();
    }
  } else if (data.includes('4G:')) {
    status.mode = '4g';
    const interfaceMatch = data.match(/4G:\s*(.+)$/);
    if (interfaceMatch) {
      status.interface = interfaceMatch[1].trim();
    }
  } else if (data.includes('Hotspot:')) {
    status.mode = 'hotspot';
    const ssidMatch = data.match(/Hotspot:\s*(.+)$/);
    if (ssidMatch) {
      status.ssid = ssidMatch[1].trim();
    }
  } else if (data.includes('Offline:')) {
    status.mode = 'offline';
  }

  return status;
}