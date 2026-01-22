'use client';

import { useState, useEffect, useRef } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

interface NetworkMetrics {
  latency: number; // in milliseconds
  dataIn: number; // in bytes per second
  dataOut: number; // in bytes per second
  packetLoss: number; // in percentage (0-100)
}

export default function useNetworkMetrics(
  refreshInterval: number = 1000
): NetworkMetrics {
  const { connection } = useRobotConnection();
  const [metrics, setMetrics] = useState<NetworkMetrics>({
    latency: -1, // Initialize with -1 to show "Measuring..." state
    dataIn: 0,
    dataOut: 0,
    packetLoss: 0,
  });

  // Refs to track message data sizes and timing
  const lastPingTime = useRef<number>(0);
  const bytesInRef = useRef<number>(0);
  const bytesOutRef = useRef<number>(0);
  const lastBytesInRef = useRef<number>(0);
  const lastBytesOutRef = useRef<number>(0);
  const metricsIntervalRef = useRef<NodeJS.Timeout | null>(null);

  // Packet loss tracking
  const totalPings = useRef<number>(0);
  const failedPings = useRef<number>(0);

  useEffect(() => {
    if (!connection.ros || !connection.online) {
      // Reset metrics when disconnected
      setMetrics({
        latency: -1,
        dataIn: 0,
        dataOut: 0,
        packetLoss: 0,
      });
      // Reset packet loss tracking
      totalPings.current = 0;
      failedPings.current = 0;
      return;
    }

    // Function to measure latency by sending a ping to the server
    const measureLatency = () => {
      try {
        if (connection.ros && connection.online) {
          lastPingTime.current = Date.now();
          totalPings.current += 1;

          // Use the getRosTime service to measure latency
          const pingService = new ROSLIB.Service({
            ros: connection.ros,
            name: '/rosapi/get_time',
            serviceType: 'rosapi/GetTime',
          });

          pingService.callService(
            {},
            () => {
              // Calculate latency as round trip time
              const latency = Date.now() - lastPingTime.current;

              // Calculate packet loss percentage
              const packetLoss = totalPings.current > 0
                ? Math.round((failedPings.current / totalPings.current) * 100)
                : 0;

              setMetrics(prev => ({ ...prev, latency, packetLoss }));
            },
            () => {
              // In case of error, set latency to -1 and increment failed pings
              failedPings.current += 1;

              // Calculate packet loss percentage
              const packetLoss = totalPings.current > 0
                ? Math.round((failedPings.current / totalPings.current) * 100)
                : 0;

              setMetrics(prev => ({ ...prev, latency: -1, packetLoss }));
            }
          );
        }
      } catch (error) {
        console.error('Error measuring latency:', error);
        failedPings.current += 1;
      }
    };

    // Subscribe to connection statistics if available
    if (connection.ros) {
      // Monkey patch the WebSocket to track data transfer
      const ws = (connection.ros as any).socket;
      
      if (ws) {
        const originalSend = ws.send;
        ws.send = function(data: any) {
          if (typeof data === 'string') {
            bytesOutRef.current += data.length;
          } else if (data instanceof ArrayBuffer) {
            bytesOutRef.current += data.byteLength;
          } else if (data instanceof Blob) {
            bytesOutRef.current += data.size;
          }
          return originalSend.apply(this, arguments);
        };

        // Track incoming messages
        const originalOnMessage = ws.onmessage;
        ws.onmessage = function(event: MessageEvent) {
          if (event.data) {
            if (typeof event.data === 'string') {
              bytesInRef.current += event.data.length;
            } else if (event.data instanceof ArrayBuffer) {
              bytesInRef.current += event.data.byteLength;
            } else if (event.data instanceof Blob) {
              bytesInRef.current += event.data.size;
            }
          }
          return originalOnMessage.apply(this, [event]);
        };
      }
    }

    // Measure metrics at the specified interval
    metricsIntervalRef.current = setInterval(() => {
      measureLatency();
      
      // Calculate data transfer rates
      const now = Date.now();
      const elapsedSec = refreshInterval / 1000;
      
      const dataIn = (bytesInRef.current - lastBytesInRef.current) / elapsedSec;
      const dataOut = (bytesOutRef.current - lastBytesOutRef.current) / elapsedSec;
      
      // Update the metrics
      setMetrics(prev => ({
        ...prev,
        dataIn,
        dataOut,
      }));
      
      // Update last values
      lastBytesInRef.current = bytesInRef.current;
      lastBytesOutRef.current = bytesOutRef.current;
    }, refreshInterval);

    // Initial latency measurement
    measureLatency();

    // Clean up on unmount or when connection changes
    return () => {
      if (metricsIntervalRef.current) {
        clearInterval(metricsIntervalRef.current);
      }
      
      // Reset the patched WebSocket methods if needed
      if (connection.ros) {
        const ws = (connection.ros as any).socket;
        if (ws && ws.send.__originalSend) {
          ws.send = ws.send.__originalSend;
        }
      }
    };
  }, [connection.ros, connection.online, refreshInterval]);

  return metrics;
} 