'use client';

import { useEffect, useRef, useState, useCallback } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { BatteryState } from '@/interfaces/ros/BatteryState';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';

const DEFAULT_BATTERY_STATE: BatteryState = {
  header: {
    frame_id: '',
    seq: 0,
    stamp: {
      nanosec: 0,
      sec: 0,
    },
  },
  voltage: 0,
  current: 0,
  charge: 0,
  capacity: 0,
  design_capacity: 0,
  percentage: 0,
  power_supply_status: 0,
  power_supply_health: 0,
  power_supply_technology: 0,
  present: false,
  cell_voltage: [],
  cell_temperature: [],
  location: '',
  serial_number: '',
};

export default function useRobotBatteryState(
  _useDummy: boolean = false
): BatteryState {
  const { connection } = useRobotConnection();
  const batteryTopicRef = useRef<any>(null);
  const connectionTimerRef = useRef<NodeJS.Timeout | null>(null);
  const [batteryState, setBatteryState] = useState<BatteryState>(DEFAULT_BATTERY_STATE);

  // Clean up function for real subscriptions
  const cleanupSubscriptions = useCallback(() => {
    if (batteryTopicRef.current) {
      batteryTopicRef.current.unsubscribe();
      batteryTopicRef.current = null;
    }

    if (connectionTimerRef.current) {
      clearTimeout(connectionTimerRef.current);
      connectionTimerRef.current = null;
    }
  }, []);

  useEffect(() => {
    cleanupSubscriptions();

    if (connection.ros && connection.online) {
      // Wait for stable connection before subscribing
      connectionTimerRef.current = setTimeout(() => {
        if (connection.ros) {
          const topicFactory = new ROSTopicFactory(connection.ros, _useDummy);
          const batteryStateTopic =
            topicFactory.createAndSubscribeTopic<BatteryState>(
              'battery',
              (msg) => {
                // Update state - React will handle re-renders
                setBatteryState(msg);
              }
            );

          batteryTopicRef.current = batteryStateTopic;
        }
      }, 1000);
    } else {
      // Reset to default when disconnected
      setBatteryState(DEFAULT_BATTERY_STATE);
    }

    return () => {
      cleanupSubscriptions();
    };
  }, [connection.ros, connection.online, _useDummy, cleanupSubscriptions]);

  return batteryState;
}
