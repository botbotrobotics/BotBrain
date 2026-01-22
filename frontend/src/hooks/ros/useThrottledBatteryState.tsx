'use client';

import { useState, useEffect, useRef } from 'react';
import useRobotBatteryState from './useRobotBatteryState';
import { BatteryState } from '@/interfaces/ros/BatteryState';

const THROTTLE_INTERVAL = 1000; // Update every 1 second (1Hz) - good balance for UI updates

/**
 * Throttled version of battery state hook to reduce re-renders
 * While ROS may publish at 20Hz, UI only needs ~1Hz updates for battery
 */
export default function useThrottledBatteryState(): BatteryState {
  const rawBatteryState = useRobotBatteryState();
  const [throttledState, setThrottledState] = useState<BatteryState>(rawBatteryState);
  const lastUpdateRef = useRef<number>(0);
  const pendingUpdateRef = useRef<BatteryState | null>(null);

  useEffect(() => {
    const now = Date.now();
    const timeSinceLastUpdate = now - lastUpdateRef.current;

    if (timeSinceLastUpdate >= THROTTLE_INTERVAL) {
      // Enough time has passed, update immediately
      setThrottledState(rawBatteryState);
      lastUpdateRef.current = now;
      pendingUpdateRef.current = null;
    } else {
      // Store pending update and schedule it
      pendingUpdateRef.current = rawBatteryState;

      const timeUntilNextUpdate = THROTTLE_INTERVAL - timeSinceLastUpdate;
      const timer = setTimeout(() => {
        if (pendingUpdateRef.current) {
          setThrottledState(pendingUpdateRef.current);
          lastUpdateRef.current = Date.now();
          pendingUpdateRef.current = null;
        }
      }, timeUntilNextUpdate);

      return () => clearTimeout(timer);
    }
  }, [rawBatteryState]);

  return throttledState;
}