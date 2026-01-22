'use client';

import { useState, useEffect, useRef, useCallback } from 'react';
import * as ROSLIB from 'roslib';
import { Database } from '@/types/database.types';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';

type Robot = Database['public']['Tables']['robots']['Row'];

export type RobotStatus = 'available' | 'unavailable' | 'checking';

interface RobotAvailability {
  robotId: string;
  status: RobotStatus;
  lastChecked: Date | null;
}

interface UseRobotAvailabilityOptions {
  checkInterval?: number;
  timeout?: number;
}

const DEFAULT_CHECK_INTERVAL = 30000;
const DEFAULT_TIMEOUT = 2000;

const createTestConnection = (address: string): ROSLIB.Ros => {
  let cleanAddress = address.startsWith('@') ? address.substring(1) : address;
  let url = cleanAddress;

  if (!url.startsWith('ws://') && !url.startsWith('wss://') && !url.startsWith('http://') && !url.startsWith('https://')) {
    url = `ws://${url}:9090`;
  } else if (url.startsWith('http://')) {
    url = url.replace('http://', 'ws://');
    if (!url.includes(':')) {
      url = `${url}:9090`;
    }
  } else if (url.startsWith('https://')) {
    url = url.replace('https://', 'wss://');
    if (!url.includes(':')) {
      url = `${url}:9090`;
    }
  } else if ((url.startsWith('ws://') && !url.includes(':', 5)) ||
           (url.startsWith('wss://') && !url.includes(':', 6))) {
    url = `${url}:9090`;
  }

  const options: any = {
    url,
    transportLibrary: 'websocket',
    groovyCompatibility: false,
    encoding: 'cbor',
  };

  return new ROSLIB.Ros(options);
};

async function checkSingleRobot(
  robot: Robot,
  timeout: number,
  connectedRobotId?: string,
  isOnline?: boolean
): Promise<RobotAvailability> {
  // If this robot is currently connected, it's definitely available
  if (connectedRobotId === robot.id && isOnline) {
    return {
      robotId: robot.id,
      status: 'available',
      lastChecked: new Date()
    };
  }

  return new Promise((resolve) => {
    let resolved = false;
    let ros: ROSLIB.Ros | null = null;

    const cleanup = () => {
      if (ros) {
        try {
          ros.close();
        } catch (e) {
          // Ignore close errors
        }
        ros = null;
      }
    };

    const resolveStatus = (status: RobotStatus) => {
      if (!resolved) {
        resolved = true;
        cleanup();
        resolve({
          robotId: robot.id,
          status,
          lastChecked: new Date()
        });
      }
    };

    const timeoutId = setTimeout(() => {
      console.log(`Timeout checking robot ${robot.name} at ${robot.address}`);
      resolveStatus('unavailable');
    }, timeout);

    try {
      ros = createTestConnection(robot.address);

      ros.on('connection', () => {
        console.log(`Robot ${robot.name} is available at ${robot.address}`);
        clearTimeout(timeoutId);
        resolveStatus('available');
      });

      ros.on('error', (error) => {
        console.log(`Robot ${robot.name} connection error at ${robot.address}:`, error);
        clearTimeout(timeoutId);
        resolveStatus('unavailable');
      });

      // Also handle close event which might fire before connection
      ros.on('close', () => {
        if (!resolved) {
          console.log(`Robot ${robot.name} connection closed at ${robot.address}`);
          clearTimeout(timeoutId);
          resolveStatus('unavailable');
        }
      });
    } catch (error) {
      console.error(`Error creating connection for robot ${robot.name}:`, error);
      clearTimeout(timeoutId);
      resolveStatus('unavailable');
    }
  });
}

export function useRobotAvailability(
  robots: Robot[],
  options: UseRobotAvailabilityOptions = {}
) {
  const {
    checkInterval = DEFAULT_CHECK_INTERVAL,
    timeout = DEFAULT_TIMEOUT
  } = options;

  const { connection } = useRobotConnection();
  const [availability, setAvailability] = useState<Map<string, RobotAvailability>>(
    new Map()
  );
  const [isChecking, setIsChecking] = useState(false);

  // Use refs to avoid dependency issues
  const robotsRef = useRef<Robot[]>(robots);
  const connectionRef = useRef(connection);
  const checkingRef = useRef(false);
  const intervalRef = useRef<NodeJS.Timeout | null>(null);

  // Update refs when values change
  useEffect(() => {
    robotsRef.current = robots;
  }, [robots]);

  useEffect(() => {
    connectionRef.current = connection;
  }, [connection]);

  // Stable check function that uses refs
  const performCheck = useCallback(async (showCheckingState = true) => {
    // Use ref values to avoid dependency issues
    const currentRobots = robotsRef.current;
    const currentConnection = connectionRef.current;

    if (currentRobots.length === 0) return;

    // Prevent concurrent checks
    if (checkingRef.current) {
      console.log('Already checking, skipping...');
      return;
    }

    checkingRef.current = true;
    setIsChecking(true);
    console.log(`Starting availability check for ${currentRobots.length} robots`);

    // Only show checking state if requested (not on initial load)
    if (showCheckingState) {
      setAvailability(prev => {
        const newMap = new Map(prev);
        currentRobots.forEach(robot => {
          // Don't change status of currently connected robot to checking
          if (currentConnection.connectedRobot?.id === robot.id && currentConnection.online) {
            newMap.set(robot.id, {
              robotId: robot.id,
              status: 'available',
              lastChecked: new Date()
            });
          } else {
            // Set to checking state
            newMap.set(robot.id, {
              robotId: robot.id,
              status: 'checking',
              lastChecked: prev.get(robot.id)?.lastChecked || null
            });
          }
        });
        return newMap;
      });
    }

    try {
      // Check all robots in parallel
      const results = await Promise.allSettled(
        currentRobots.map(robot =>
          checkSingleRobot(
            robot,
            timeout,
            currentConnection.connectedRobot?.id,
            currentConnection.online
          )
        )
      );

      // Update availability with results
      setAvailability(prev => {
        const newMap = new Map(prev);
        results.forEach((result, index) => {
          const robot = currentRobots[index];
          if (result.status === 'fulfilled') {
            newMap.set(robot.id, result.value);
          } else {
            newMap.set(robot.id, {
              robotId: robot.id,
              status: 'unavailable',
              lastChecked: new Date()
            });
          }
        });
        return newMap;
      });

      console.log('Availability check completed');
    } catch (error) {
      console.error('Error checking robot availability:', error);
    } finally {
      checkingRef.current = false;
      setIsChecking(false);
    }
  }, [timeout]); // Only depends on timeout which rarely changes

  // Manual refresh function (shows checking state)
  const refreshAvailability = useCallback(() => {
    performCheck(true);
  }, [performCheck]);

  // Initialize availability state for new robots
  useEffect(() => {
    if (robots.length > 0) {
      setAvailability(prev => {
        const newMap = new Map(prev);
        robots.forEach(robot => {
          // Only add if not already in the map
          if (!newMap.has(robot.id)) {
            // Check if this is the connected robot
            if (connection.connectedRobot?.id === robot.id && connection.online) {
              newMap.set(robot.id, {
                robotId: robot.id,
                status: 'available',
                lastChecked: new Date()
              });
            } else {
              // Default to unavailable until checked
              newMap.set(robot.id, {
                robotId: robot.id,
                status: 'unavailable',
                lastChecked: null
              });
            }
          }
        });
        return newMap;
      });
    }
  }, [robots, connection.connectedRobot?.id, connection.online]);

  // Initial check and setup interval
  useEffect(() => {
    // Don't start checking until we have robots
    if (robots.length === 0) {
      return;
    }

    // Initial check without showing checking state
    const initialTimer = setTimeout(() => {
      console.log('Performing initial availability check');
      performCheck(false); // Don't show checking state on initial load
    }, 500); // Shorter delay since we're not showing checking state

    // Setup interval for periodic checks (these will show checking state)
    intervalRef.current = setInterval(() => {
      performCheck(true);
    }, checkInterval);

    return () => {
      clearTimeout(initialTimer);
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
    };
  }, [performCheck, checkInterval, robots.length]);

  // Update connected robot status immediately when connection changes
  useEffect(() => {
    if (connection.connectedRobot) {
      setAvailability(prev => {
        const newMap = new Map(prev);
        // Only update if not currently checking
        const current = newMap.get(connection.connectedRobot!.id);
        if (current?.status !== 'checking') {
          newMap.set(connection.connectedRobot!.id, {
            robotId: connection.connectedRobot!.id,
            status: connection.online ? 'available' : 'unavailable',
            lastChecked: new Date()
          });
        }
        return newMap;
      });
    }
  }, [connection.connectedRobot?.id, connection.online]);

  return {
    availability,
    isChecking,
    refreshAvailability
  };
}