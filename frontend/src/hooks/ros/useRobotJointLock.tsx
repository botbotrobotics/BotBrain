'use client';

import { useState, useEffect } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import useRobotStatus from './useRobotStatus';

export default function useRobotJointLock(_useDummy: boolean = false): boolean {
  const { connection } = useRobotConnection();
  const [isJointLock, setIsJointLock] = useState(false);
  const { robotStatus } = useRobotStatus();

  useEffect(() => {
    if (!connection.ros || !connection.online) return;

    setIsJointLock(robotStatus === 'jointLock');
  }, [connection.ros, connection.online, robotStatus]);

  return isJointLock;
}
