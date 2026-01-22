'use client';

import { useState, useEffect } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import useRobotStatus from './useRobotStatus';

export default function useRobotPose(_useDummy: boolean = false): boolean {
  const { connection } = useRobotConnection();
  const [isPose, setIsPose] = useState(false);
  const { robotStatus } = useRobotStatus();

  useEffect(() => {
    if (!connection.ros || !connection.online) return;

    setIsPose(robotStatus === 'pose');
  }, [connection.ros, connection.online, robotStatus]);

  return isPose;
}
