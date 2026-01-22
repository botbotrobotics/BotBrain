'use client';

import { useState, useEffect } from 'react';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { LaserScan } from '@/interfaces/ros/LaserScan';

export default function useLaserScan(useDummy: boolean = false): LaserScan {
  const { connection } = useRobotConnection();

  const [laserScan, setLaserScan] = useState<LaserScan>({
    angle_increment: 0,
    angle_max: 0,
    angle_min: 0,
    header: {
      frame_id: '',
      seq: 0,
      stamp: {
        nanosec: 0,
        sec: 0,
      },
    },
    range_max: 0,
    scan_time: 0,
    time_increment: 0,
    range_min: 0,
    intensities: [],
    ranges: [],
  });

  useEffect(() => {
    if (!connection.ros || !connection.online) return;

    const topicFactory: ROSTopicFactory = new ROSTopicFactory(
      connection.ros,
      useDummy
    );
    const laserScanTopic = topicFactory.createAndSubscribeTopic<LaserScan>(
      'laserScan',
      (msg) => {
        setLaserScan(msg);
      }
    );

    // Cleanup: desinscreve do tópico quando o componente é desmontado ou a conexão muda
    return () => {
      laserScanTopic.unsubscribe();
    };
  }, [connection.ros, connection.online]);

  return laserScan;
}
