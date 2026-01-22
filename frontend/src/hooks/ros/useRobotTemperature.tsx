'use client';

import { useState, useEffect } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { Temperature } from '@/interfaces/ros/Temperature';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';

export default function useRobotTemperature(
  _useDummy: boolean = false
): Temperature {
  const { connection } = useRobotConnection();

  const [temperature, setTemperature] = useState<Temperature>({
    data: 0,
  });

  useEffect(() => {
    if (!connection.ros || !connection.online) return;

    const topicFactory: ROSTopicFactory = new ROSTopicFactory(
      connection.ros,
      _useDummy
    );
    const temperatureTopic = topicFactory.createAndSubscribeTopic<Temperature>(
      'temperature',
      (msg) => {
        const currentTemp = temperature.data;
        const newTemp = msg.data;
        if (currentTemp !== newTemp) {
          setTemperature(msg);
        }
      }
    );

    // Cleanup: desinscreve do tópico quando o componente é desmontado ou a conexão muda
    return () => {
      temperatureTopic.unsubscribe();
    };
  }, [connection.ros, connection.online]);

  return temperature;
}
