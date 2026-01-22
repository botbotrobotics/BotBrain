'use client';

import { useState, useEffect } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import RobotProfileData from '@/interfaces/RobotProfileData';

export default function useRobotProfile(
  _useDummy: boolean = false
): RobotProfileData {
  const { connection } = useRobotConnection();

  const [profile, setProfile] = useState<RobotProfileData>({
    name: connection.connectedRobot?.name || 'Robot',
    avatar: 'mmx.jpg',
  });

  useEffect(() => {
    // Update profile when connected robot changes
    if (connection.connectedRobot) {
      setProfile({
        name: connection.connectedRobot.name,
        avatar: 'mmx.jpg',
      });
    }
  }, [connection.connectedRobot]);

  useEffect(() => {
    if (!connection.ros || !connection.online) return;

    // const topicFactory: ROSTopicFactory = new ROSTopicFactory(
    //   connection.ros,
    //   useDummy
    // );
    // const velocityTopic = topicFactory.createAndSubscribeTopic<Twist>(
    //   'velocity',
    //   (msg) => {
    //     setVelocity(msg);
    //   }
    // );

    // Cleanup: desinscreve do tópico quando o componente é desmontado ou a conexão muda
    return () => {
      // velocityTopic.unsubscribe();
    };
  }, [connection.ros, connection.online]);

  return profile;
}
