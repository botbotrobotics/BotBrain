'use client';

import { useState, useEffect } from 'react';
import { Twist } from '@/interfaces/ros/Twist';
import { Vector3 } from 'roslib';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import useJoystickMove from '../useJoystickMove';

export default function useRobotVelocity(useDummy: boolean = false): Twist {
  const { connection } = useRobotConnection();
  const { joyVel } = useJoystickMove();

  const [velocity, setVelocity] = useState<Twist>({
    angular: new Vector3(),
    linear: new Vector3(),
  });

  useEffect(() => {
    if (connection.ros && connection.online) {
      const topicFactory: ROSTopicFactory = new ROSTopicFactory(
        connection.ros,
        useDummy
      );

      const velocityTopic = topicFactory.createAndSubscribeTopic<Twist>(
        'velocity',
        (vel) => {
          setVelocity(vel);
        }
      );

      const nippleVelTopic = topicFactory.createAndSubscribeTopic<Twist>(
        'velocityNipple',
        (vel) => {
          setVelocity({
            ...vel,
            linear: new Vector3({ x: -vel.linear.y, y: vel.linear.x, z: 0 }),
            angular: new Vector3({ x: 0, y: 0, z: -vel.angular.z }),
          });
          // setVelocity(vel);
        }
      );

      // Cleanup: desinscreve do tópico quando o componente é desmontado ou a conexão muda
      return () => {
        velocityTopic.unsubscribe();
        nippleVelTopic.unsubscribe();
      };
    } else {
      setVelocity({
        angular: new Vector3(),
        linear: new Vector3({ x: joyVel.x, y: joyVel.y }),
      });
    }
  }, [connection.ros, connection.online]);

  return velocity;
}
