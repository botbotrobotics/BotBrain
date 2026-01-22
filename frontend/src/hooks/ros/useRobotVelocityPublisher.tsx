'use client';

import { useMemo, useRef } from 'react';
import { Twist } from '@/interfaces/ros/Twist';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useSpeedMode } from '@/contexts/SpeedModeContext';
import { auditLogger } from '@/utils/audit-logger';
import { Vector3 } from 'roslib';

export default function useRobotVelocityPublisher(useDummy: boolean = false) {
  const { connection } = useRobotConnection();
  const { speedMultiplier, speedMode } = useSpeedMode();
  const lastLogTimeRef = useRef<number>(0);
  const isMovingRef = useRef<boolean>(false);
  const controlTypeRef = useRef<'joystick' | 'keyboard' | 'gamepad'>('joystick');

  const velocityTopic = useMemo(() => {
    if (!connection.ros || !connection.online) return null;
    const topicFactory: ROSTopicFactory = new ROSTopicFactory(
      connection.ros,
      useDummy
    );

    return topicFactory.createAndSubscribeTopic<Twist>(
      'velocityNipple',
      () => {}
    );
  }, [connection.ros, connection.online, useDummy]);

  const publishVelocity = (newVelocity: Twist, inputSource?: 'joystick' | 'keyboard' | 'gamepad') => {
    if (!velocityTopic) {
      // console.warn(
      //   'Conexão ROS não estabelecida ou offline. Não foi possível publicar.'
      // );
      return;
    }

    // Apply speed multiplier to the velocity
    const scaledVelocity: Twist = {
      linear: new Vector3({
        x: newVelocity.linear.x * speedMultiplier,
        y: newVelocity.linear.y * speedMultiplier,
        z: newVelocity.linear.z * speedMultiplier,
      }),
      angular: new Vector3({
        x: newVelocity.angular.x * speedMultiplier,
        y: newVelocity.angular.y * speedMultiplier,
        z: newVelocity.angular.z * speedMultiplier,
      }),
    };

    // Check if robot is moving (non-zero velocity after scaling)
    const isCurrentlyMoving =
      scaledVelocity.linear.x !== 0 ||
      scaledVelocity.linear.y !== 0 ||
      scaledVelocity.linear.z !== 0 ||
      scaledVelocity.angular.x !== 0 ||
      scaledVelocity.angular.y !== 0 ||
      scaledVelocity.angular.z !== 0;

    // Throttle audit logging to once per second when moving
    const now = Date.now();
    const timeSinceLastLog = now - lastLogTimeRef.current;

    // Update control type if provided
    if (inputSource) {
      controlTypeRef.current = inputSource;
    }

    // Log when starting movement, every second during movement, or when stopping
    if (isCurrentlyMoving && (!isMovingRef.current || timeSinceLastLog > 1000)) {
      // Starting movement or periodic log during movement
      const action = controlTypeRef.current === 'keyboard' ? 'keyboard_control' :
                     controlTypeRef.current === 'gamepad' ? 'gamepad_control' :
                     'joystick_control';

      auditLogger.log({
        event_type: 'command',
        event_action: action,
        robot_id: connection.connectedRobot?.id,
        robot_name: connection.connectedRobot?.name,
        event_details: {
          linear_x: scaledVelocity.linear.x.toFixed(2),
          linear_y: scaledVelocity.linear.y.toFixed(2),
          angular_z: scaledVelocity.angular.z.toFixed(2),
          speed_mode: speedMode,
          speed_multiplier: speedMultiplier,
        }
      });

      lastLogTimeRef.current = now;
    } else if (!isCurrentlyMoving && isMovingRef.current) {
      // Stopping movement
      auditLogger.log({
        event_type: 'command',
        event_action: 'command_sent',
        robot_id: connection.connectedRobot?.id,
        robot_name: connection.connectedRobot?.name,
        event_details: {
          command: 'stop',
          source: controlTypeRef.current
        }
      });
    }

    isMovingRef.current = isCurrentlyMoving;
    velocityTopic.publish(scaledVelocity);
  };

  return publishVelocity;
}
