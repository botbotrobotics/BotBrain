'use client';

import { useMemo } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { auditLogger } from '@/utils/audit-logger';
import * as ROSLIB from 'roslib';

interface TwistMessage {
  linear: {
    x: number;
    y: number;
    z: number;
  };
  angular: {
    x: number;
    y: number;
    z: number;
  };
}

export default function useEmergencyStop() {
  const { connection } = useRobotConnection();

  const cmdVelTopic = useMemo(() => {
    if (!connection.ros || !connection.online) return null;
    
    return new ROSLIB.Topic({
      ros: connection.ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });
  }, [connection.ros, connection.online]);

  const emergencyStop = () => {
    if (!cmdVelTopic) {
      console.warn('ROS connection not established. Cannot send emergency stop.');
      return false;
    }

    // Send zero velocity command to immediately stop the robot
    const stopMessage: TwistMessage = {
      linear: {
        x: 0,
        y: 0,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: 0
      }
    };

    try {
      // Send multiple times to ensure it gets through
      for (let i = 0; i < 3; i++) {
        cmdVelTopic.publish(stopMessage);
      }
      console.log('Emergency stop command sent');

      // Log emergency stop to audit
      auditLogger.logEmergencyStop(
        connection.connectedRobot?.id,
        connection.connectedRobot?.name
      );

      return true;
    } catch (error) {
      console.error('Failed to send emergency stop:', error);
      return false;
    }
  };

  return {
    emergencyStop,
    isConnected: !!cmdVelTopic
  };
}