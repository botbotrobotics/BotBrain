'use client';

import { useState, useEffect, SetStateAction, Dispatch } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';
import { SportModeState } from '@/interfaces/ros/SportModeState';
import { RobotStatus } from '@/types/RobotStatus';
import { modeStates } from '@/utils/ros/modeStates';
import { CustomRobotStatusResponse } from '@/interfaces/ros/CustomRobotStatusResponse';
import { useRobotCustomModeContext } from '@/contexts/RobotCustomModesContext';
import { auditLogger } from '@/utils/audit-logger';

export default function useRobotStatus(_useDummy: boolean = false): {
  robotStatus: RobotStatus;
  setRobotStatus: Dispatch<SetStateAction<RobotStatus>>;
} {
  const { connection } = useRobotConnection();
  const [robotStatus, setRobotStatus] = useState<RobotStatus>(modeStates[0]);
  const { isPose, setIsPose } = useRobotCustomModeContext();
  const { light, antiCollision, setLight, setAntiCollision } =
    useRobotCustomModeContext();

  useEffect(() => {
    if (!connection.ros || !connection.online) return;

    const topicFactory = new ROSTopicFactory(connection.ros, false);

    const statusTopic = topicFactory.createAndSubscribeTopic<SportModeState>(
      'sportModeState',
      (msg) => {
        setRobotStatus((current) => {
          // Se o estado atual for 'emergency' ou 'obstacleAvoidance', não permite alteração
          if (current === 'emergency' || current === 'obstacleAvoidance') {
            return current;
          }

          const newStatus = modeStates[msg.mode];
          
          // Log mode change if status actually changed
          if (current !== newStatus && connection.connectedRobot) {
            auditLogger.logModeChange(
              newStatus, 
              connection.connectedRobot.id,
              connection.connectedRobot.name
            );
          }

          return newStatus;
        });

        if (msg.mode === 2 && !isPose) {
          // pose mode
          setIsPose(true);
        } else if (isPose) {
          setIsPose(false);
        }
      }
    );

    const customStatusTopic =
      topicFactory.createAndSubscribeTopic<CustomRobotStatusResponse>(
        'robotStatus',
        (msg) => {
          setRobotStatus((current) => {
            if (msg.emergency_stop && current !== 'emergency') {
              // Log emergency stop activation
              if (connection.connectedRobot) {
                auditLogger.logEmergencyStop(
                  connection.connectedRobot.id,
                  connection.connectedRobot.name
                );
              }
              return 'emergency';
            } else if (!msg.emergency_stop && current === 'emergency') {
              // Log emergency stop deactivation  
              if (connection.connectedRobot) {
                auditLogger.logCommand(
                  'emergency_stop_released',
                  connection.connectedRobot.id,
                  connection.connectedRobot.name,
                  { previous_mode: current }
                );
              }
              return 'idle';
            }

            return current;
          });

          if (light !== msg.light_state) {
            setLight(msg.light_state);
          }

          if (antiCollision !== msg.obstacle_avoidance_state) {
            setAntiCollision(msg.obstacle_avoidance_state);
          }
        }
      );

    // Cleanup: desinscreve dos tópicos quando o componente é desmontado ou a conexão muda
    return () => {
      statusTopic.unsubscribe();
      customStatusTopic.unsubscribe();
    };
  }, [connection.ros, connection.online, connection.connectedRobot]);

  return {
    robotStatus,
    setRobotStatus,
  };
}
