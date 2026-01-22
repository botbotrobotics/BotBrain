'use client';

import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';
import { auditLogger } from '@/utils/audit-logger';

export function useRosPlayAudio() {
  const { connection } = useRobotConnection();

  const playAudioOnRobot = (clipId: string, clipName?: string) => {
    if (!connection.ros || !connection.ros.isConnected) {
      console.error('No ROS connection available');
      return;
    }

    try {
      // Log the command
      const robotId = connection.connectedRobot?.id;
      const robotName = connection.connectedRobot?.name;

      // Log audio played
      auditLogger.logAudioPlayed(
        clipName || clipId,
        robotId,
        robotName
      );

      // Create the service client
      const playAudioService = new ROSLIB.Service({
        ros: connection.ros,
        name: '/play_audio',
        serviceType: 'bot_custom_interfaces/srv/PlaySound'
      });

      // Create the service request
      const request = {
        clip_id: clipId
      };

      // Call the service
      playAudioService.callService(
        request,
        (response) => {
          console.log('Audio played successfully on robot:', response);
        },
        (error) => {
          console.error('Failed to play audio on robot:', error);
        }
      );
    } catch (error) {
      console.error('Error calling play audio service:', error);
    }
  };

  return { playAudioOnRobot };
}