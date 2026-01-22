'use client';

import { useMemo } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { getRobotProfile } from '@/config/robot-profiles';
import { ROSTopicFactory, ROSServiceFactory } from '@/utils/ros/topics-and-services-v2';
import { ServiceOptions } from '@/interfaces/ros/ServiceOptions';
import { auditLogger } from '@/utils/audit-logger';

export function useProfileAwareROS() {
  const { connection } = useRobotConnection();
  
  // Get the current robot profile based on connected robot
  const robotProfile = useMemo(() => {
    if (!connection.connectedRobot) return null;
    return getRobotProfile(connection.connectedRobot.type);
  }, [connection.connectedRobot]);

  // Create profile-aware topic factory
  const topicFactory = useMemo(() => {
    if (!connection.ros || !connection.ros.isConnected) return null;
    return new ROSTopicFactory(connection.ros, false, robotProfile);
  }, [connection.ros, connection.ros?.isConnected, robotProfile]);

  // Create profile-aware service factory
  const serviceFactory = useMemo(() => {
    if (!connection.ros || !connection.ros.isConnected) return null;
    return new ROSServiceFactory(connection.ros, false, robotProfile);
  }, [connection.ros, connection.ros?.isConnected, robotProfile]);

  // Profile-aware action dispatcher
  const dispatchAction = (options: ServiceOptions) => {
    if (!connection.ros || !connection.ros.isConnected || !serviceFactory) {
      options.failedCallback?.(
        'Unable to execute action: No ROS connection available.'
      );
      return;
    }
    
    // Log the command being dispatched
    const robotId = connection.connectedRobot?.id;
    const robotName = connection.connectedRobot?.name;
    auditLogger.logCommand(options.typeKey, robotId, robotName, {
      request: options.request,
      robotType: connection.connectedRobot?.type,
      profile: robotProfile?.name,
    });
    
    serviceFactory.callServiceWithCallback(options);
  };

  return {
    robotProfile,
    topicFactory,
    serviceFactory,
    dispatchAction,
    isConnected: connection.ros?.isConnected || false,
  };
}