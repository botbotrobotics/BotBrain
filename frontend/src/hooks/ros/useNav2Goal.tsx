'use client';

import { useMemo } from 'react';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { auditLogger } from '@/utils/audit-logger';
import * as ROSLIB from 'roslib';

interface PoseStamped   {
  header: {
    stamp: {
      sec: number;
      nanosec: number;
    };
    frame_id: string;
  };
  pose: {
    position: {
      x: number;
      y: number;
      z: number;
    };
    orientation: {
      x: number;
      y: number;
      z: number;
      w: number;
    };
  };
}

interface BoolMsg   {
  data: boolean;
}

export default function useNav2Goal() {
  const { connection } = useRobotConnection();
  
  const goalTopic = useMemo(() => {
    if (!connection.ros || !connection.online) return null;
    
    const topicFactory = new ROSTopicFactory(connection.ros, false);
    
    // Create topic but don't subscribe (we only publish)
    return topicFactory.createAndSubscribeTopic<PoseStamped>(
      'goalPose',
      () => {} // Empty callback since we only publish
    );
  }, [connection.ros, connection.online]);

  const cancelTopic = useMemo(() => {
    if (!connection.ros || !connection.online) return null;
    
    const topicFactory = new ROSTopicFactory(connection.ros, false);
    
    // Create cancel topic
    return topicFactory.createAndSubscribeTopic<BoolMsg>(
      'cancelGoal',
      () => {} // Empty callback since we only publish
    );
  }, [connection.ros, connection.online]);

  const publishGoal = (x: number, y: number, theta: number = 0) => {
    if (!goalTopic) {
      console.warn('ROS connection not established. Cannot publish goal.');
      return false;
    }

    // Convert theta to quaternion
    const qz = Math.sin(theta / 2);
    const qw = Math.cos(theta / 2);

    const goalMessage: PoseStamped = {
      header: {
        stamp: {
          sec: Math.floor(Date.now() / 1000),
          nanosec: (Date.now() % 1000) * 1000000
        },
        frame_id: 'map'
      },
      pose: {
        position: {
          x: x,
          y: y,
          z: 0.0
        },
        orientation: {
          x: 0.0,
          y: 0.0,
          z: qz,
          w: qw
        }
      }
    };

    try {
      goalTopic.publish(goalMessage);
      console.log('Published navigation goal:', goalMessage);

      // Log navigation goal to audit
      auditLogger.logNavigationGoalSet(
        { x, y, theta },
        connection.connectedRobot?.id,
        connection.connectedRobot?.name
      );

      return true;
    } catch (error) {
      console.error('Failed to publish navigation goal:', error);

      // Log navigation failure
      auditLogger.logNavigationFailed(
        connection.connectedRobot?.id,
        connection.connectedRobot?.name,
        error instanceof Error ? error.message : 'Unknown error'
      );

      return false;
    }
  };

  const cancelNavigation = () => {
    if (!cancelTopic) {
      console.warn('ROS connection not established. Cannot cancel navigation.');
      return false;
    }

    const cancelMessage: BoolMsg = {
      data: true
    };

    try {
      cancelTopic.publish(cancelMessage);
      console.log('Published navigation cancel command');

      // Log navigation cancellation
      auditLogger.logNavigationCancelled(
        connection.connectedRobot?.id,
        connection.connectedRobot?.name,
        'User cancelled'
      );

      return true;
    } catch (error) {
      console.error('Failed to cancel navigation:', error);
      return false;
    }
  };

  return {
    publishGoal,
    cancelNavigation,
    isConnected: !!goalTopic && !!cancelTopic
  };
}