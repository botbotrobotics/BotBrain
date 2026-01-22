'use client';

import { useMemo, useCallback, useState } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

// Nav2 NavigateThroughPoses action message types
interface PoseStamped {
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

interface NavigateThroughPosesGoal {
  poses: {
    header: {
      stamp: {
        sec: number;
        nanosec: number;
      };
      frame_id: string;
    };
    goals: PoseStamped[];
  };
  behavior_tree: string;
}

interface NavigateThroughPosesFeedback {
  current_waypoint_index: number;
  distance_remaining: number;
  estimated_time_remaining: {
    sec: number;
    nanosec: number;
  };
  navigation_time: {
    sec: number;
    nanosec: number;
  };
  number_of_recoveries: number;
  current_pose: PoseStamped;
}

interface NavigateThroughPosesResult {
  result: boolean;
  error_code: number;
}

export interface Waypoint {
  x: number;
  y: number;
  theta?: number;
}

export default function useNav2NavigateThroughPoses() {
  const { connection } = useRobotConnection();
  const [isNavigating, setIsNavigating] = useState(false);
  const [currentWaypointIndex, setCurrentWaypointIndex] = useState(-1);
  const [navigationFeedback, setNavigationFeedback] = useState<NavigateThroughPosesFeedback | null>(null);
  const [activeGoal, setActiveGoal] = useState<ROSLIB.Goal<NavigateThroughPosesGoal, NavigateThroughPosesFeedback, NavigateThroughPosesResult> | null>(null);

  const actionClient = useMemo(() => {
    if (!connection.ros || !connection.online) return null;

    try {
      return new ROSLIB.ActionClient({
        ros: connection.ros,
        serverName: '/navigate_through_poses',
        actionName: 'nav2_msgs/NavigateThroughPoses'
      });
    } catch (error) {
      console.error('Failed to create NavigateThroughPoses action client:', error);
      return null;
    }
  }, [connection.ros, connection.online]);

  const navigateThroughPoses = useCallback((waypoints: Waypoint[]) => {
    if (!actionClient || waypoints.length === 0) {
      console.warn('Cannot navigate: No action client or empty waypoints');
      return false;
    }

    // Cancel any existing navigation
    if (activeGoal) {
      activeGoal.cancel();
    }

    // Convert waypoints to PoseStamped messages
    const poses: PoseStamped[] = waypoints.map(waypoint => {
      // Convert theta to quaternion if provided, otherwise face forward
      const theta = waypoint.theta || 0;
      const qz = Math.sin(theta / 2);
      const qw = Math.cos(theta / 2);

      return {
        header: {
          stamp: {
            sec: Math.floor(Date.now() / 1000),
            nanosec: (Date.now() % 1000) * 1000000
          },
          frame_id: 'map'
        },
        pose: {
          position: {
            x: waypoint.x,
            y: waypoint.y,
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
    });

    const goalMessage: NavigateThroughPosesGoal = {
      poses: {
        header: {
          stamp: {
            sec: Math.floor(Date.now() / 1000),
            nanosec: (Date.now() % 1000) * 1000000
          },
          frame_id: 'map'
        },
        goals: poses
      },
      behavior_tree: '' // Use default behavior tree
    };

    try {
      const goal = new ROSLIB.Goal({
        actionClient: actionClient,
        goalMessage: goalMessage
      });

      // Set up feedback callback
      goal.on('feedback', (fb: unknown) => {
        const feedback = fb as NavigateThroughPosesFeedback;
        console.log('Navigation feedback:', feedback);
        setNavigationFeedback(feedback);
        setCurrentWaypointIndex(feedback.current_waypoint_index);
      });

      // Set up result callback
      goal.on('result', (res: unknown) => {
        const result = res as NavigateThroughPosesResult;
        console.log('Navigation result:', result);
        setIsNavigating(false);
        setActiveGoal(null);
        setCurrentWaypointIndex(-1);
        
        if (result.result) {
          console.log('Navigation completed successfully');
        } else {
          console.error('Navigation failed with error code:', result.error_code);
        }
      });

      // Set up status callback
      goal.on('status', (status) => {
        console.log('Goal status:', status);
        // Status codes: 1=ACTIVE, 2=PREEMPTED, 3=SUCCEEDED, 4=ABORTED, 5=REJECTED, 6=PREEMPTING, 7=RECALLING, 8=RECALLED, 9=LOST
        if ([2, 4, 5, 8, 9].includes(status.status)) {
          setIsNavigating(false);
          setActiveGoal(null);
          setCurrentWaypointIndex(-1);
        }
      });

      // Send the goal
      goal.send();

      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      setActiveGoal(goal as any);
      setIsNavigating(true);
      setCurrentWaypointIndex(0);
      
      console.log('Sent navigation goal with', waypoints.length, 'waypoints');
      return true;
    } catch (error) {
      console.error('Failed to send navigation goal:', error);
      return false;
    }
  }, [actionClient, activeGoal]);

  const cancelNavigation = useCallback(() => {
    if (!activeGoal) {
      console.warn('No active navigation to cancel');
      return false;
    }

    try {
      activeGoal.cancel();
      setIsNavigating(false);
      setActiveGoal(null);
      setCurrentWaypointIndex(-1);
      setNavigationFeedback(null);
      console.log('Navigation cancelled');
      return true;
    } catch (error) {
      console.error('Failed to cancel navigation:', error);
      return false;
    }
  }, [activeGoal]);

  return {
    navigateThroughPoses,
    cancelNavigation,
    isNavigating,
    currentWaypointIndex,
    navigationFeedback,
    isConnected: !!actionClient
  };
}