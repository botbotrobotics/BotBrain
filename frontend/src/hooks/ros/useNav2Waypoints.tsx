'use client';

/**
 * @deprecated This hook is deprecated. Use `useFollowWaypoints` instead.
 *
 * This hook uses client-side position monitoring to detect waypoint arrival,
 * which is unreliable and requires continuous network connectivity.
 *
 * The new `useFollowWaypoints` hook properly uses Nav2's FollowWaypoints
 * action server which handles all waypoint sequencing server-side with:
 * - Proper action feedback (current waypoint index)
 * - Missed waypoint reporting
 * - Recovery behaviors
 * - Path planning between waypoints
 *
 * @example
 * // Instead of:
 * import useNav2Waypoints from '@/hooks/ros/useNav2Waypoints';
 *
 * // Use:
 * import useFollowWaypoints from '@/hooks/ros/useFollowWaypoints';
 */

import { useMemo, useCallback, useEffect, useState, useRef } from 'react';
import { ROSTopicFactory, getRosTopic } from '@/utils/ros/topics-and-services';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import useMapPose from '@/hooks/ros/useMapPose';
import * as ROSLIB from 'roslib';

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

interface NavigateThroughPosesGoal   {
  poses: PoseStamped[];
}

interface NavigateThroughPosesFeedback   {
  current_pose: PoseStamped;
  number_of_poses_remaining: number;
  navigation_time: {
    sec: number;
    nanosec: number;
  };
  estimated_time_remaining: {
    sec: number;
    nanosec: number;
  };
}

interface NavigateThroughPosesResult   {
  result: boolean;
}

interface BoolMsg   {
  data: boolean;
}

export interface Waypoint {
  x: number;
  y: number;
  theta: number;
}

export default function useNav2Waypoints() {
  // Log deprecation warning once per mount
  useEffect(() => {
    console.warn(
      '[DEPRECATED] useNav2Waypoints is deprecated. ' +
      'Please migrate to useFollowWaypoints which properly uses the Nav2 FollowWaypoints action server.'
    );
  }, []);

  const { connection } = useRobotConnection();
  const robotPose = useMapPose();
  const [isNavigating, setIsNavigating] = useState(false);
  const [currentWaypointIndex, setCurrentWaypointIndex] = useState(0);
  const [navigationError, setNavigationError] = useState<string | null>(null);
  const [actionServerConnected, setActionServerConnected] = useState(false);
  const [retryCount, setRetryCount] = useState(0);
  const [useSingleGoalFallback, setUseSingleGoalFallback] = useState(false);
  const [remainingWaypoints, setRemainingWaypoints] = useState<Waypoint[]>([]);
  const [goalReachedThreshold] = useState({ position: 0.5, orientation: 0.5 }); // meters and radians
  const lastLogTimeRef = useRef<number>(0);
  const lastMonitorLogRef = useRef<number>(0);
  const goalTopicRef = useRef<ROSLIB.Topic<unknown> | null>(null);
  const maxRetries = 3;
  
  // NavigateThroughPoses action client
  const actionClient = useMemo(() => {
    if (!connection.ros || !connection.online) return null;
    
    // Create action client for NavigateThroughPoses
    // Try without leading slash as some ROS2 setups don't use it
    const client = new ROSLIB.ActionClient({
      ros: connection.ros,
      serverName: 'navigate_through_poses',
      actionName: 'nav2_msgs/action/NavigateThroughPoses'
    });
    
    // Check if action server is available by querying the action server
    // This is a workaround since ROSLIB doesn't have a direct connection check
    try {
      // Set a flag to indicate the client was created successfully
      setActionServerConnected(true);
    } catch (error) {
      console.error('Failed to create Nav2 action client:', error);
      setActionServerConnected(false);
    }
    
    return client;
  }, [connection.ros, connection.online]);

  // Check if action server is available when client changes
  useEffect(() => {
    if (actionClient && connection.ros && connection.online) {
      // Test if server is available by checking ROS connection
      // In a real implementation, you might want to check available action servers
      const checkServerAvailability = () => {
        // This is a simple check - a more robust solution would query available action servers
        const isAvailable = !!(connection.ros && connection.online);
        setActionServerConnected(isAvailable);
      };
      
      checkServerAvailability();
      
      // Re-check periodically
      const interval = setInterval(checkServerAvailability, 5000);
      
      return () => clearInterval(interval);
    } else {
      setActionServerConnected(false);
    }
  }, [actionClient, connection.ros, connection.online]);

  // Cancel navigation topic
  const cancelTopic = useMemo(() => {
    if (!connection.ros || !connection.online) return null;
    
    const topicFactory = new ROSTopicFactory(connection.ros, false);
    
    // Create cancel topic for Nav2
    return topicFactory.createAndSubscribeTopic<BoolMsg>(
      'cancelGoal',
      () => {} // Empty callback since we only publish
    );
  }, [connection.ros, connection.online]);

  // Navigate through waypoints using action server
  const navigateThroughWaypoints = useCallback((waypoints: Waypoint[]) => {
    if (!actionClient || waypoints.length === 0) {
      console.warn('Cannot navigate: no action client or empty waypoints');
      setNavigationError('Navigation system not connected');
      return false;
    }

    // Convert waypoints to PoseStamped array
    const poses: PoseStamped[] = waypoints.map(wp => {
      // Convert theta to quaternion
      const qz = Math.sin(wp.theta / 2);
      const qw = Math.cos(wp.theta / 2);

      return {
        header: {
          stamp: {
            sec: 0,  // Use 0 for current time in ROS2
            nanosec: 0
          },
          frame_id: 'map'
        },
        pose: {
          position: {
            x: wp.x,
            y: wp.y,
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

    // Log the poses being sent
    console.log('Creating NavigateThroughPoses goal with poses:', poses);
    console.log('Action client status:', {
      connected: !!actionClient,
      serverName: 'navigate_through_poses',
      actionName: 'nav2_msgs/action/NavigateThroughPoses'
    });
    
    // Create goal message
    const goal = new ROSLIB.Goal({
      actionClient: actionClient,
      goalMessage: {
        poses: poses
      }
    });

    // Set up callbacks
    goal.on('status', (status: any) => {
      console.log('Goal status update:', status);
      // Status meanings: 1=ACTIVE, 2=PREEMPTED, 3=SUCCEEDED, 4=ABORTED, 5=REJECTED, 6=PREEMPTING, 7=RECALLING, 8=RECALLED, 9=LOST
      if (status.status === 5) {
        console.error('Goal was REJECTED by Nav2');
        setNavigationError('Navigation goal rejected by Nav2');
      } else if (status.status === 4) {
        console.error('Goal was ABORTED by Nav2');
        setNavigationError('Navigation goal aborted by Nav2');
      }
    });
    
    goal.on('feedback', (fb: unknown) => {
      const feedback = fb as NavigateThroughPosesFeedback;
      const remainingPoses = feedback.number_of_poses_remaining;
      const currentIndex = waypoints.length - remainingPoses - 1;
      setCurrentWaypointIndex(Math.max(0, currentIndex));

      console.log('Navigation feedback:', {
        currentWaypointIndex: currentIndex,
        remainingWaypoints: remainingPoses,
        navigationTime: feedback.navigation_time,
        estimatedTimeRemaining: feedback.estimated_time_remaining
      });
    });

    goal.on('result', (res: unknown) => {
      const result = res as NavigateThroughPosesResult;
      setIsNavigating(false);
      if (result.result) {
        console.log('Navigation completed successfully');
        setNavigationError(null);
        setRetryCount(0);  // Reset retry count on success
      } else {
        console.error('Navigation failed');
        setNavigationError('Navigation failed to complete');
        
        // Retry logic
        if (retryCount < maxRetries) {
          console.log(`Retrying navigation (attempt ${retryCount + 1} of ${maxRetries})...`);
          setTimeout(() => {
            setRetryCount(prev => prev + 1);
            // Re-call this function with the same waypoints
            navigateThroughWaypoints(waypoints);
          }, 2000);  // Wait 2 seconds before retry
        } else {
          setNavigationError(`Navigation failed after ${maxRetries} attempts`);
          setRetryCount(0);
        }
      }
    });

    goal.on('timeout', () => {
      setIsNavigating(false);
      setNavigationError('Navigation timeout');
      console.error('Navigation goal timeout');
      
      // Retry logic for timeout
      if (retryCount < maxRetries) {
        console.log(`Retrying after timeout (attempt ${retryCount + 1} of ${maxRetries})...`);
        setTimeout(() => {
          setRetryCount(prev => prev + 1);
          navigateThroughWaypoints(waypoints);
        }, 2000);
      } else {
        setNavigationError(`Navigation timed out after ${maxRetries} attempts`);
        setRetryCount(0);
      }
    });

    // Send the goal
    try {
      console.log('Sending goal to Nav2...');
      goal.send();
      setIsNavigating(true);
      setCurrentWaypointIndex(0);
      setNavigationError(null);
      console.log('Sent navigation goal with', waypoints.length, 'waypoints');
      console.log('Goal sent successfully');
      
      // Add a timeout to check if we get any response
      let receivedFeedback = false;
      
      // Set a flag when we receive feedback
      goal.on('feedback', () => { receivedFeedback = true; });
      goal.on('status', () => { receivedFeedback = true; });
      
      setTimeout(() => {
        if (!receivedFeedback) {
          console.warn('No navigation feedback received after 5 seconds');
          console.warn('This might indicate Nav2 is not running or the action server is not available');
          console.warn('Try running: ros2 action list | grep navigate_through_poses');
        }
      }, 5000);
      
      return true;
    } catch (error) {
      console.error('Failed to send navigation goal:', error);
      setNavigationError('Failed to start navigation');
      return false;
    }
  }, [actionClient, retryCount, maxRetries]);

  // Cancel navigation
  const cancelNavigation = useCallback(() => {
    if (!connection.ros || !connection.online) {
      console.warn('Cannot cancel: no ROS connection');
      return false;
    }

    try {
      // Cancel via action client if available
      if (actionClient && actionClient.cancel) {
        try {
          // Cancel all goals on the action server
          actionClient.cancel();
          console.log('Sent cancel request to action server');
        } catch (error) {
          console.error('Error cancelling via action client:', error);
        }
      }
      
      // Always publish to cancel topic for redundancy and single goal fallback
      if (cancelTopic) {
        const cancelMessage: BoolMsg = {
          data: true
        };
        cancelTopic.publish(cancelMessage);
        console.log('Published cancel message to topic');
      }
      
      // Also publish empty goal to stop robot (common Nav2 pattern)
      if (goalTopicRef.current) {
        const stopMessage: PoseStamped = {
          header: {
            stamp: {
              sec: 0,
              nanosec: 0
            },
            frame_id: 'map'
          },
          pose: {
            position: {
              x: 0,
              y: 0,
              z: 0
            },
            orientation: {
              x: 0,
              y: 0,
              z: 0,
              w: 1
            }
          }
        };
        
        // Advertise the topic if needed
        goalTopicRef.current.advertise();
        
        goalTopicRef.current.publish(stopMessage);
        console.log('Published stop message to goal_pose topic');
      }
      
      // Reset navigation state
      setIsNavigating(false);
      setCurrentWaypointIndex(0);
      setRetryCount(0);
      setRemainingWaypoints([]);
      setNavigationError(null);
      console.log('Navigation cancelled and state reset');
      
      return true;
    } catch (error) {
      console.error('Failed to cancel navigation:', error);
      return false;
    }
  }, [connection.ros, connection.online, cancelTopic, actionClient]);

  // Publish single goal (fallback for single waypoint)
  const publishSingleGoal = useCallback((x: number, y: number, theta: number = 0) => {
    if (!connection.ros || !connection.online) {
      console.warn('ROS connection not established');
      return false;
    }

    // Use the standard goal_pose topic for single waypoints - reuse if exists
    let goalTopic = goalTopicRef.current;
    if (!goalTopic) {
      const topicFactory = new ROSTopicFactory(connection.ros, false);
      goalTopic = topicFactory.createAndSubscribeTopic<PoseStamped>(
        'goalPose',
        () => {}
      );
      goalTopicRef.current = goalTopic;
      console.log('Created new goal_pose topic');
    } else {
      console.log('Reusing existing goal_pose topic');
    }

    // Convert theta to quaternion
    const qz = Math.sin(theta / 2);
    const qw = Math.cos(theta / 2);

    const goalMessage: PoseStamped = {
      header: {
        stamp: {
          sec: 0,  // Use 0 for current time in ROS2
          nanosec: 0
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
      console.log('Publishing to topic:', getRosTopic('goalPose', false));
      console.log('Goal message:', JSON.stringify(goalMessage, null, 2));
      
      // CRITICAL: Ensure the topic is advertised before publishing
      goalTopic.advertise();
      console.log('Advertised goal_pose topic');
      
      goalTopic.publish(goalMessage);
      console.log('Published single navigation goal successfully');
      
      // Also log the topic info
      console.log('Topic details:', {
        name: goalTopic.name,
        messageType: goalTopic.messageType
      });
      
      return true;
    } catch (error) {
      console.error('Failed to publish navigation goal:', error);
      return false;
    }
  }, [connection.ros, connection.online]);

  // Navigate through waypoints with single goal fallback
  const navigateThroughWaypointsWithFallback = useCallback((waypoints: Waypoint[]) => {
    if (waypoints.length === 0) {
      console.warn('Cannot navigate: empty waypoints');
      setNavigationError('No waypoints provided');
      return false;
    }
    
    // Use sequential single goal navigation
    console.log('Starting sequential waypoint navigation with', waypoints.length, 'waypoints');
    console.log('Waypoints:', waypoints.map((wp, i) => `${i+1}: (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})`).join(', '));
    setRemainingWaypoints(waypoints);
    setCurrentWaypointIndex(0);
    setIsNavigating(true);
    setNavigationError(null);
    
    // Send first waypoint
    const firstWaypoint = waypoints[0];
    console.log('Sending waypoint 1 of', waypoints.length, ':', firstWaypoint);
    const success = publishSingleGoal(firstWaypoint.x, firstWaypoint.y, firstWaypoint.theta);
    
    if (!success) {
      console.error('Failed to publish first goal');
      setNavigationError('Failed to publish navigation goal');
      setIsNavigating(false);
      return false;
    }
    
    return true;
    
    // For multiple waypoints, use action server
    if (!actionClient) {
      console.warn('Cannot navigate: no action client');
      setNavigationError('Navigation action server not connected');
      return false;
    }
    
    return navigateThroughWaypoints(waypoints);
  }, [actionClient, publishSingleGoal, navigateThroughWaypoints]);

  // Monitor robot position and check if we reached current waypoint
  useEffect(() => {
    if (!isNavigating || remainingWaypoints.length === 0 || !robotPose) {
      if (!isNavigating && remainingWaypoints.length > 0) {
        console.log('Navigation monitoring stopped but waypoints remain:', remainingWaypoints.length);
      }
      return;
    }
    
    const currentWaypoint = remainingWaypoints[currentWaypointIndex];
    if (!currentWaypoint) {
      console.error('No current waypoint at index', currentWaypointIndex, 'total waypoints:', remainingWaypoints.length);
      return;
    }
    
    // Debug: Log which waypoint we're monitoring
    if (Date.now() - lastMonitorLogRef.current > 5000) {
      console.log(`Monitoring waypoint ${currentWaypointIndex + 1}/${remainingWaypoints.length} for arrival`);
      lastMonitorLogRef.current = Date.now();
    }
    
    // Calculate distance to current waypoint
    const dx = robotPose.x - currentWaypoint.x;
    const dy = robotPose.y - currentWaypoint.y;
    const distance = Math.sqrt(dx * dx + dy * dy);
    
    // Calculate orientation difference
    const dtheta = Math.abs(robotPose.theta - currentWaypoint.theta);
    const orientationDiff = Math.min(dtheta, 2 * Math.PI - dtheta); // Handle wraparound
    
    // Log progress every few seconds
    const now = Date.now();
    if (now - lastLogTimeRef.current > 2000) {
      console.log(`Navigation progress - Waypoint ${currentWaypointIndex + 1}/${remainingWaypoints.length}:`, {
        distance: distance.toFixed(2) + 'm',
        orientationDiff: (orientationDiff * 180 / Math.PI).toFixed(1) + '°',
        threshold: `${goalReachedThreshold.position}m, ${(goalReachedThreshold.orientation * 180 / Math.PI).toFixed(1)}°`,
        robotPose: { x: robotPose.x.toFixed(2), y: robotPose.y.toFixed(2), theta: (robotPose.theta * 180 / Math.PI).toFixed(1) + '°' },
        targetPose: { x: currentWaypoint.x.toFixed(2), y: currentWaypoint.y.toFixed(2), theta: (currentWaypoint.theta * 180 / Math.PI).toFixed(1) + '°' }
      });
      lastLogTimeRef.current = now;
    }
    
    // Check if we reached the waypoint
    if (distance < goalReachedThreshold.position && orientationDiff < goalReachedThreshold.orientation) {
      console.log(`✓ Reached waypoint ${currentWaypointIndex + 1} of ${remainingWaypoints.length}`);
      
      // Check if there are more waypoints
      if (currentWaypointIndex < remainingWaypoints.length - 1) {
        // Send next waypoint
        const nextIndex = currentWaypointIndex + 1;
        const nextWaypoint = remainingWaypoints[nextIndex];
        console.log(`Sending waypoint ${nextIndex + 1} of ${remainingWaypoints.length}:`, nextWaypoint);
        
        const success = publishSingleGoal(nextWaypoint.x, nextWaypoint.y, nextWaypoint.theta);
        if (success) {
          console.log(`Successfully sent waypoint ${nextIndex + 1}, updating index from ${currentWaypointIndex} to ${nextIndex}`);
          setCurrentWaypointIndex(nextIndex);
        } else {
          console.error('Failed to send next waypoint');
          setNavigationError('Failed to send next waypoint');
          setIsNavigating(false);
        }
      } else {
        // We've reached the last waypoint!
        console.log('Reached the last waypoint! Navigation complete.');
        setIsNavigating(false);
        // Update the index to reflect we've reached the last waypoint
        // This ensures the UI knows we've completed all waypoints
        setCurrentWaypointIndex(remainingWaypoints.length - 1);
        // Clear waypoints after a small delay
        setTimeout(() => {
          setRemainingWaypoints([]);
        }, 100);
      }
    }
  }, [robotPose, isNavigating, remainingWaypoints, currentWaypointIndex, goalReachedThreshold, publishSingleGoal]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (goalTopicRef.current) {
        goalTopicRef.current.unsubscribe();
        goalTopicRef.current = null;
      }
    };
  }, []);

  return {
    navigateThroughWaypoints: navigateThroughWaypointsWithFallback,  // Use the fallback version
    cancelNavigation,
    publishSingleGoal,
    isNavigating,
    currentWaypointIndex,
    navigationError,
    isConnected: !!actionClient && !!cancelTopic && actionServerConnected
  };
}