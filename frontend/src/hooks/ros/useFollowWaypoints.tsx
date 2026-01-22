'use client';

import { useState, useCallback, useRef, useEffect } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

// ============ Nav2 Message Types ============

interface PoseStamped {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}

// FollowWaypoints action goal
interface FollowWaypointsGoal {
  poses: PoseStamped[];
}

// FollowWaypoints action feedback
interface FollowWaypointsFeedback {
  current_waypoint: number;
}

// WaypointStatus from nav2_msgs (newer Nav2 versions)
interface WaypointStatus {
  waypoint_index: number;
  waypoint_pose: PoseStamped;
  waypoint_status: number; // 0 = success, others = failure
  error_code: number;
  error_msg: string;
}

// FollowWaypoints action result
// Note: In older Nav2 versions, missed_waypoints was int32[]
// In newer versions (Rolling/Jazzy), it's WaypointStatus[]
interface FollowWaypointsResult {
  // Can be either number[] (old) or WaypointStatus[] (new)
  missed_waypoints: number[] | WaypointStatus[];
  error_code?: number;
  error_msg?: string;
}

// Error codes from Nav2
export enum FollowWaypointsErrorCode {
  NONE = 0,
  UNKNOWN = 600,
  TASK_EXECUTOR_FAILED = 601,
  NO_VALID_WAYPOINTS = 602,
  STOP_ON_MISSED_WAYPOINT = 603,
}

// ============ Hook Input/Output Types ============

export interface Waypoint {
  x: number;
  y: number;
  theta: number;
  id?: string;
}

export interface NavigationConfig {
  /** Whether to abort mission if a single waypoint fails. Default: false */
  stopOnFailure: boolean;
}

export type NavigationStatus =
  | 'idle'
  | 'ready'
  | 'navigating'
  | 'completed'
  | 'cancelled'
  | 'failed';

export interface NavigationProgress {
  currentWaypointIndex: number;
  totalWaypoints: number;
  missedWaypoints: number[];
  startTime: number | null;
  elapsedTime: number;
}

export interface NavigationError {
  code: FollowWaypointsErrorCode;
  message: string;
  failedWaypointIndices: number[];
}

export interface UseFollowWaypointsReturn {
  // Actions
  startNavigation: (waypoints: Waypoint[], config?: Partial<NavigationConfig>) => boolean;
  cancelNavigation: () => boolean;

  // State
  status: NavigationStatus;
  progress: NavigationProgress;
  error: NavigationError | null;

  // Connection info
  isConnected: boolean;
  isActionServerAvailable: boolean;
}

// ============ Constants ============

const DEFAULT_CONFIG: NavigationConfig = {
  stopOnFailure: false,
};

const FOLLOW_WAYPOINTS_ACTION = '/follow_waypoints';
const FOLLOW_WAYPOINTS_ACTION_TYPE = 'nav2_msgs/action/FollowWaypoints';

// ============ Helper Functions ============

/**
 * Convert yaw angle (theta) to quaternion orientation.
 */
function yawToQuaternion(theta: number): { x: number; y: number; z: number; w: number } {
  return {
    x: 0,
    y: 0,
    z: Math.sin(theta / 2),
    w: Math.cos(theta / 2),
  };
}

// ============ Main Hook ============

/**
 * Hook for waypoint navigation using Nav2's FollowWaypoints action server.
 *
 * This implementation uses the ROS2 Action API (roslib 2.x) which:
 * - Takes all waypoints at once
 * - Nav2 handles sequencing server-side
 * - Provides feedback on current waypoint index
 * - Returns list of missed waypoints in the result
 *
 * @example
 * ```tsx
 * const { startNavigation, cancelNavigation, status, progress } = useFollowWaypoints();
 *
 * startNavigation([
 *   { x: 1.0, y: 2.0, theta: 0 },
 *   { x: 3.0, y: 4.0, theta: Math.PI / 2 },
 * ]);
 * ```
 */
export default function useFollowWaypoints(): UseFollowWaypointsReturn {
  const { connection } = useRobotConnection();

  // State
  const [status, setStatus] = useState<NavigationStatus>('idle');
  const [progress, setProgress] = useState<NavigationProgress>({
    currentWaypointIndex: 0,
    totalWaypoints: 0,
    missedWaypoints: [],
    startTime: null,
    elapsedTime: 0,
  });
  const [error, setError] = useState<NavigationError | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [isActionServerAvailable, setIsActionServerAvailable] = useState(false);

  // Refs for managing navigation state
  // Using roslib 2.x Action class for ROS2 action support
  const actionRef = useRef<ROSLIB.Action<FollowWaypointsGoal, FollowWaypointsFeedback, FollowWaypointsResult> | null>(null);
  const currentGoalIdRef = useRef<string | null>(null);
  const timerRef = useRef<NodeJS.Timeout | null>(null);
  const navigationStartTimeRef = useRef<number | null>(null);
  const isCancelledRef = useRef(false);
  const totalWaypointsRef = useRef<number>(0);

  // Initialize action when connection changes
  useEffect(() => {
    if (!connection.ros || !connection.online) {
      setIsConnected(false);
      setIsActionServerAvailable(false);
      setStatus(prev => prev === 'ready' ? 'idle' : prev);
      actionRef.current = null;
      return;
    }

    try {
      // Create ROS2 Action client for FollowWaypoints (roslib 2.x API)
      actionRef.current = new ROSLIB.Action({
        ros: connection.ros,
        name: FOLLOW_WAYPOINTS_ACTION,
        actionType: FOLLOW_WAYPOINTS_ACTION_TYPE,
      });

      console.log('[FollowWaypoints] ROS2 Action client initialized for', FOLLOW_WAYPOINTS_ACTION);
      setIsConnected(true);
      setIsActionServerAvailable(true);
      setStatus(prev => prev === 'idle' ? 'ready' : prev);

    } catch (err) {
      console.error('[FollowWaypoints] Failed to initialize action client:', err);
      setIsConnected(false);
      setIsActionServerAvailable(false);
    }

    return () => {
      // Cancel any active goal on cleanup
      if (currentGoalIdRef.current && actionRef.current) {
        try {
          actionRef.current.cancelGoal(currentGoalIdRef.current);
        } catch (e) {
          // Ignore
        }
        currentGoalIdRef.current = null;
      }
    };
  }, [connection.ros, connection.online]);

  // Elapsed time updater
  useEffect(() => {
    if (status === 'navigating' && navigationStartTimeRef.current) {
      timerRef.current = setInterval(() => {
        setProgress(prev => ({
          ...prev,
          elapsedTime: Math.floor((Date.now() - (navigationStartTimeRef.current || Date.now())) / 1000),
        }));
      }, 1000);
    }

    return () => {
      if (timerRef.current) {
        clearInterval(timerRef.current);
        timerRef.current = null;
      }
    };
  }, [status]);

  // Start navigation through waypoints
  const startNavigation = useCallback((
    waypoints: Waypoint[],
    config: Partial<NavigationConfig> = {}
  ): boolean => {
    if (!isConnected || !actionRef.current) {
      console.error('[FollowWaypoints] Cannot start: not connected or action client not available');
      setError({
        code: FollowWaypointsErrorCode.UNKNOWN,
        message: 'Not connected to robot or action server not available',
        failedWaypointIndices: [],
      });
      return false;
    }

    if (waypoints.length === 0) {
      console.error('[FollowWaypoints] Cannot start: no waypoints provided');
      setError({
        code: FollowWaypointsErrorCode.NO_VALID_WAYPOINTS,
        message: 'No waypoints provided',
        failedWaypointIndices: [],
      });
      return false;
    }

    // Cancel any existing goal
    if (currentGoalIdRef.current && actionRef.current) {
      try {
        actionRef.current.cancelGoal(currentGoalIdRef.current);
      } catch (e) {
        // Ignore
      }
    }

    // Reset state
    isCancelledRef.current = false;
    navigationStartTimeRef.current = Date.now();
    totalWaypointsRef.current = waypoints.length;

    console.log('[FollowWaypoints] Starting navigation');
    console.log('[FollowWaypoints] Waypoints:', waypoints.length);
    console.log('[FollowWaypoints] Config:', { ...DEFAULT_CONFIG, ...config });

    setError(null);
    setProgress({
      currentWaypointIndex: 0,
      totalWaypoints: waypoints.length,
      missedWaypoints: [],
      startTime: Date.now(),
      elapsedTime: 0,
    });
    setStatus('navigating');

    // Convert waypoints to PoseStamped array
    const poses: PoseStamped[] = waypoints.map(wp => ({
      header: {
        stamp: { sec: 0, nanosec: 0 },
        frame_id: 'map',
      },
      pose: {
        position: { x: wp.x, y: wp.y, z: 0.0 },
        orientation: yawToQuaternion(wp.theta),
      },
    }));

    // Create the goal message
    const goalMessage: FollowWaypointsGoal = {
      poses: poses,
    };

    // Result callback - called when action completes successfully
    const resultCallback = (result: FollowWaypointsResult) => {
      if (isCancelledRef.current) {
        console.log('[FollowWaypoints] Goal was cancelled, ignoring result');
        return;
      }

      console.log('[FollowWaypoints] Navigation completed');
      console.log('[FollowWaypoints] Full result:', JSON.stringify(result, null, 2));
      console.log('[FollowWaypoints] Result error_code:', result.error_code);
      console.log('[FollowWaypoints] Result error_msg:', result.error_msg);
      console.log('[FollowWaypoints] Raw missed_waypoints type:', typeof result.missed_waypoints);
      console.log('[FollowWaypoints] Raw missed_waypoints:', result.missed_waypoints);

      // Parse missed_waypoints - handle both old (int32[]) and new (WaypointStatus[]) formats
      // Also handle edge cases where rosbridge might send unexpected formats
      let missedIndices: number[] = [];
      const rawMissedWaypoints = result.missed_waypoints;

      if (rawMissedWaypoints && Array.isArray(rawMissedWaypoints) && rawMissedWaypoints.length > 0) {
        // Check if it's the new WaypointStatus[] format or old number[] format
        const firstItem = rawMissedWaypoints[0];
        console.log('[FollowWaypoints] First missed_waypoint item type:', typeof firstItem);
        console.log('[FollowWaypoints] First missed_waypoint item:', firstItem);

        if (typeof firstItem === 'object' && firstItem !== null && 'waypoint_index' in firstItem) {
          // New format: WaypointStatus[]
          console.log('[FollowWaypoints] Using new WaypointStatus[] format');
          const waypointStatuses = rawMissedWaypoints as WaypointStatus[];
          missedIndices = waypointStatuses.map(ws => ws.waypoint_index);

          // Log detailed error info for each missed waypoint
          waypointStatuses.forEach(ws => {
            console.log(`[FollowWaypoints] Missed waypoint ${ws.waypoint_index}: error_code=${ws.error_code}, error_msg="${ws.error_msg}"`);
          });
        } else if (typeof firstItem === 'number') {
          // Old format: number[]
          console.log('[FollowWaypoints] Using old int32[] format');
          missedIndices = rawMissedWaypoints as number[];
        } else {
          // Unknown format - try to extract what we can
          console.warn('[FollowWaypoints] Unknown missed_waypoints format, attempting to parse');
          missedIndices = rawMissedWaypoints.map((item: unknown, idx: number) => {
            if (typeof item === 'number') return item;
            if (typeof item === 'object' && item !== null && 'waypoint_index' in item) {
              return (item as WaypointStatus).waypoint_index;
            }
            console.warn(`[FollowWaypoints] Could not parse missed_waypoint at index ${idx}:`, item);
            return idx; // Fallback to index
          });
        }
      }

      console.log('[FollowWaypoints] Parsed missed waypoint indices:', missedIndices);

      setProgress(prev => ({
        ...prev,
        currentWaypointIndex: totalWaypointsRef.current,
        missedWaypoints: missedIndices,
      }));

      if (missedIndices.length > 0) {
        const errorMessage = result.error_msg
          ? `${result.error_msg} (${missedIndices.length} missed waypoint(s))`
          : `Completed with ${missedIndices.length} missed waypoint(s)`;
        setError({
          code: result.error_code || FollowWaypointsErrorCode.TASK_EXECUTOR_FAILED,
          message: errorMessage,
          failedWaypointIndices: missedIndices,
        });
      }

      // Mark as completed regardless of missed waypoints
      setStatus('completed');
      currentGoalIdRef.current = null;
    };

    // Feedback callback - called during action execution
    const feedbackCallback = (feedback: FollowWaypointsFeedback) => {
      if (isCancelledRef.current) return;

      console.log(`[FollowWaypoints] Feedback: current_waypoint = ${feedback.current_waypoint}`);

      setProgress(prev => ({
        ...prev,
        currentWaypointIndex: feedback.current_waypoint,
      }));
    };

    // Failed callback - called when action fails or is cancelled
    const failedCallback = (errorMsg: string) => {
      if (isCancelledRef.current) {
        console.log('[FollowWaypoints] Goal was cancelled');
        setStatus('cancelled');
      } else {
        console.error('[FollowWaypoints] Action failed:', errorMsg);
        setStatus('failed');
        setError({
          code: FollowWaypointsErrorCode.TASK_EXECUTOR_FAILED,
          message: errorMsg || 'Navigation failed',
          failedWaypointIndices: [],
        });
      }
      currentGoalIdRef.current = null;
    };

    // Send the goal using roslib 2.x API
    console.log('[FollowWaypoints] Sending goal to', FOLLOW_WAYPOINTS_ACTION);
    const goalId = actionRef.current.sendGoal(
      goalMessage,
      resultCallback,
      feedbackCallback,
      failedCallback
    );

    if (goalId) {
      currentGoalIdRef.current = goalId;
      console.log('[FollowWaypoints] Goal sent with ID:', goalId);
    } else {
      console.error('[FollowWaypoints] Failed to send goal');
      setStatus('failed');
      setError({
        code: FollowWaypointsErrorCode.UNKNOWN,
        message: 'Failed to send goal to action server',
        failedWaypointIndices: [],
      });
      return false;
    }

    return true;
  }, [isConnected]);

  // Cancel navigation
  const cancelNavigation = useCallback((): boolean => {
    console.log('[FollowWaypoints] Cancel requested');

    isCancelledRef.current = true;

    // Clear timer
    if (timerRef.current) {
      clearInterval(timerRef.current);
      timerRef.current = null;
    }

    // Cancel the goal using roslib 2.x API
    if (currentGoalIdRef.current && actionRef.current) {
      try {
        actionRef.current.cancelGoal(currentGoalIdRef.current);
        console.log('[FollowWaypoints] Cancel sent to action server');
      } catch (e) {
        console.error('[FollowWaypoints] Error cancelling goal:', e);
      }
      currentGoalIdRef.current = null;
    }

    setStatus('cancelled');
    console.log('[FollowWaypoints] Navigation cancelled');

    return true;
  }, []);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      isCancelledRef.current = true;

      if (timerRef.current) {
        clearInterval(timerRef.current);
      }

      if (currentGoalIdRef.current && actionRef.current) {
        try {
          actionRef.current.cancelGoal(currentGoalIdRef.current);
        } catch (e) {
          // Ignore
        }
      }
    };
  }, []);

  return {
    startNavigation,
    cancelNavigation,
    status,
    progress,
    error,
    isConnected,
    isActionServerAvailable,
  };
}
