'use client';

import { useState, useEffect, useRef } from 'react';
import useMapPose, { MapPose } from './useMapPose';
import { normalizeAngle, angleDifference } from '@/utils/waypointProjection';

interface UseSmoothedPoseOptions {
  alpha?: number;           // Smoothing factor (0-1), default 0.15
  enabled?: boolean;
  topicName?: string;
}

/**
 * Returns smoothed robot pose with yaw jitter filtering
 *
 * Uses exponential moving average (EMA) with angle wraparound handling.
 * Formula: smoothed = alpha * raw + (1 - alpha) * previous
 *
 * The smoothing handles the angle discontinuity at +/- PI by using
 * shortest-path interpolation (always interpolates via the shorter arc).
 *
 * @param options.alpha - Smoothing factor (0-1). Lower = smoother but more lag.
 *                        Default 0.15 provides good balance of smoothness and responsiveness.
 * @param options.enabled - Whether to enable pose subscription
 * @param options.topicName - Custom topic name (passed to useMapPose)
 */
export default function useSmoothedPose(options?: UseSmoothedPoseOptions): MapPose | null {
  const { alpha = 0.15, enabled = true, topicName } = options || {};

  // Get raw pose from useMapPose
  const rawPose = useMapPose({ enabled, topicName });

  // State for smoothed pose
  const [smoothedPose, setSmoothedPose] = useState<MapPose | null>(null);

  // Ref to track previous smoothed theta for EMA calculation
  const prevSmoothedThetaRef = useRef<number | null>(null);

  // Ref to track if we've initialized
  const initializedRef = useRef(false);

  useEffect(() => {
    if (!rawPose) {
      // Reset state when pose becomes unavailable
      setSmoothedPose(null);
      prevSmoothedThetaRef.current = null;
      initializedRef.current = false;
      return;
    }

    // First pose - no smoothing, just use raw values
    if (!initializedRef.current || prevSmoothedThetaRef.current === null) {
      setSmoothedPose(rawPose);
      prevSmoothedThetaRef.current = rawPose.theta;
      initializedRef.current = true;
      return;
    }

    // Apply EMA smoothing to theta with angle wraparound handling
    const prevTheta = prevSmoothedThetaRef.current;
    const rawTheta = rawPose.theta;

    // Calculate the angular difference using shortest path
    const diff = angleDifference(rawTheta, prevTheta);

    // Apply EMA: new_smoothed = prev + alpha * diff
    // This interpolates along the shortest arc
    const smoothedTheta = normalizeAngle(prevTheta + alpha * diff);

    // Update ref for next iteration
    prevSmoothedThetaRef.current = smoothedTheta;

    // Create smoothed pose (only theta is smoothed, position doesn't typically need it)
    setSmoothedPose({
      ...rawPose,
      theta: smoothedTheta,
    });
  }, [rawPose, alpha]);

  return smoothedPose;
}

/**
 * Hook to get both raw and smoothed pose for comparison/debugging
 */
export function useSmoothedPoseDebug(options?: UseSmoothedPoseOptions) {
  const rawPose = useMapPose({
    enabled: options?.enabled,
    topicName: options?.topicName
  });
  const smoothedPose = useSmoothedPose(options);

  return {
    raw: rawPose,
    smoothed: smoothedPose,
    thetaDifference: rawPose && smoothedPose
      ? Math.abs(angleDifference(rawPose.theta, smoothedPose.theta)) * 180 / Math.PI
      : 0,
  };
}
