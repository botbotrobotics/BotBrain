import { CameraType } from '@/types/CameraType';

export interface ProjectedWaypoint {
  screenX: number;           // X position on canvas (0 = left edge)
  screenY: number;           // Y position on canvas (0 = top)
  distance: number;          // Distance in meters
  relativeBearing: number;   // Angle from center (-PI to PI)
  isInView: boolean;         // Is waypoint within camera FOV?
  isInFront: boolean;        // Is waypoint in front of robot (|bearing| < PI/2)?
  edgeDirection: 'left' | 'right' | null; // For off-screen indicators
}

export interface CameraConfig {
  horizontalFov: number;     // Radians
  isFrontFacing: boolean;    // true for front/thermal, false for back
}

export const DEFAULT_CAMERA_CONFIGS: Record<CameraType, CameraConfig> = {
  camera: { horizontalFov: Math.PI * 70 / 180, isFrontFacing: true },
  back: { horizontalFov: Math.PI * 70 / 180, isFrontFacing: false },
  thermal: { horizontalFov: Math.PI * 60 / 180, isFrontFacing: true },
  rgb: { horizontalFov: Math.PI * 70 / 180, isFrontFacing: true },
  yolo: { horizontalFov: Math.PI * 70 / 180, isFrontFacing: true },
  moondream: { horizontalFov: Math.PI * 70 / 180, isFrontFacing: true },
};

/**
 * Normalize angle to [-PI, PI] range
 */
export function normalizeAngle(angle: number): number {
  while (angle > Math.PI) angle -= 2 * Math.PI;
  while (angle < -Math.PI) angle += 2 * Math.PI;
  return angle;
}

/**
 * Calculate shortest angular difference between two angles
 * Returns value in [-PI, PI]
 */
export function angleDifference(target: number, current: number): number {
  return normalizeAngle(target - current);
}

/**
 * Project a waypoint to screen coordinates using bearing-based calculation
 *
 * Algorithm:
 * 1. Calculate vector from robot to waypoint
 * 2. Get bearing angle from robot to waypoint in map frame
 * 3. Compute relative bearing (waypoint bearing - robot heading)
 * 4. Map relative bearing to horizontal screen position
 * 5. Map distance to vertical screen position (closer = lower)
 */
export function projectWaypoint(
  waypoint: { x: number; y: number },
  robotPose: { x: number; y: number; theta: number },
  canvasWidth: number,
  canvasHeight: number,
  config: CameraConfig
): ProjectedWaypoint {
  // Calculate vector from robot to waypoint
  const dx = waypoint.x - robotPose.x;
  const dy = waypoint.y - robotPose.y;
  const distance = Math.sqrt(dx * dx + dy * dy);

  // Get bearing angle from robot to waypoint in map frame
  const waypointBearing = Math.atan2(dy, dx);

  // Calculate relative bearing based on camera facing direction
  let robotHeading = robotPose.theta;
  if (!config.isFrontFacing) {
    // For back camera, flip the robot heading by 180 degrees
    robotHeading = normalizeAngle(robotHeading + Math.PI);
  }

  const relativeBearing = normalizeAngle(waypointBearing - robotHeading);

  // Determine if waypoint is in front (within +/- 90 degrees of camera direction)
  const isInFront = Math.abs(relativeBearing) < Math.PI / 2;

  // Determine if waypoint is within camera FOV
  const halfFov = config.horizontalFov / 2;
  const isInView = Math.abs(relativeBearing) <= halfFov;

  // Calculate screen X position
  // Map relative bearing from [-halfFov, halfFov] to [0, canvasWidth]
  // NOTE: In ROS convention, positive Y is LEFT, so positive relativeBearing means
  // waypoint is to the LEFT. But on screen, we need to NEGATE this because
  // screen X increases to the RIGHT.
  const centerX = canvasWidth / 2;
  let screenX: number;

  if (isInView) {
    // Linear mapping within FOV (negate bearing for correct screen mapping)
    screenX = centerX - (relativeBearing / halfFov) * (canvasWidth / 2);
  } else {
    // Clamp to edge for off-screen waypoints
    // Positive bearing = left of robot = left edge of screen
    screenX = relativeBearing > 0 ? 20 : canvasWidth - 20;
  }

  // Calculate screen Y position based on distance
  // Closer waypoints appear lower on screen (near bottom 1/3)
  // Farther waypoints appear higher (near top 1/3)
  const minDistance = 0.5;  // meters
  const maxDistance = 20;   // meters
  const clampedDistance = Math.max(minDistance, Math.min(maxDistance, distance));

  // Map distance to Y: close = bottom (0.75 * height), far = top (0.25 * height)
  const normalizedDistance = (clampedDistance - minDistance) / (maxDistance - minDistance);
  const minY = canvasHeight * 0.25;
  const maxY = canvasHeight * 0.75;
  const screenY = maxY - normalizedDistance * (maxY - minY);

  // Determine edge direction for off-screen indicators
  // Positive bearing = waypoint to LEFT = show left edge indicator
  let edgeDirection: 'left' | 'right' | null = null;
  if (!isInView && isInFront) {
    edgeDirection = relativeBearing > 0 ? 'left' : 'right';
  } else if (!isInFront) {
    // Waypoint is behind - show edge indicator on the side it would appear if rotated
    edgeDirection = relativeBearing > 0 ? 'left' : 'right';
  }

  return {
    screenX,
    screenY,
    distance,
    relativeBearing,
    isInView,
    isInFront,
    edgeDirection,
  };
}

/**
 * Calculate marker size based on distance
 * Closer waypoints have larger markers, farther ones are smaller
 */
export function calculateMarkerSize(distance: number): number {
  const minSize = 12;
  const maxSize = 30;
  const minDistance = 0.5;
  const maxDistance = 15;

  const clampedDistance = Math.max(minDistance, Math.min(maxDistance, distance));
  const normalizedDistance = (clampedDistance - minDistance) / (maxDistance - minDistance);

  // Inverse relationship: closer = larger
  return maxSize - normalizedDistance * (maxSize - minSize);
}

// ============================================================================
// PATH PROJECTION UTILITIES
// ============================================================================

export interface ProjectedPathPoint {
  screenX: number;
  screenY: number;
  distance: number;
  relativeBearing: number;
  isInView: boolean;
  isInFront: boolean;
  width: number;  // Line width based on distance (closer = wider)
}

/**
 * Calculate path line width based on distance
 * Closer points have wider lines, farther ones are thinner (3D perspective effect)
 */
export function calculatePathWidth(distance: number): number {
  const minWidth = 3;
  const maxWidth = 14;
  const minDistance = 0.5;
  const maxDistance = 8;  // Match the path rendering max distance

  const clampedDistance = Math.max(minDistance, Math.min(maxDistance, distance));
  const normalizedDistance = (clampedDistance - minDistance) / (maxDistance - minDistance);

  // Inverse relationship: closer = wider, with exponential falloff
  const perspectiveNorm = Math.pow(normalizedDistance, 0.8);
  return maxWidth - perspectiveNorm * (maxWidth - minWidth);
}

/**
 * Project a path point to screen coordinates with width for 3D effect
 * Similar to projectWaypoint but includes line width calculation
 *
 * The path is rendered in the lower portion of the screen to match
 * the ground/road perspective - closer points at bottom, farther at middle.
 */
export function projectPathPoint(
  point: { x: number; y: number },
  robotPose: { x: number; y: number; theta: number },
  canvasWidth: number,
  canvasHeight: number,
  config: CameraConfig
): ProjectedPathPoint {
  // Calculate vector from robot to point
  const dx = point.x - robotPose.x;
  const dy = point.y - robotPose.y;
  const distance = Math.sqrt(dx * dx + dy * dy);

  // Get bearing angle from robot to point in map frame
  const pointBearing = Math.atan2(dy, dx);

  // Calculate relative bearing based on camera facing direction
  let robotHeading = robotPose.theta;
  if (!config.isFrontFacing) {
    // For back camera, flip the robot heading by 180 degrees
    robotHeading = normalizeAngle(robotHeading + Math.PI);
  }

  const relativeBearing = normalizeAngle(pointBearing - robotHeading);

  // Determine if point is in front (within +/- 90 degrees of camera direction)
  const isInFront = Math.abs(relativeBearing) < Math.PI / 2;

  // Determine if point is within camera FOV
  const halfFov = config.horizontalFov / 2;
  const isInView = Math.abs(relativeBearing) <= halfFov;

  // Calculate screen X position
  const centerX = canvasWidth / 2;
  let screenX: number;

  if (isInView) {
    // Linear mapping within FOV (negate bearing for correct screen mapping)
    screenX = centerX - (relativeBearing / halfFov) * (canvasWidth / 2);
  } else {
    // Clamp to edge for off-screen points
    screenX = relativeBearing > 0 ? 0 : canvasWidth;
  }

  // Calculate screen Y position based on distance
  // Path should appear on the ground level - closer points near bottom, farther toward horizon
  // Using a more limited range focused on the lower-middle portion of the screen
  const minDistance = 0.5;
  const maxDistance = 8;  // Reduced from 20m - only show nearby path
  const clampedDistance = Math.max(minDistance, Math.min(maxDistance, distance));

  // Use exponential mapping for more natural perspective (closer points spread out more)
  const normalizedDistance = (clampedDistance - minDistance) / (maxDistance - minDistance);
  const perspectiveDistance = Math.pow(normalizedDistance, 0.7); // Exponential for better depth

  // Position path in lower-center area: close = 90% height, far = 55% height (horizon line)
  const minY = canvasHeight * 0.55;  // Horizon line (farther points)
  const maxY = canvasHeight * 0.92;  // Near bottom (closer points)
  const screenY = maxY - perspectiveDistance * (maxY - minY);

  // Calculate line width based on distance
  const width = calculatePathWidth(distance);

  return {
    screenX,
    screenY,
    distance,
    relativeBearing,
    isInView,
    isInFront,
    width,
  };
}
