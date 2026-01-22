'use client';

import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import useRobotCamera from '@/hooks/ros/useRobotCamera';
import { useEffect, useRef, useState, forwardRef } from 'react';
import RobotOffline from './robot-offline';
import { Loader, Maximize2, Minimize2 } from 'lucide-react';
import { CameraType, OverlayType } from '@/types/CameraType';
import { useLanguage } from '@/contexts/LanguageContext';
import { getConfig } from '@/utils/config';
import { NavigationTarget } from '@/contexts/NavigationTargetsContext';
import { MapPose } from '@/hooks/ros/useMapPose';
import { CameraConfig, projectWaypoint, calculateMarkerSize, projectPathPoint } from '@/utils/waypointProjection';
import { NavPlan } from '@/hooks/ros/useRosNavPlan';

// Simple throttle function
function throttle<T extends (...args: any[]) => void>(fn: T, delay: number): T {
  let lastCall = 0;
  return ((...args: any[]) => {
    const now = Date.now();
    if (now - lastCall >= delay) {
      lastCall = now;
      fn(...args);
    }
  }) as T;
}

// Color mapping for overlay colors
const overlayColorMap: Record<string, string> = {
  white: 'rgba(255, 255, 255, 0.7)',
  black: 'rgba(0, 0, 0, 0.7)',
  red: 'rgba(255, 0, 0, 0.7)',
  purple: 'rgba(128, 0, 128, 0.7)',
  blue: 'rgba(0, 0, 255, 0.7)',
  green: 'rgba(0, 128, 0, 0.7)',
};

export default function RosCameraImg({
  width,
  height,
  topicName,
  cameraType,
  offlineMsg,
  overlay = 'none',
  waypointArEnabled = false,
  navigationTargets = [],
  robotPose = null,
  cameraConfig,
  pathArEnabled = false,
  navPlan = null,
}: {
  width?: number;
  height?: number;
  topicName?: string;
  cameraType: CameraType;
  offlineMsg?: string;
  overlay?: OverlayType;
  waypointArEnabled?: boolean;
  navigationTargets?: NavigationTarget[];
  robotPose?: MapPose | null;
  cameraConfig?: CameraConfig;
  pathArEnabled?: boolean;
  navPlan?: NavPlan | null;
}) {
  const [content, setContent] = useState<React.ReactNode | undefined>(
    undefined
  );
  const [isFullScreen, setIsFullScreen] = useState(false);
  const containerRef = useRef<HTMLDivElement>(null);
  
  const image = useRobotCamera(false, cameraType, topicName);

  const latestImageRef = useRef(image);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const { connection } = useRobotConnection();
  const canvasSetSize = useRef(false);
  const { t } = useLanguage();

  // Cache of last image data (can be string or binary)
  const lastImageDataRef = useRef<any>(null);
  // Ref for throttled canvas update function
  const throttledUpdateCanvasRef = useRef<() => void>(() => {});
  // Store the current overlay type in a ref for access in the draw function
  const overlayRef = useRef<OverlayType>(overlay);
  // Store the current overlay color
  const [overlayColor, setOverlayColor] = useState('white');

  // Refs for waypoint AR overlay
  const waypointArEnabledRef = useRef(waypointArEnabled);
  const navigationTargetsRef = useRef(navigationTargets);
  const robotPoseRef = useRef(robotPose);
  const cameraConfigRef = useRef(cameraConfig);

  // Refs for path AR overlay
  const pathArEnabledRef = useRef(pathArEnabled);
  const navPlanRef = useRef(navPlan);

  // Update AR refs when props change
  useEffect(() => {
    waypointArEnabledRef.current = waypointArEnabled;
  }, [waypointArEnabled]);

  useEffect(() => {
    navigationTargetsRef.current = navigationTargets;
  }, [navigationTargets]);

  useEffect(() => {
    robotPoseRef.current = robotPose;
  }, [robotPose]);

  useEffect(() => {
    cameraConfigRef.current = cameraConfig;
  }, [cameraConfig]);

  // Update path AR refs when props change
  useEffect(() => {
    pathArEnabledRef.current = pathArEnabled;
  }, [pathArEnabled]);

  useEffect(() => {
    navPlanRef.current = navPlan;
  }, [navPlan]);

  // Load the overlay color from settings
  useEffect(() => {
    const savedColor = getConfig('overlayColor');
    setOverlayColor(savedColor);
  }, []);

  useEffect(() => {
    overlayRef.current = overlay;
  }, [overlay]);

  useEffect(() => {
    latestImageRef.current = image;
  }, [image]);

  // Handle full-screen toggling
  const toggleFullScreen = () => {
    if (!containerRef.current) return;
    
    if (!isFullScreen) {
      // Enter full-screen mode
      if (containerRef.current.requestFullscreen) {
        containerRef.current.requestFullscreen().then(() => {
          setIsFullScreen(true);
        }).catch((err) => {
          console.error('Error attempting to enable full-screen mode:', err);
        });
      } else if ((containerRef.current as any).webkitRequestFullscreen) {
        (containerRef.current as any).webkitRequestFullscreen();
        setIsFullScreen(true);
      } else if ((containerRef.current as any).msRequestFullscreen) {
        (containerRef.current as any).msRequestFullscreen();
        setIsFullScreen(true);
      }
    } else {
      // Exit full-screen mode
      if (document.exitFullscreen) {
        document.exitFullscreen().then(() => {
          setIsFullScreen(false);
        }).catch((err) => {
          console.error('Error attempting to exit full-screen mode:', err);
        });
      } else if ((document as any).webkitExitFullscreen) {
        (document as any).webkitExitFullscreen();
        setIsFullScreen(false);
      } else if ((document as any).msExitFullscreen) {
        (document as any).msExitFullscreen();
        setIsFullScreen(false);
      }
    }
  };

  // Listen for exit full-screen event (e.g. when user presses Escape)
  useEffect(() => {
    const handleFullScreenChange = () => {
      if (document.fullscreenElement === null) {
        setIsFullScreen(false);
        // Reset canvas size when exiting full-screen
        canvasSetSize.current = false;
        throttledUpdateCanvasRef.current();
      }
    };

    document.addEventListener('fullscreenchange', handleFullScreenChange);
    document.addEventListener('webkitfullscreenchange', handleFullScreenChange);
    document.addEventListener('mozfullscreenchange', handleFullScreenChange);
    document.addEventListener('MSFullscreenChange', handleFullScreenChange);

    return () => {
      document.removeEventListener('fullscreenchange', handleFullScreenChange);
      document.removeEventListener('webkitfullscreenchange', handleFullScreenChange);
      document.removeEventListener('mozfullscreenchange', handleFullScreenChange);
      document.removeEventListener('MSFullscreenChange', handleFullScreenChange);
    };
  }, []);

  // Function to check if image data is empty
  const isImageDataEmpty = (data: any): boolean => {
    if (!data) return true;
    
    // If data is a string, check if it's empty after trimming
    if (typeof data === 'string') {
      return data.trim() === '';
    }
    
    // If data is an ArrayBuffer or Uint8Array, check if it has content
    if (data instanceof ArrayBuffer || data instanceof Uint8Array) {
      return data.byteLength === 0;
    }
    
    // If data is an array, check if it's empty
    if (Array.isArray(data)) {
      return data.length === 0;
    }
    
    return false;
  };

  // Draw overlay on the canvas
  const drawOverlay = (ctx: CanvasRenderingContext2D, width: number, height: number) => {
    const currentOverlay = overlayRef.current;
    if (currentOverlay === 'none') return;

    ctx.save();
    // Use the selected color from settings
    ctx.strokeStyle = overlayColorMap[overlayColor] || overlayColorMap.white;
    ctx.lineWidth = 1;

    if (currentOverlay === 'crosshair') {
      // Draw crosshair
      const centerX = width / 2;
      const centerY = height / 2;
      
      // Horizontal line
      ctx.beginPath();
      ctx.moveTo(0, centerY);
      ctx.lineTo(width, centerY);
      ctx.stroke();
      
      // Vertical line
      ctx.beginPath();
      ctx.moveTo(centerX, 0);
      ctx.lineTo(centerX, height);
      ctx.stroke();
      
      // Center circle
      ctx.beginPath();
      ctx.arc(centerX, centerY, 10, 0, 2 * Math.PI);
      ctx.stroke();
    } 
    else if (currentOverlay === 'grid') {
      // Draw grid - 5x5 grid
      const numLines = 5;
      const spacing = width / numLines;
      
      // Vertical lines
      for (let i = 1; i < numLines; i++) {
        ctx.beginPath();
        ctx.moveTo(i * spacing, 0);
        ctx.lineTo(i * spacing, height);
        ctx.stroke();
      }
      
      // Horizontal lines
      const vSpacing = height / numLines;
      for (let i = 1; i < numLines; i++) {
        ctx.beginPath();
        ctx.moveTo(0, i * vSpacing);
        ctx.lineTo(width, i * vSpacing);
        ctx.stroke();
      }
    } 
    else if (currentOverlay === 'corners') {
      // Draw diagonal lines from corners
      
      // Top-left to bottom-right
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(width, height);
      ctx.stroke();
      
      // Top-right to bottom-left
      ctx.beginPath();
      ctx.moveTo(width, 0);
      ctx.lineTo(0, height);
      ctx.stroke();
    }
    
    ctx.restore();
  };

  // Video bounds for proper overlay positioning (accounts for letterboxing)
  const videoBoundsRef = useRef({ offsetX: 0, offsetY: 0, width: 0, height: 0 });

  // Draw path AR overlay on the canvas (3D-looking path)
  // Only renders on front-facing cameras (not back camera)
  // Uses video bounds to properly position the path on the actual video area
  const drawPathOverlay = (ctx: CanvasRenderingContext2D, canvasWidth: number, canvasHeight: number) => {
    if (!pathArEnabledRef.current) return;

    const plan = navPlanRef.current;
    const pose = robotPoseRef.current;
    const config = cameraConfigRef.current;
    const bounds = videoBoundsRef.current;

    // Skip if no data or if this is the back camera
    if (!plan || !plan.poses || plan.poses.length < 2 || !pose || !config) return;
    if (!config.isFrontFacing) return;  // Don't render path on back camera
    if (bounds.width === 0 || bounds.height === 0) return;  // No video bounds yet

    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    // Clip to video bounds to prevent drawing outside the video area
    ctx.beginPath();
    ctx.rect(bounds.offsetX, bounds.offsetY, bounds.width, bounds.height);
    ctx.clip();

    // Sample poses for performance (every 2nd point for smoother path)
    const sampledPoses = plan.poses.filter((_, i) =>
      i % 2 === 0 || i === plan.poses.length - 1
    );

    // Project points using video dimensions (not canvas dimensions)
    // Then transform to canvas coordinates
    const projectedPoints = sampledPoses.map(p => {
      const projected = projectPathPointWithBounds(
        { x: p.pose.position.x, y: p.pose.position.y },
        { x: pose.x, y: pose.y, theta: pose.theta },
        bounds.width,
        bounds.height,
        config
      );
      // Transform to canvas coordinates
      return {
        ...projected,
        screenX: projected.screenX + bounds.offsetX,
        screenY: projected.screenY + bounds.offsetY,
      };
    }).filter(p =>
      p.isInFront &&
      p.distance > 0.5 &&    // Min distance to avoid giant lines
      p.distance < 5 &&      // Max distance - keep path short and accurate
      p.isInView             // Only render points within FOV
    );

    if (projectedPoints.length < 2) {
      ctx.restore();
      return;
    }

    // Draw path segments from far to near (painter's algorithm for proper layering)
    for (let i = projectedPoints.length - 2; i >= 0; i--) {
      const p1 = projectedPoints[i];
      const p2 = projectedPoints[i + 1];

      // Skip segments that are too far apart (discontinuities in screen space)
      const segmentScreenDist = Math.sqrt(
        Math.pow(p2.screenX - p1.screenX, 2) + Math.pow(p2.screenY - p1.screenY, 2)
      );
      if (segmentScreenDist > bounds.width * 0.3) continue;

      // Average width and alpha for segment
      const avgWidth = (p1.width + p2.width) / 2;
      const avgDistance = (p1.distance + p2.distance) / 2;
      // Alpha based on distance: close = opaque, far = slightly transparent
      const alpha = Math.max(0.7, 1.0 - (avgDistance / 5) * 0.3);

      // Scale width based on video size (smaller video = thinner lines)
      const scaledWidth = avgWidth * (bounds.height / 400);

      // Draw outer glow first (under main path)
      ctx.strokeStyle = 'rgba(34, 197, 94, 0.25)';
      ctx.lineWidth = scaledWidth * 2.2;
      ctx.globalAlpha = alpha * 0.4;
      ctx.beginPath();
      ctx.moveTo(p1.screenX, p1.screenY);
      ctx.lineTo(p2.screenX, p2.screenY);
      ctx.stroke();

      // Draw dark edge for definition (under green path)
      ctx.strokeStyle = 'rgba(0, 0, 0, 0.4)';
      ctx.lineWidth = scaledWidth + 2;
      ctx.globalAlpha = alpha * 0.5;
      ctx.beginPath();
      ctx.moveTo(p1.screenX, p1.screenY);
      ctx.lineTo(p2.screenX, p2.screenY);
      ctx.stroke();

      // Draw main green path
      ctx.strokeStyle = '#22c55e';
      ctx.lineWidth = scaledWidth;
      ctx.globalAlpha = alpha;
      ctx.beginPath();
      ctx.moveTo(p1.screenX, p1.screenY);
      ctx.lineTo(p2.screenX, p2.screenY);
      ctx.stroke();

      // Draw inner highlight for 3D effect (lighter green center line)
      ctx.strokeStyle = 'rgba(200, 255, 220, 0.8)';
      ctx.lineWidth = scaledWidth * 0.3;
      ctx.globalAlpha = alpha * 0.9;
      ctx.beginPath();
      ctx.moveTo(p1.screenX, p1.screenY);
      ctx.lineTo(p2.screenX, p2.screenY);
      ctx.stroke();
    }

    ctx.restore();
  };

  /**
   * Project path point with proper perspective for video overlay
   * Positions the path in the lower portion of the video where the ground is visible
   */
  function projectPathPointWithBounds(
    point: { x: number; y: number },
    robotPose: { x: number; y: number; theta: number },
    videoWidth: number,
    videoHeight: number,
    config: CameraConfig
  ) {
    // Calculate vector from robot to point
    const dx = point.x - robotPose.x;
    const dy = point.y - robotPose.y;
    const distance = Math.sqrt(dx * dx + dy * dy);

    // Get bearing angle from robot to point in map frame
    const pointBearing = Math.atan2(dy, dx);

    // Calculate relative bearing based on camera facing direction
    let robotHeading = robotPose.theta;
    if (!config.isFrontFacing) {
      robotHeading = robotHeading + Math.PI;
      // Normalize to [-PI, PI]
      while (robotHeading > Math.PI) robotHeading -= 2 * Math.PI;
      while (robotHeading < -Math.PI) robotHeading += 2 * Math.PI;
    }

    let relativeBearing = pointBearing - robotHeading;
    // Normalize
    while (relativeBearing > Math.PI) relativeBearing -= 2 * Math.PI;
    while (relativeBearing < -Math.PI) relativeBearing += 2 * Math.PI;

    // Determine if point is in front
    const isInFront = Math.abs(relativeBearing) < Math.PI / 2;

    // Determine if point is within camera FOV
    const halfFov = config.horizontalFov / 2;
    const isInView = Math.abs(relativeBearing) <= halfFov;

    // Calculate screen X position (horizontal)
    const centerX = videoWidth / 2;
    let screenX: number;

    if (isInView) {
      // Linear mapping within FOV
      screenX = centerX - (relativeBearing / halfFov) * (videoWidth / 2);
    } else {
      screenX = relativeBearing > 0 ? 0 : videoWidth;
    }

    // Calculate screen Y position
    // The path should be in the LOWER part of the frame (where the ground is)
    //
    // Simple approach: linear mapping with distance
    // - Closest point (0.5m): at 98% of video height (very bottom)
    // - Farthest point (5m): at 50% of video height (middle - horizon area)
    //
    // This keeps the path in the bottom half of the video

    const minDist = 0.5;
    const maxDist = 5;
    const clampedDist = Math.max(minDist, Math.min(maxDist, distance));

    // Normalize distance: 0 = closest, 1 = farthest
    const distNorm = (clampedDist - minDist) / (maxDist - minDist);

    // Screen Y: bottom (98%) for close, higher (50%) for far (horizon area)
    // Path spans bottom 48% of video
    const bottomY = videoHeight * 0.98;
    const topY = videoHeight * 0.50;
    const screenY = bottomY - distNorm * (bottomY - topY);

    // Calculate line width based on distance
    // Close = thick (12px), far = thin (3px)
    const maxWidth = 12;
    const minWidth = 3;
    const width = maxWidth - distNorm * (maxWidth - minWidth);

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

  // Draw waypoint AR overlay on the canvas
  const drawWaypointOverlay = (ctx: CanvasRenderingContext2D, width: number, height: number) => {
    if (!waypointArEnabledRef.current) return;

    const targets = navigationTargetsRef.current;
    const pose = robotPoseRef.current;
    const config = cameraConfigRef.current;

    if (!targets || targets.length === 0 || !pose || !config) return;

    ctx.save();

    for (const target of targets) {
      const projected = projectWaypoint(
        { x: target.x, y: target.y },
        { x: pose.x, y: pose.y, theta: pose.theta },
        width,
        height,
        config
      );

      // Determine marker color based on status
      let color: string;
      if (target.isReached) {
        color = '#22c55e'; // green
      } else if (target.isCurrent) {
        color = '#f59e0b'; // orange
      } else if (target.type === 'mission') {
        color = '#a855f7'; // purple
      } else {
        color = '#3b82f6'; // blue
      }

      if (projected.isInView) {
        drawDiamondMarker(ctx, projected.screenX, projected.screenY, projected.distance, color, target, height);
      } else if (projected.edgeDirection) {
        drawEdgeIndicator(ctx, projected.edgeDirection, projected.distance, color, width, height);
      }
    }

    ctx.restore();
  };

  // Draw a diamond marker for waypoints in view
  const drawDiamondMarker = (
    ctx: CanvasRenderingContext2D,
    screenX: number,
    screenY: number,
    distance: number,
    color: string,
    target: NavigationTarget,
    canvasHeight: number
  ) => {
    const size = calculateMarkerSize(distance);

    ctx.save();
    ctx.translate(screenX, screenY);

    // Draw vertical guide line (dashed)
    ctx.strokeStyle = color;
    ctx.globalAlpha = 0.4;
    ctx.lineWidth = 1;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    ctx.moveTo(0, size + 5);
    ctx.lineTo(0, canvasHeight - screenY);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;

    // Draw diamond shape
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(0, -size);           // Top
    ctx.lineTo(size * 0.6, 0);      // Right
    ctx.lineTo(0, size);            // Bottom
    ctx.lineTo(-size * 0.6, 0);     // Left
    ctx.closePath();
    ctx.fill();

    // Draw white border for contrast
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 2;
    ctx.stroke();

    // Draw distance text below marker
    ctx.fillStyle = 'white';
    ctx.font = 'bold 11px Arial, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';

    // Text shadow for better readability
    ctx.shadowColor = 'rgba(0, 0, 0, 0.8)';
    ctx.shadowBlur = 3;
    ctx.shadowOffsetX = 1;
    ctx.shadowOffsetY = 1;
    ctx.fillText(`${distance.toFixed(1)}m`, 0, size + 8);

    // Draw waypoint index inside diamond for missions
    if (target.type === 'mission' && target.index !== undefined) {
      ctx.shadowBlur = 0;
      ctx.shadowOffsetX = 0;
      ctx.shadowOffsetY = 0;
      ctx.fillStyle = 'white';
      ctx.font = `bold ${Math.max(9, size * 0.6)}px Arial, sans-serif`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText((target.index + 1).toString(), 0, 0);
    }

    ctx.restore();
  };

  // Draw edge indicator for waypoints outside FOV
  const drawEdgeIndicator = (
    ctx: CanvasRenderingContext2D,
    direction: 'left' | 'right',
    distance: number,
    color: string,
    canvasWidth: number,
    canvasHeight: number
  ) => {
    const arrowSize = 14;
    const margin = 20;
    const yPos = canvasHeight * 0.5;
    const xPos = direction === 'left' ? margin : canvasWidth - margin;

    ctx.save();
    ctx.translate(xPos, yPos);

    // Draw arrow pointing in direction
    ctx.fillStyle = color;
    ctx.globalAlpha = 0.85;
    ctx.beginPath();
    if (direction === 'left') {
      ctx.moveTo(-arrowSize, 0);
      ctx.lineTo(arrowSize * 0.4, -arrowSize * 0.7);
      ctx.lineTo(arrowSize * 0.4, arrowSize * 0.7);
    } else {
      ctx.moveTo(arrowSize, 0);
      ctx.lineTo(-arrowSize * 0.4, -arrowSize * 0.7);
      ctx.lineTo(-arrowSize * 0.4, arrowSize * 0.7);
    }
    ctx.closePath();
    ctx.fill();

    // White border
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Distance text
    ctx.globalAlpha = 1;
    ctx.fillStyle = 'white';
    ctx.font = 'bold 10px Arial, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    ctx.shadowColor = 'rgba(0, 0, 0, 0.8)';
    ctx.shadowBlur = 2;
    ctx.fillText(`${distance.toFixed(1)}m`, 0, arrowSize + 4);

    ctx.restore();
  };

  // Create throttled function only once
  useEffect(() => {
    throttledUpdateCanvasRef.current = throttle(async () => {
      const currentImage = latestImageRef.current;
      if (!currentImage.data || isImageDataEmpty(currentImage.data)) {
        return;
      }
      if (!canvasRef.current) return;

      try {
        const canvas = canvasRef.current;
        const bitmap = await createBitmapFromJPEG(currentImage.data);
        
        // Set canvas dimensions to match its display size
        if (!canvasSetSize.current) {
          const parentRect = canvas.parentElement?.getBoundingClientRect();
          if (parentRect) {
            // Get the display dimensions
            const displayWidth = parentRect.width;
            const displayHeight = parentRect.height;
            
            // Set the canvas dimensions to match the display size
            // This ensures the canvas uses the full container space
            canvas.width = displayWidth;
            canvas.height = displayHeight;
            
            canvasSetSize.current = true;
          }
        }
        
        // Calculate scaling to maintain aspect ratio
        const canvasRatio = canvas.width / canvas.height;
        const imageRatio = bitmap.width / bitmap.height;

        let drawWidth, drawHeight, offsetX, offsetY;

        if (imageRatio > canvasRatio) {
          // Image is wider than canvas
          drawWidth = canvas.width;
          drawHeight = canvas.width / imageRatio;
          offsetX = 0;
          offsetY = (canvas.height - drawHeight) / 2;
        } else {
          // Image is taller than canvas
          drawHeight = canvas.height;
          drawWidth = canvas.height * imageRatio;
          offsetX = (canvas.width - drawWidth) / 2;
          offsetY = 0;
        }

        // Store video bounds for overlay positioning
        videoBoundsRef.current = {
          offsetX,
          offsetY,
          width: drawWidth,
          height: drawHeight
        };

        const ctx = canvas.getContext('2d', { willReadFrequently: true });
        if (ctx) {
          // Clear previous frame
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          // Draw image with calculated dimensions to maintain aspect ratio
          ctx.drawImage(bitmap, offsetX, offsetY, drawWidth, drawHeight);

          // Draw overlay if needed
          drawOverlay(ctx, canvas.width, canvas.height);

          // Draw path AR overlay if enabled (draw path behind waypoints)
          drawPathOverlay(ctx, canvas.width, canvas.height);

          // Draw waypoint AR overlay if enabled
          drawWaypointOverlay(ctx, canvas.width, canvas.height);
        }
      } catch (error) {
        console.error('Error processing JPEG image:', error);
      }
    }, 25);
  }, [overlayColor]);

  // Update canvas when entering or exiting full-screen
  useEffect(() => {
    if (isFullScreen) {
      // Reset canvas size when entering full-screen so it adapts to new size
      canvasSetSize.current = false;
    }
    
    if (
      canvasRef.current &&
      image !== undefined &&
      image.format === 'jpeg' &&
      image.data &&
      !isImageDataEmpty(image.data)
    ) {
      throttledUpdateCanvasRef.current();
    }
  }, [isFullScreen]);

  // Update canvas only if the image changed and using throttledUpdateCanvas
  useEffect(() => {
    if (
      canvasRef.current &&
      image !== undefined &&
      image.format === 'jpeg' &&
      image.data &&
      !isImageDataEmpty(image.data)
    ) {
      // Check if the image data is different from the last one
      // For binary data, we can't directly compare, so we'll just update
      const isStringData = typeof image.data === 'string';
      if (isStringData && lastImageDataRef.current === image.data) {
        return;
      }
      lastImageDataRef.current = image.data;
      throttledUpdateCanvasRef.current();
    }
  }, [image, width, height, cameraType]);

  // Force update when overlay changes
  useEffect(() => {
    if (
      canvasRef.current &&
      image !== undefined &&
      image.format === 'jpeg' &&
      image.data &&
      !isImageDataEmpty(image.data)
    ) {
      throttledUpdateCanvasRef.current();
    }
  }, [overlay]);

  // Force update when overlay color changes
  useEffect(() => {
    if (
      canvasRef.current &&
      image !== undefined &&
      image.format === 'jpeg' &&
      image.data &&
      !isImageDataEmpty(image.data)
    ) {
      throttledUpdateCanvasRef.current();
    }
  }, [overlayColor]);

  // Force update when AR overlay state changes
  useEffect(() => {
    if (
      canvasRef.current &&
      image !== undefined &&
      image.format === 'jpeg' &&
      image.data &&
      !isImageDataEmpty(image.data)
    ) {
      throttledUpdateCanvasRef.current();
    }
  }, [waypointArEnabled, navigationTargets, robotPose]);

  // Force update when path AR overlay state changes
  useEffect(() => {
    if (
      canvasRef.current &&
      image !== undefined &&
      image.format === 'jpeg' &&
      image.data &&
      !isImageDataEmpty(image.data)
    ) {
      throttledUpdateCanvasRef.current();
    }
  }, [pathArEnabled, navPlan]);

  useEffect(() => {
    if (image === undefined || image.format !== 'jpeg') {
      setContent(
        <span className="w-full h-full border-2 border-gray-200 dark:border-black rounded-lg flex flex-col items-center justify-center text-center">
          <Loader className="w-5 h-5 animate-spin" />
          {t('robotCams', 'initializingCamera')}
        </span>
      );
    } else {
      setContent(
        <div className="relative w-full h-full">
          <canvas
            className="rounded-lg border-2 border-gray-200 dark:border-black"
            style={{ 
              width: '100%',
              height: '100%',
              objectFit: 'contain',
            }}
            ref={canvasRef}
            id={`camera-canvas-${cameraType}`}
          />
          <button
            className="absolute top-3 left-3 bg-black/50 hover:bg-black/70 text-white rounded-lg p-1.5 transition-colors"
            onClick={toggleFullScreen}
            aria-label={isFullScreen ? t('robotCams', 'exitFullscreen') : t('robotCams', 'enterFullscreen')}
            title={isFullScreen ? t('robotCams', 'exitFullscreen') : t('robotCams', 'enterFullscreen')}
          >
            {isFullScreen ? (
              <Minimize2 className="w-4 h-4" />
            ) : (
              <Maximize2 className="w-4 h-4" />
            )}
          </button>
        </div>
      );
    }
  }, [image, cameraType, t, isFullScreen]);

  return (
    <div 
      className={`w-full h-full relative flex items-center justify-center rounded-lg ${isFullScreen ? 'fullscreen-camera' : ''}`}
      ref={containerRef}
    >
      {connection.online && content}
      {!connection.online && <RobotOffline msg={offlineMsg} useBorder />}
    </div>
  );
}

async function createBitmapFromJPEG(data: any): Promise<ImageBitmap> {
  let blob: Blob;

  // Handle different data types
  if (typeof data === 'string') {
    // Handle string data (base64)
    const dataUrl = data.startsWith('data:image/jpeg;base64,')
      ? data
      : 'data:image/jpeg;base64,' + data;
      
    // Convert dataURL to Blob
    const response = await fetch(dataUrl);
    blob = await response.blob();
  } 
  else if (data instanceof Uint8Array || data instanceof ArrayBuffer) {
    // Handle binary data from CBOR
    blob = new Blob([data], { type: 'image/jpeg' });
  }
  else if (Array.isArray(data)) {
    // Handle array of numbers (sometimes returned by CBOR)
    const uint8Array = new Uint8Array(data);
    blob = new Blob([uint8Array], { type: 'image/jpeg' });
  }
  else {
    throw new Error('Unsupported image data format');
  }

  // Create and return ImageBitmap
  return await createImageBitmap(blob);
}
