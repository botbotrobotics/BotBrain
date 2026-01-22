'use client';

import React, { useEffect, useRef, useState, useCallback } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { MapPin, Map as MapIcon, ZoomIn, ZoomOut, RotateCcw, Navigation, Square, Trash2, Play, Route, X, RefreshCw, Layers } from 'lucide-react';
import { useLanguage } from '@/contexts/LanguageContext';
import { useNotifications } from '@/contexts/NotificationsContext';
import useOccupancyGrid from '@/hooks/ros/useOccupancyGrid';
import useMapPose from '@/hooks/ros/useMapPose';
import useFollowWaypoints from '@/hooks/ros/useFollowWaypoints';
import useRosNavPlan from '@/hooks/ros/useRosNavPlan';

interface MissionMapViewProps {
  className?: string;
  onWaypointsChange?: (waypoints: Waypoint[]) => void;
  activeMissionId?: string | null;
  onNavigationComplete?: () => void;
  initialWaypoints?: Waypoint[];
  showNavPlan?: boolean;
}

interface ViewState {
  scale: number;
  translateX: number;
  translateY: number;
  isDragging: boolean;
  dragStartX: number;
  dragStartY: number;
}

interface Waypoint {
  id: string;
  x: number;
  y: number;
  theta: number;
  order: number;
}

export default function MissionMapView({ 
  className = '', 
  onWaypointsChange,
  activeMissionId,
  onNavigationComplete,
  initialWaypoints = [],
  showNavPlan = false
}: MissionMapViewProps) {
  const { connection } = useRobotConnection();
  const { t } = useLanguage();
  const { dispatch: notificationDispatch } = useNotifications();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const animationFrameRef = useRef<number | undefined>(undefined);
  const mapTopic = '/map';
  const { occupancyGrid, isLoading, error, retry } = useOccupancyGrid({ topicName: mapTopic });
  
  // Get robot position in map frame
  const robotPose = useMapPose();
  
  // Get navigation plan
  const { navPlan } = useRosNavPlan('/plan');

  // Local costmap for obstacle visualization
  const [showLocalCostmap, setShowLocalCostmap] = useState(false);
  const { occupancyGrid: localCostmap } = useOccupancyGrid({
    topicName: '/local_costmap/costmap',
    enabled: showLocalCostmap,
    throttleMs: 0
  });
  const localCostmapRef = useRef(localCostmap);
  const costmapImageCacheRef = useRef<ImageData | null>(null);
  const costmapDataHashRef = useRef<string>('');

  // Nav2 FollowWaypoints action - proper waypoint following
  const {
    startNavigation: startFollowWaypoints,
    cancelNavigation: cancelFollowWaypoints,
    status: navigationStatus,
    progress: navigationProgress,
    error: navigationError,
    isConnected: nav2Connected,
    isActionServerAvailable,
  } = useFollowWaypoints();

  // Derived states for UI logic
  // With the new FollowWaypoints action, Nav2 handles all sequencing server-side
  // so we only need to check for 'navigating' status
  const isNav2Navigating = navigationStatus === 'navigating';
  const nav2CurrentIndex = navigationProgress.currentWaypointIndex;
  
  // Waypoint management - Initialize with initialWaypoints
  const [waypoints, setWaypoints] = useState<Waypoint[]>(initialWaypoints);
  const [waypointReachedStatus, setWaypointReachedStatus] = useState<Record<string, boolean>>({});
  const [isEditMode, setIsEditMode] = useState(false); // Start in view mode by default
  const [isDraggingWaypoint, setIsDraggingWaypoint] = useState<string | null>(null);
  const [tempWaypoint, setTempWaypoint] = useState<Waypoint | null>(null);
  const [selectedWaypointId, setSelectedWaypointId] = useState<string | null>(null);
  const [isSettingDirection, setIsSettingDirection] = useState(false);
  const [hasCompletedNavigation, setHasCompletedNavigation] = useState(false);

  // Ref to track if navigation has been started for the current mission
  // This prevents re-triggering navigation when other state changes occur
  const hasStartedNavigationRef = useRef(false);

  const [viewState, setViewState] = useState<ViewState>({
    scale: 1,
    translateX: 0,
    translateY: 0,
    isDragging: false,
    dragStartX: 0,
    dragStartY: 0
  });

  // Store robot pose in a ref to access in render loop without causing re-creation
  const robotPoseRef = useRef(robotPose);
  useEffect(() => {
    robotPoseRef.current = robotPose;
  }, [robotPose]);

  // Keep localCostmap ref updated
  useEffect(() => {
    localCostmapRef.current = localCostmap;
  }, [localCostmap]);

  // Update waypoints when activeMissionId changes (mission starts)
  useEffect(() => {
    if (activeMissionId && initialWaypoints.length > 0) {
      console.log('Mission activated, loading waypoints from database:', initialWaypoints);
      setWaypoints(initialWaypoints);
      // Reset the user modified flag when loading from database
      hasUserModifiedWaypoints.current = false;
    }
  }, [activeMissionId]); // Only depend on activeMissionId to avoid loops

  // Navigation functions
  const startNavigation = useCallback(async () => {
    if (waypoints.length === 0) {
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'Navigation Error',
          message: 'No waypoints defined. Please add waypoints first.',
          type: 'error'
        }
      });
      return;
    }

    if (!isActionServerAvailable) {
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'Navigation Error',
          message: 'Nav2 FollowWaypoints action server not available. Ensure Nav2 is running on the robot.',
          type: 'error'
        }
      });
      return;
    }

    // Reset all waypoints reached status and completion flag
    setWaypointReachedStatus({});
    setHasCompletedNavigation(false);

    // Convert waypoints to Nav2 format (without id and order)
    const nav2Waypoints = waypoints.map(wp => ({
      x: wp.x,
      y: wp.y,
      theta: wp.theta
    }));

    // Start navigation using FollowWaypoints action
    console.log('Starting FollowWaypoints navigation with waypoints:', nav2Waypoints);
    const success = startFollowWaypoints(nav2Waypoints, {
      stopOnFailure: false, // Continue to next waypoint if one fails
    });

    if (!success) {
      console.error('Failed to start navigation:', navigationError);
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'Navigation Failed',
          message: navigationError?.message || 'Failed to start navigation. Check robot connection and Nav2 status.',
          type: 'error'
        }
      });
    } else {
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'Navigation Started',
          message: `Navigating through ${waypoints.length} waypoints`,
          type: 'success'
        }
      });
    }
  }, [waypoints, isActionServerAvailable, startFollowWaypoints, navigationError, notificationDispatch]);

  const stopNavigation = useCallback(() => {
    try {
      const success = cancelFollowWaypoints();
      if (success) {
        console.log('Navigation cancelled successfully');
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            title: 'Navigation Cancelled',
            message: 'Mission navigation has been stopped',
            type: 'info'
          }
        });
      } else {
        console.error('Failed to cancel navigation');
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            title: 'Cancel Failed',
            message: 'Failed to cancel navigation. Please check robot connection.',
            type: 'error'
          }
        });
      }
    } catch (error) {
      console.error('Error cancelling navigation:', error);
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'Cancel Error',
          message: 'An error occurred while cancelling navigation',
          type: 'error'
        }
      });
    }
  }, [cancelFollowWaypoints, notificationDispatch]);

  // Notify parent of waypoint changes when they are modified by user actions
  // Use a ref to store previous waypoints to detect real changes
  const prevWaypointsRef = useRef<Waypoint[]>(initialWaypoints);
  const hasUserModifiedWaypoints = useRef(false);
  
  useEffect(() => {
    // Check if waypoints have actually changed (not just reference)
    const waypointsChanged = JSON.stringify(prevWaypointsRef.current) !== JSON.stringify(waypoints);
    
    if (waypointsChanged && hasUserModifiedWaypoints.current && onWaypointsChange) {
      console.log('Waypoints changed, notifying parent:', waypoints);
      onWaypointsChange(waypoints);
      prevWaypointsRef.current = waypoints;
    }
  }, [waypoints, onWaypointsChange]);

  // Handle mission activation/deactivation
  // Uses hasStartedNavigationRef to prevent re-triggering navigation when status changes
  useEffect(() => {
    console.log('Mission activation effect:', {
      activeMissionId: !!activeMissionId,
      waypointsLength: waypoints.length,
      hasStartedNavigation: hasStartedNavigationRef.current,
      hasCompletedNavigation,
      navigationStatus
    });

    if (activeMissionId && waypoints.length > 0 && !hasStartedNavigationRef.current && !hasCompletedNavigation) {
      console.log('Starting navigation from mission activation effect');
      hasStartedNavigationRef.current = true;
      startNavigation();
    } else if (!activeMissionId) {
      // Reset flags when mission is deactivated
      hasStartedNavigationRef.current = false;
      setHasCompletedNavigation(false);
      if (isNav2Navigating) {
        stopNavigation();
      }
    }
  }, [activeMissionId, waypoints.length, hasCompletedNavigation, navigationStatus, isNav2Navigating, startNavigation, stopNavigation]);

  // Track previous waypoint index to detect progress
  const prevWaypointIndexRef = useRef<number>(0);

  // Update waypoint reached status based on navigation progress feedback
  useEffect(() => {
    if (isNav2Navigating && nav2CurrentIndex >= 0) {
      // Update the reached status based on current waypoint index
      // Waypoints before current index are considered reached
      const newStatus: Record<string, boolean> = {};
      waypoints.forEach((wp, index) => {
        // Mark as reached if we're past this waypoint and it wasn't missed
        const isMissed = navigationProgress.missedWaypoints.includes(index);
        newStatus[wp.id] = index < nav2CurrentIndex && !isMissed;
      });
      setWaypointReachedStatus(newStatus);

      // Show progress notification when we advance to a new waypoint
      if (nav2CurrentIndex > prevWaypointIndexRef.current && nav2CurrentIndex > 0) {
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            title: 'Navigation Progress',
            message: `Navigating to waypoint ${nav2CurrentIndex + 1} of ${waypoints.length}`,
            type: 'info'
          }
        });
      }
      prevWaypointIndexRef.current = nav2CurrentIndex;
    }
  }, [nav2CurrentIndex, isNav2Navigating, waypoints, navigationProgress.missedWaypoints, notificationDispatch]);

  // Track previous navigation status to avoid duplicate handling
  const prevNavigationStatusRef = useRef<string>(navigationStatus);

  // Handle navigation status changes (completed, failed, cancelled)
  useEffect(() => {
    // Only handle status changes, not repeated same-status triggers
    if (navigationStatus === prevNavigationStatusRef.current) {
      return;
    }

    console.log('Navigation status changed:', prevNavigationStatusRef.current, '->', navigationStatus);
    prevNavigationStatusRef.current = navigationStatus;

    if (navigationStatus === 'completed') {
      console.log('Navigation completed');
      setHasCompletedNavigation(true);
      // Reset started flag so a new mission can be started
      hasStartedNavigationRef.current = false;

      // Mark all waypoints as reached (except missed ones)
      const newStatus: Record<string, boolean> = {};
      waypoints.forEach((wp, index) => {
        const isMissed = navigationProgress.missedWaypoints.includes(index);
        newStatus[wp.id] = !isMissed;
      });
      setWaypointReachedStatus(newStatus);

      // Show appropriate completion message
      if (navigationProgress.missedWaypoints.length > 0) {
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            title: 'Navigation Complete',
            message: `Completed with ${navigationProgress.missedWaypoints.length} missed waypoint(s)`,
            type: 'warning'
          }
        });
      } else {
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            title: 'Navigation Complete',
            message: 'Successfully reached all waypoints!',
            type: 'success'
          }
        });
      }

      if (onNavigationComplete) {
        onNavigationComplete();
      }
    } else if (navigationStatus === 'failed' && navigationError) {
      console.error('Navigation failed:', navigationError);
      // Reset started flag so navigation can be retried
      hasStartedNavigationRef.current = false;
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'Navigation Failed',
          message: navigationError.message,
          type: 'error'
        }
      });
    } else if (navigationStatus === 'cancelled') {
      console.log('Navigation cancelled');
      // Reset started flag and waypoint index
      hasStartedNavigationRef.current = false;
      prevWaypointIndexRef.current = 0;
    }
  }, [navigationStatus, navigationError, waypoints, navigationProgress.missedWaypoints, onNavigationComplete, notificationDispatch]);

  const renderMap = useCallback(() => {
    if (!canvasRef.current || !occupancyGrid || !containerRef.current) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const container = containerRef.current;
    const containerWidth = container.clientWidth;
    const containerHeight = container.clientHeight;

    // Set canvas size to match container
    canvas.width = containerWidth;
    canvas.height = containerHeight;

    // Clear canvas
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Save context state
    ctx.save();

    // Apply transformations
    ctx.translate(containerWidth / 2, containerHeight / 2);
    ctx.translate(viewState.translateX, viewState.translateY);
    ctx.scale(viewState.scale, viewState.scale);

    // Calculate map dimensions in pixels
    const mapWidth = occupancyGrid.info.width;
    const mapHeight = occupancyGrid.info.height;
    const resolution = occupancyGrid.info.resolution;

    // Draw map centered
    const mapOffsetX = -mapWidth / 2;
    const mapOffsetY = -mapHeight / 2;

    // Create image data for the map
    const imageData = ctx.createImageData(mapWidth, mapHeight);
    const data = imageData.data;

    // Convert occupancy grid to image
    for (let y = 0; y < mapHeight; y++) {
      for (let x = 0; x < mapWidth; x++) {
        const srcIdx = y * mapWidth + x;
        // Flip Y axis for correct orientation
        const dstIdx = ((mapHeight - 1 - y) * mapWidth + x) * 4;
        
        const value = occupancyGrid.data[srcIdx];
        let r, g, b;
        
        if (value === -1) {
          // Unknown - dark gray
          r = g = b = 100;
        } else if (value > 50) {
          // Occupied - black
          r = g = b = 0;
        } else {
          // Free - white
          r = g = b = 255;
        }
        
        data[dstIdx] = r;
        data[dstIdx + 1] = g;
        data[dstIdx + 2] = b;
        data[dstIdx + 3] = 255;
      }
    }

    // Create a temporary canvas for the map image
    const tempCanvas = document.createElement('canvas');
    tempCanvas.width = mapWidth;
    tempCanvas.height = mapHeight;
    const tempCtx = tempCanvas.getContext('2d');
    if (tempCtx) {
      tempCtx.putImageData(imageData, 0, 0);

      // Draw the map image
      ctx.imageSmoothingEnabled = false; // Crisp pixels when zoomed
      ctx.drawImage(tempCanvas, mapOffsetX, mapOffsetY);
    }

    // Draw local costmap overlay if enabled
    if (showLocalCostmap && localCostmapRef.current && occupancyGrid.info.origin) {
      const costmap = localCostmapRef.current;
      const costmapWidth = costmap.info.width;
      const costmapHeight = costmap.info.height;
      const costmapResolution = costmap.info.resolution;

      // Create a simple hash to detect data changes
      const dataHash = `${costmapWidth}-${costmapHeight}-${costmap.data[0]}-${costmap.data[costmap.data.length - 1]}`;

      // Only regenerate image if data changed
      if (dataHash !== costmapDataHashRef.current || !costmapImageCacheRef.current) {
        costmapDataHashRef.current = dataHash;

        // Create costmap image data with color gradient
        const costmapImageData = new ImageData(costmapWidth, costmapHeight);
        const costmapData = costmapImageData.data;

        for (let y = 0; y < costmapHeight; y++) {
          for (let x = 0; x < costmapWidth; x++) {
            const srcIdx = y * costmapWidth + x;
            const dstIdx = ((costmapHeight - 1 - y) * costmapWidth + x) * 4; // Y-flip

            const cost = costmap.data[srcIdx];
            let r = 0, g = 0, b = 0, a = 0;

            if (cost === 0) {
              // Free space - fully transparent
              a = 0;
            } else if (cost === -1) {
              // Unknown - semi-transparent gray
              r = g = b = 128;
              a = 60;
            } else if (cost >= 100) {
              // Definitely occupied - solid dark red
              r = 185; g = 28; b = 28;
              a = 220;
            } else {
              // Gradient: green (low cost) -> yellow -> orange -> red (high cost)
              const normalized = cost / 99;

              if (normalized < 0.33) {
                const t = normalized / 0.33;
                r = Math.round(34 + t * (250 - 34));
                g = Math.round(197 - t * (197 - 204));
                b = Math.round(94 - t * 94);
              } else if (normalized < 0.66) {
                const t = (normalized - 0.33) / 0.33;
                r = 250;
                g = Math.round(204 - t * (204 - 115));
                b = Math.round(t * 22);
              } else {
                const t = (normalized - 0.66) / 0.34;
                r = Math.round(249 - t * (249 - 220));
                g = Math.round(115 - t * (115 - 38));
                b = Math.round(22 + t * (38 - 22));
              }

              // Alpha increases with cost for better visibility
              a = Math.round(80 + normalized * 140);
            }

            costmapData[dstIdx] = r;
            costmapData[dstIdx + 1] = g;
            costmapData[dstIdx + 2] = b;
            costmapData[dstIdx + 3] = a;
          }
        }

        costmapImageCacheRef.current = costmapImageData;
      }

      // Draw the costmap overlay
      if (costmapImageCacheRef.current) {
        const costmapCanvas = document.createElement('canvas');
        costmapCanvas.width = costmapWidth;
        costmapCanvas.height = costmapHeight;
        const costmapCtx = costmapCanvas.getContext('2d');

        if (costmapCtx) {
          costmapCtx.putImageData(costmapImageCacheRef.current, 0, 0);

          // Scale factor if costmap resolution differs from base map
          const scaleFactor = costmapResolution / resolution;

          // The local costmap is in odom frame, centered around the robot
          const currentPose = robotPoseRef.current;

          let drawX: number, drawY: number;

          if (currentPose) {
            // Calculate the costmap center in odom frame
            const costmapCenterOdomX = costmap.info.origin.position.x + (costmapWidth * costmapResolution) / 2;
            const costmapCenterOdomY = costmap.info.origin.position.y + (costmapHeight * costmapResolution) / 2;

            const offsetX = costmap.info.origin.position.x - costmapCenterOdomX;
            const offsetY = costmap.info.origin.position.y - costmapCenterOdomY;

            // Position costmap origin in map frame (robot position + offset)
            const costmapMapOriginX = currentPose.x + offsetX;
            const costmapMapOriginY = currentPose.y + offsetY;

            // Convert to canvas coordinates
            const costmapPixelX = (costmapMapOriginX - occupancyGrid.info.origin.position.x) / resolution;
            const costmapPixelY = (costmapMapOriginY - occupancyGrid.info.origin.position.y) / resolution;

            drawX = mapOffsetX + costmapPixelX;
            drawY = mapOffsetY + (mapHeight - costmapPixelY - costmapHeight * scaleFactor);
          } else {
            // Fallback: direct origin comparison
            const costmapOriginX = (costmap.info.origin.position.x - occupancyGrid.info.origin.position.x) / resolution;
            const costmapOriginY = (costmap.info.origin.position.y - occupancyGrid.info.origin.position.y) / resolution;
            drawX = mapOffsetX + costmapOriginX;
            drawY = mapOffsetY + (mapHeight - costmapOriginY - costmapHeight * scaleFactor);
          }

          ctx.drawImage(
            costmapCanvas,
            drawX,
            drawY,
            costmapWidth * scaleFactor,
            costmapHeight * scaleFactor
          );
        }
      }
    }

    // Draw navigation plan if enabled and available
    if (showNavPlan && navPlan && navPlan.poses.length > 1 && occupancyGrid.info.origin) {
      ctx.strokeStyle = '#22c55e'; // Green color for nav plan
      ctx.lineWidth = 2 / viewState.scale;
      ctx.globalAlpha = 0.8;
      
      ctx.beginPath();
      for (let i = 0; i < navPlan.poses.length; i++) {
        const pose = navPlan.poses[i].pose.position;
        
        // Convert to canvas coordinates
        const x = mapOffsetX + (pose.x - occupancyGrid.info.origin.position.x) / resolution;
        const y = mapOffsetY + (mapHeight - (pose.y - occupancyGrid.info.origin.position.y) / resolution);
        
        if (i === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      }
      ctx.stroke();
      
      // Draw small circles at each pose for visibility
      ctx.fillStyle = '#22c55e';
      for (const poseStamped of navPlan.poses) {
        const pose = poseStamped.pose.position;
        const x = mapOffsetX + (pose.x - occupancyGrid.info.origin.position.x) / resolution;
        const y = mapOffsetY + (mapHeight - (pose.y - occupancyGrid.info.origin.position.y) / resolution);
        
        ctx.beginPath();
        ctx.arc(x, y, 2 / viewState.scale, 0, Math.PI * 2);
        ctx.fill();
      }
      
      ctx.globalAlpha = 1;
    }

    // Draw waypoint paths
    if (waypoints.length > 1) {
      ctx.strokeStyle = isNav2Navigating ? '#f59e0b' : '#3b82f6';
      ctx.lineWidth = 3 / viewState.scale;
      ctx.setLineDash([5 / viewState.scale, 5 / viewState.scale]);
      ctx.globalAlpha = 0.6;
      
      ctx.beginPath();
      for (let i = 0; i < waypoints.length - 1; i++) {
        const wp1 = waypoints[i];
        const wp2 = waypoints[i + 1];
        
        // Convert to canvas coordinates
        const x1 = mapOffsetX + (wp1.x - occupancyGrid.info.origin.position.x) / resolution;
        const y1 = mapOffsetY + (mapHeight - (wp1.y - occupancyGrid.info.origin.position.y) / resolution);
        const x2 = mapOffsetX + (wp2.x - occupancyGrid.info.origin.position.x) / resolution;
        const y2 = mapOffsetY + (mapHeight - (wp2.y - occupancyGrid.info.origin.position.y) / resolution);
        
        if (i === 0) {
          ctx.moveTo(x1, y1);
        }
        ctx.lineTo(x2, y2);
      }
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.globalAlpha = 1;
    }

    // Draw waypoints
    waypoints.forEach((waypoint, index) => {
      if (occupancyGrid.info.origin) {
        // Convert waypoint world position to canvas coordinates
        const wpMapX = (waypoint.x - occupancyGrid.info.origin.position.x) / resolution;
        const wpMapY = (waypoint.y - occupancyGrid.info.origin.position.y) / resolution;
        const wpCanvasX = mapOffsetX + wpMapX;
        const wpCanvasY = mapOffsetY + (mapHeight - wpMapY);

        ctx.save();
        ctx.translate(wpCanvasX, wpCanvasY);

        // Waypoint marker size
        const wpSize = 12 / viewState.scale;
        const isActive = isNav2Navigating && index === nav2CurrentIndex;
        const isReached = waypointReachedStatus[waypoint.id] || false;
        const isSelected = waypoint.id === selectedWaypointId;
        
        // Draw waypoint circle
        ctx.fillStyle = isReached ? '#10b981' : (isActive ? '#f59e0b' : '#3b82f6');
        ctx.strokeStyle = isSelected ? '#8b5cf6' : 'white';
        ctx.lineWidth = isSelected ? 3 / viewState.scale : 2 / viewState.scale;
        
        // Pulsing effect for active waypoint
        if (isActive) {
          const pulse = Math.sin(Date.now() * 0.006) * 0.2 + 0.8;
          ctx.globalAlpha = pulse;
        }
        
        ctx.beginPath();
        ctx.arc(0, 0, wpSize, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.globalAlpha = 1;
        
        // Draw waypoint number
        ctx.fillStyle = 'white';
        ctx.font = `bold ${10 / viewState.scale}px sans-serif`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText((index + 1).toString(), 0, 0);
        
        // Draw direction indicator
        ctx.rotate(-waypoint.theta);
        ctx.strokeStyle = ctx.fillStyle = isReached ? '#10b981' : (isActive ? '#f59e0b' : '#3b82f6');
        ctx.lineWidth = 2 / viewState.scale;
        ctx.beginPath();
        ctx.moveTo(wpSize, 0);
        ctx.lineTo(wpSize * 2, 0);
        ctx.stroke();
        
        // Arrow head
        ctx.beginPath();
        ctx.moveTo(wpSize * 2, 0);
        ctx.lineTo(wpSize * 1.6, -wpSize * 0.3);
        ctx.lineTo(wpSize * 1.6, wpSize * 0.3);
        ctx.closePath();
        ctx.fill();

        ctx.restore();
      }
    });

    // Draw temporary waypoint while adding
    if (tempWaypoint && occupancyGrid.info.origin) {
      const twMapX = (tempWaypoint.x - occupancyGrid.info.origin.position.x) / resolution;
      const twMapY = (tempWaypoint.y - occupancyGrid.info.origin.position.y) / resolution;
      const twCanvasX = mapOffsetX + twMapX;
      const twCanvasY = mapOffsetY + (mapHeight - twMapY);

      ctx.save();
      ctx.translate(twCanvasX, twCanvasY);
      
      const twSize = 12 / viewState.scale;
      ctx.fillStyle = 'rgba(59, 130, 246, 0.5)';
      ctx.strokeStyle = '#3b82f6';
      ctx.lineWidth = 2 / viewState.scale;
      ctx.setLineDash([2 / viewState.scale, 2 / viewState.scale]);
      
      ctx.beginPath();
      ctx.arc(0, 0, twSize, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      
      // Direction indicator
      ctx.rotate(-tempWaypoint.theta);
      ctx.beginPath();
      ctx.moveTo(twSize, 0);
      ctx.lineTo(twSize * 2.5, 0);
      ctx.stroke();
      
      ctx.restore();
    }

    // Draw robot pose if available
    const currentRobotPose = robotPoseRef.current;
    if (currentRobotPose !== null && occupancyGrid.info.origin) {
      // Convert robot world position to map coordinates
      const mapX = (currentRobotPose.x - occupancyGrid.info.origin.position.x) / resolution;
      const mapY = (currentRobotPose.y - occupancyGrid.info.origin.position.y) / resolution;
      
      // Convert to canvas coordinates (with Y flip)
      const canvasX = mapOffsetX + mapX;
      const canvasY = mapOffsetY + (mapHeight - mapY);

      // Draw robot marker
      ctx.save();
      ctx.translate(canvasX, canvasY);
      ctx.rotate(-currentRobotPose.theta); // Negative for correct orientation

      // Robot size scaled appropriately
      const robotSize = 10 / viewState.scale;
      
      // Draw shadow for better visibility
      ctx.fillStyle = 'rgba(0, 0, 0, 0.3)';
      ctx.beginPath();
      ctx.arc(2 / viewState.scale, 2 / viewState.scale, robotSize, 0, Math.PI * 2);
      ctx.fill();

      // Draw robot body
      ctx.fillStyle = '#8b5cf6'; // Purple color (violet-500)
      ctx.strokeStyle = 'white';
      ctx.lineWidth = 2 / viewState.scale;
      ctx.beginPath();
      ctx.arc(0, 0, robotSize, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Draw direction arrow
      ctx.fillStyle = 'white';
      ctx.strokeStyle = '#8b5cf6';
      ctx.lineWidth = 1.5 / viewState.scale;
      ctx.beginPath();
      const arrowLength = robotSize * 2;
      const arrowWidth = robotSize * 0.8;
      ctx.moveTo(robotSize + arrowLength, 0);
      ctx.lineTo(robotSize + arrowWidth, -arrowWidth);
      ctx.lineTo(robotSize + arrowWidth, -arrowWidth/2);
      ctx.lineTo(robotSize, -arrowWidth/2);
      ctx.lineTo(robotSize, arrowWidth/2);
      ctx.lineTo(robotSize + arrowWidth, arrowWidth/2);
      ctx.lineTo(robotSize + arrowWidth, arrowWidth);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      ctx.restore();
    }

    // Restore context state
    ctx.restore();
  }, [occupancyGrid, viewState, waypoints, tempWaypoint, selectedWaypointId, isNav2Navigating, nav2CurrentIndex, waypointReachedStatus, showNavPlan, navPlan, showLocalCostmap]);

  // Initial scale calculation when map loads
  useEffect(() => {
    if (!occupancyGrid || !containerRef.current) return;

    const container = containerRef.current;
    const mapWidth = occupancyGrid.info.width;
    const mapHeight = occupancyGrid.info.height;
    
    const scaleX = (container.clientWidth * 0.8) / mapWidth;
    const scaleY = (container.clientHeight * 0.8) / mapHeight;
    const initialScale = Math.min(scaleX, scaleY, 2);

    setViewState(prev => ({
      ...prev,
      scale: initialScale,
      translateX: 0,
      translateY: 0
    }));
  }, [occupancyGrid]);

  // Render loop
  useEffect(() => {
    const animate = () => {
      renderMap();
      animationFrameRef.current = requestAnimationFrame(animate);
    };
    
    animate();

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [renderMap]);

  // Handle keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        // Cancel current operation
        setIsSettingDirection(false);
        setTempWaypoint(null);
        setIsDraggingWaypoint(null);
        setSelectedWaypointId(null);
      } else if (e.key === 'e' || e.key === 'E') {
        // Toggle edit mode
        setIsEditMode(prev => !prev);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, []);

  // Convert screen coordinates to world coordinates
  const screenToWorld = (screenX: number, screenY: number) => {
    if (!canvasRef.current || !occupancyGrid || !containerRef.current) return null;
    
    const rect = canvasRef.current.getBoundingClientRect();
    const canvasX = screenX - rect.left;
    const canvasY = screenY - rect.top;
    
    const container = containerRef.current;
    const containerWidth = container.clientWidth;
    const containerHeight = container.clientHeight;
    
    // Reverse the transformations
    const mapX = (canvasX - containerWidth / 2 - viewState.translateX) / viewState.scale + occupancyGrid.info.width / 2;
    const mapY = (canvasY - containerHeight / 2 - viewState.translateY) / viewState.scale + occupancyGrid.info.height / 2;
    
    // Convert map coordinates to world coordinates
    const worldX = mapX * occupancyGrid.info.resolution + occupancyGrid.info.origin.position.x;
    const worldY = (occupancyGrid.info.height - mapY) * occupancyGrid.info.resolution + occupancyGrid.info.origin.position.y;
    
    return { x: worldX, y: worldY };
  };

  // Check if click is on a waypoint
  const getWaypointAtPosition = (screenX: number, screenY: number): Waypoint | null => {
    const worldPos = screenToWorld(screenX, screenY);
    if (!worldPos) return null;

    const clickRadius = 20 / viewState.scale; // Click tolerance in world units

    for (const waypoint of waypoints) {
      const dx = waypoint.x - worldPos.x;
      const dy = waypoint.y - worldPos.y;
      const distance = Math.sqrt(dx * dx + dy * dy);
      
      if (distance < clickRadius * occupancyGrid!.info.resolution) {
        return waypoint;
      }
    }
    
    return null;
  };


  // Update waypoint reached status based on Nav2 feedback

  // Handle navigation completion

  // Mouse event handlers
  const handleMouseDown = (e: React.MouseEvent) => {
    e.preventDefault();
    
    // Check if clicking on existing waypoint first
    const clickedWaypoint = getWaypointAtPosition(e.clientX, e.clientY);
    
    if (clickedWaypoint) {
      // Always allow interaction with waypoints
      setSelectedWaypointId(clickedWaypoint.id);
      if (isEditMode) {
        setIsDraggingWaypoint(clickedWaypoint.id);
      }
    } else if (isEditMode && occupancyGrid) {
      // In edit mode, clicking empty space adds a waypoint
      const worldPos = screenToWorld(e.clientX, e.clientY);
      if (worldPos) {
        const newWaypoint: Waypoint = {
          id: Date.now().toString(),
          x: worldPos.x,
          y: worldPos.y,
          theta: 0,
          order: waypoints.length
        };
        setTempWaypoint(newWaypoint);
        setIsSettingDirection(true);
      }
    } else {
      // Not in edit mode - pan the map
      setSelectedWaypointId(null);
      setViewState(prev => ({
        ...prev,
        isDragging: true,
        dragStartX: e.clientX - prev.translateX,
        dragStartY: e.clientY - prev.translateY
      }));
    }
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    e.preventDefault();
    
    if (isSettingDirection && tempWaypoint) {
      // Update orientation based on mouse position
      const worldPos = screenToWorld(e.clientX, e.clientY);
      if (worldPos) {
        const dx = worldPos.x - tempWaypoint.x;
        const dy = worldPos.y - tempWaypoint.y;
        const distance = Math.sqrt(dx * dx + dy * dy);
        
        // Only set direction if mouse moved far enough (avoid jittery directions)
        if (distance > 0.1) {
          const theta = Math.atan2(dy, dx);
          setTempWaypoint(prev => prev ? { ...prev, theta } : null);
        }
      }
    } else if (isDraggingWaypoint && selectedWaypointId) {
      // Move existing waypoint
      const worldPos = screenToWorld(e.clientX, e.clientY);
      if (worldPos) {
        hasUserModifiedWaypoints.current = true;
        setWaypoints(waypoints.map(wp => 
          wp.id === selectedWaypointId 
            ? { ...wp, x: worldPos.x, y: worldPos.y }
            : wp
        ));
      }
    } else if (viewState.isDragging) {
      // Normal pan mode
      setViewState(prev => ({
        ...prev,
        translateX: e.clientX - prev.dragStartX,
        translateY: e.clientY - prev.dragStartY
      }));
    }
  };

  const handleMouseUp = () => {
    if (isSettingDirection && tempWaypoint) {
      // Finalize new waypoint
      hasUserModifiedWaypoints.current = true;
      setWaypoints([...waypoints, tempWaypoint]);
      setSelectedWaypointId(tempWaypoint.id);
      setTempWaypoint(null);
      setIsSettingDirection(false);
    }
    
    setIsDraggingWaypoint(null);
    setViewState(prev => ({
      ...prev,
      isDragging: false
    }));
  };

  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    const delta = e.deltaY > 0 ? 0.9 : 1.1;
    setViewState(prev => ({
      ...prev,
      scale: Math.max(0.1, Math.min(10, prev.scale * delta))
    }));
  };

  // Control button handlers
  const handleZoomIn = () => {
    setViewState(prev => ({
      ...prev,
      scale: Math.min(prev.scale * 1.2, 10)
    }));
  };

  const handleZoomOut = () => {
    setViewState(prev => ({
      ...prev,
      scale: Math.max(prev.scale / 1.2, 0.1)
    }));
  };

  const handleReset = () => {
    if (!occupancyGrid || !containerRef.current) return;

    const container = containerRef.current;
    const mapWidth = occupancyGrid.info.width;
    const mapHeight = occupancyGrid.info.height;
    
    const scaleX = (container.clientWidth * 0.8) / mapWidth;
    const scaleY = (container.clientHeight * 0.8) / mapHeight;
    const resetScale = Math.min(scaleX, scaleY, 2);

    setViewState({
      scale: resetScale,
      translateX: 0,
      translateY: 0,
      isDragging: false,
      dragStartX: 0,
      dragStartY: 0
    });
  };

  const handleDeleteSelectedWaypoint = () => {
    if (selectedWaypointId) {
      hasUserModifiedWaypoints.current = true;
      setWaypoints(waypoints.filter(wp => wp.id !== selectedWaypointId));
      setSelectedWaypointId(null);
    }
  };

  const handleClearAllWaypoints = () => {
    hasUserModifiedWaypoints.current = true;
    setWaypoints([]);
    setSelectedWaypointId(null);
    setTempWaypoint(null);
    setIsSettingDirection(false);
  };

  if (!connection.online) {
    return (
      <div className={`w-full h-full flex items-center justify-center bg-gray-100 dark:bg-botbot-dark rounded-lg ${className}`}>
        <div className="text-center">
          <MapIcon className="w-12 h-12 mx-auto mb-2 text-gray-400" />
          <p className="text-sm text-gray-500">{t('robotOffline', 'robotDisconnected')}</p>
        </div>
      </div>
    );
  }

  return (
    <div className={`w-full h-full bg-gray-100 dark:bg-botbot-dark rounded-lg overflow-hidden relative ${className}`}>

      {/* Edit controls */}
      <div className="absolute top-3 left-3 z-10">
        <div className="bg-white dark:bg-gray-800 rounded-lg shadow-lg p-2 flex items-center gap-2">
          <button
            onClick={() => setIsEditMode(!isEditMode)}
            className={`px-3 py-1.5 rounded transition-colors flex items-center gap-2 text-sm font-medium ${
              isEditMode 
                ? 'bg-blue-500 hover:bg-blue-600 text-white' 
                : 'bg-gray-200 dark:bg-gray-700 text-gray-700 dark:text-white hover:bg-gray-300 dark:hover:bg-gray-600'
            }`}
            title="Toggle edit mode (E)"
          >
            {isEditMode ? (
              <>
                <X className="w-4 h-4" />
                Exit Edit
              </>
            ) : (
              <>
                <Navigation className="w-4 h-4" />
                Edit
              </>
            )}
          </button>
          
          {isEditMode && waypoints.length > 0 && (
            <button
              onClick={handleClearAllWaypoints}
              className="px-3 py-1.5 bg-gray-500 hover:bg-gray-600 text-white text-sm rounded transition-colors"
              title="Clear all waypoints"
            >
              Clear
            </button>
          )}
        </div>
      </div>

      {/* Zoom controls */}
      <div className="absolute top-3 right-3 z-10 flex flex-col gap-2">
        <button
          onClick={handleZoomIn}
          className="bg-white dark:bg-gray-800 text-gray-700 dark:text-white w-8 h-8 rounded-lg border border-gray-300 dark:border-gray-700 shadow-sm hover:bg-gray-50 dark:hover:bg-gray-700 transition-colors flex items-center justify-center"
          title="Zoom in"
        >
          <ZoomIn className="w-4 h-4" />
        </button>
        <button
          onClick={handleZoomOut}
          className="bg-white dark:bg-gray-800 text-gray-700 dark:text-white w-8 h-8 rounded-lg border border-gray-300 dark:border-gray-700 shadow-sm hover:bg-gray-50 dark:hover:bg-gray-700 transition-colors flex items-center justify-center"
          title="Zoom out"
        >
          <ZoomOut className="w-4 h-4" />
        </button>
        <button
          onClick={handleReset}
          className="bg-white dark:bg-gray-800 text-gray-700 dark:text-white w-8 h-8 rounded-lg border border-gray-300 dark:border-gray-700 shadow-sm hover:bg-gray-50 dark:hover:bg-gray-700 transition-colors flex items-center justify-center"
          title="Reset view"
        >
          <RotateCcw className="w-4 h-4" />
        </button>
        <div className="w-full h-px bg-gray-300 dark:bg-gray-700" />
        <button
          onClick={retry}
          className="bg-white dark:bg-gray-800 text-gray-700 dark:text-white w-8 h-8 rounded-lg border border-gray-300 dark:border-gray-700 shadow-sm hover:bg-gray-50 dark:hover:bg-gray-700 transition-colors flex items-center justify-center"
          title="Refresh map data"
        >
          <RefreshCw className="w-4 h-4" />
        </button>
        <div className="w-full h-px bg-gray-300 dark:bg-gray-700" />
        <button
          onClick={() => setShowLocalCostmap(!showLocalCostmap)}
          className={`w-8 h-8 rounded-lg border shadow-sm transition-colors flex items-center justify-center ${
            showLocalCostmap
              ? 'bg-orange-500 dark:bg-orange-600 text-white border-orange-600 dark:border-orange-700 hover:bg-orange-600 dark:hover:bg-orange-700'
              : 'bg-white dark:bg-gray-800 text-gray-700 dark:text-white border-gray-300 dark:border-gray-700 hover:bg-gray-50 dark:hover:bg-gray-700'
          }`}
          title={showLocalCostmap ? "Hide local costmap" : "Show local costmap"}
        >
          <Layers className="w-4 h-4" />
        </button>
      </div>

      {/* Canvas container */}
      <div 
        ref={containerRef}
        className="w-full h-full relative"
        style={{ 
          cursor: isSettingDirection ? 'crosshair' : (isDraggingWaypoint ? 'move' : (isEditMode ? 'crosshair' : (viewState.isDragging ? 'grabbing' : 'grab'))), 
          userSelect: 'none' 
        }}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        onWheel={handleWheel}
      >
        <canvas 
          ref={canvasRef}
          className="absolute inset-0"
          style={{ imageRendering: 'pixelated' }}
        />
      </div>

      {/* Loading indicator */}
      {isLoading && (
        <div className="absolute inset-0 flex items-center justify-center bg-black bg-opacity-50">
          <div className="bg-white dark:bg-gray-800 rounded-lg p-6 text-center shadow-lg">
            <div className="w-8 h-8 border-4 border-gray-300 border-t-blue-500 rounded-full animate-spin mb-3 mx-auto"></div>
            <p className="text-sm font-medium text-gray-700 dark:text-white mb-2">Loading map...</p>
            <p className="text-xs text-gray-500 dark:text-gray-400">
              Waiting for map data from {mapTopic}
            </p>
            <p className="text-xs text-gray-400 dark:text-gray-500 mt-1">
              This may take up to 15 seconds
            </p>
          </div>
        </div>
      )}

      {/* Error message */}
      {error && (
        <div className="absolute inset-0 flex items-center justify-center">
          <div className="bg-white dark:bg-gray-800 rounded-lg p-6 text-center shadow-lg max-w-sm">
            <MapIcon className="w-12 h-12 mx-auto mb-2 text-red-500" />
            <p className="text-sm text-red-500 mb-4">{error}</p>
            <div className="space-y-2">
              <button
                onClick={retry}
                className="w-full px-4 py-2 bg-blue-500 hover:bg-blue-600 text-white rounded-lg transition-colors text-sm font-medium"
              >
                Retry Loading Map
              </button>
              <p className="text-xs text-gray-500 dark:text-gray-400">
                Make sure the map server is running on the robot
              </p>
            </div>
          </div>
        </div>
      )}

      {/* Legend */}
      {occupancyGrid && !isLoading && (
        <div className="absolute bottom-3 left-3 bg-white dark:bg-gray-800 bg-opacity-90 dark:bg-opacity-90 rounded-lg p-2 text-xs">
          <div className="flex items-center gap-2 mb-1">
            <MapPin className="w-3 h-3 text-violet-500" />
            <span className="text-gray-700 dark:text-gray-300">Robot Position</span>
          </div>
          {waypoints.length > 0 && (
            <div className="flex items-center gap-2 mb-1">
              <div className="w-3 h-3 bg-blue-500 rounded-full"></div>
              <span className="text-gray-700 dark:text-gray-300">Waypoints</span>
            </div>
          )}
          {isNav2Navigating && (
            <div className="flex items-center gap-2">
              <div className="w-3 h-3 bg-orange-500 rounded-full animate-pulse"></div>
              <span className="text-gray-700 dark:text-gray-300">
                Navigating ({nav2CurrentIndex + 1}/{waypoints.length})
              </span>
            </div>
          )}
          {!nav2Connected && (
            <div className="flex items-center gap-2 text-red-500">
              <X className="w-3 h-3" />
              <span className="text-xs">Nav2 disconnected</span>
            </div>
          )}
          {showNavPlan && navPlan && navPlan.poses.length > 0 && (
            <div className="flex items-center gap-2 mb-1">
              <div className="w-3 h-3 bg-green-500 rounded"></div>
              <span className="text-gray-700 dark:text-gray-300">Navigation Plan</span>
            </div>
          )}
          {showLocalCostmap && (
            <div className="flex items-center gap-2">
              <div className="w-3 h-3 rounded-sm" style={{ background: 'linear-gradient(to right, #22c55e, #facc15, #f97316, #dc2626)' }}></div>
              <span className="text-gray-700 dark:text-gray-300">Costmap</span>
            </div>
          )}
        </div>
      )}
    </div>
  );
}