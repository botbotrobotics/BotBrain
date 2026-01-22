'use client';

import React, { useEffect, useRef, useState, useCallback } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { MapPin, Map as MapIcon, ZoomIn, ZoomOut, RotateCcw, Square, Route, X, Send, Home, Layers } from 'lucide-react';
import { useLanguage } from '@/contexts/LanguageContext';
import { useNotifications } from '@/contexts/NotificationsContext';
import useOccupancyGrid from '@/hooks/ros/useOccupancyGrid';
import useMapPose from '@/hooks/ros/useMapPose';
import useNav2Waypoints, { Waypoint } from '@/hooks/ros/useNav2Waypoints';
import useRosNavPlan from '@/hooks/ros/useRosNavPlan';
import useInitialPose from '@/hooks/ros/useInitialPose';
import { useActiveMission } from '@/contexts/ActiveMissionContext';
import { useNavigationTargets } from '@/contexts/NavigationTargetsContext';

interface MapViewNav2Props {
  className?: string;
}

interface ViewState {
  scale: number;
  translateX: number;
  translateY: number;
  isDragging: boolean;
  dragStartX: number;
  dragStartY: number;
}

export default function MapViewNav2({ className = '' }: MapViewNav2Props) {
  const { connection } = useRobotConnection();
  const { t } = useLanguage();
  const { dispatch: notificationDispatch } = useNotifications();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const animationFrameRef = useRef<number | undefined>(undefined);
  const mapTopic = '/map';
  const { occupancyGrid, isLoading, error } = useOccupancyGrid({ topicName: mapTopic });
  
  // Get robot position in map frame
  const robotPose = useMapPose();
  
  // Get navigation plan
  const { navPlan } = useRosNavPlan('/plan');
  const [showNavPlan, setShowNavPlan] = useState(false);

  // Local costmap overlay
  const [showLocalCostmap, setShowLocalCostmap] = useState(false);
  const { occupancyGrid: localCostmap } = useOccupancyGrid({
    topicName: '/local_costmap/costmap',
    enabled: showLocalCostmap,
    throttleMs: 0 // No throttling for real-time updates
  });
  const localCostmapRef = useRef(localCostmap);
  const costmapImageCacheRef = useRef<ImageData | null>(null);
  const costmapDataHashRef = useRef<string>('');
  
  // Navigation waypoints functionality
  const {
    navigateThroughWaypoints,
    cancelNavigation,
    isNavigating,
    currentWaypointIndex,
    navigationError,
    isConnected: nav2Connected
  } = useNav2Waypoints();

  // Active mission context for displaying mission waypoints on map
  const {
    missionWaypoints,
    currentWaypointIndex: missionWaypointIndex,
    isMissionActive
  } = useActiveMission();

  // Navigation targets context for AR overlay
  const { setSingleGoal } = useNavigationTargets();

  const [isWaypointMode, setIsWaypointMode] = useState(false);
  const [waypoint, setWaypoint] = useState<Waypoint | null>(null);
  const [isDraggingWaypoint, setIsDraggingWaypoint] = useState(false);
  const [tempWaypoint, setTempWaypoint] = useState<Waypoint | null>(null);
  const [hasNavigationStarted, setHasNavigationStarted] = useState(false);
  const [isStopping, setIsStopping] = useState(false);
  
  // Set Home mode state
  const [isSetHomeMode, setIsSetHomeMode] = useState(false);
  const [isDraggingHome, setIsDraggingHome] = useState(false);
  const [tempHomePose, setTempHomePose] = useState<{ x: number; y: number; theta: number } | null>(null);
  const { publishInitialPose } = useInitialPose();
  
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

  // Store local costmap in a ref to access in render loop
  useEffect(() => {
    localCostmapRef.current = localCostmap;
  }, [localCostmap]);

  // Monitor navigation state changes
  useEffect(() => {
    if (!isNavigating && hasNavigationStarted) {
      // Navigation just ended
      setHasNavigationStarted(false);
      setIsStopping(false); // Reset stopping state
      console.log('Navigation ended');

      // Clear single goal from AR overlay context
      setSingleGoal(null);

      // Check if waypoint was reached (single waypoint mode)
      if (waypoint !== null) {
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'success',
            title: 'Navigation Complete',
            message: 'Successfully reached destination'
          }
        });
        // Clear the waypoint after successful navigation
        setWaypoint(null);
      }
    }
  }, [isNavigating, hasNavigationStarted, waypoint, notificationDispatch, setSingleGoal]);

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
              // Cost values 1-99
              const normalized = cost / 99;

              if (normalized < 0.33) {
                // Green to yellow
                const t = normalized / 0.33;
                r = Math.round(34 + t * (250 - 34));
                g = Math.round(197 - t * (197 - 204));
                b = Math.round(94 - t * 94);
              } else if (normalized < 0.66) {
                // Yellow to orange
                const t = (normalized - 0.33) / 0.33;
                r = 250;
                g = Math.round(204 - t * (204 - 115));
                b = Math.round(t * 22);
              } else {
                // Orange to red
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
          // We need to position it relative to the robot's position in the map frame
          const currentPose = robotPoseRef.current;

          let drawX: number, drawY: number;

          if (currentPose) {
            // Calculate the costmap center in odom frame
            const costmapCenterOdomX = costmap.info.origin.position.x + (costmapWidth * costmapResolution) / 2;
            const costmapCenterOdomY = costmap.info.origin.position.y + (costmapHeight * costmapResolution) / 2;

            // The robot position in the map frame gives us the transform
            // Assuming odom and map frames are approximately aligned (which is common with AMCL/localization)
            // The costmap origin relative to robot = costmap.origin - robot_in_odom
            // We use robot's map position to place the costmap

            // Calculate where the costmap origin would be in the map frame
            // The costmap follows the robot, so its position relative to the robot should be preserved
            const robotOdomX = costmapCenterOdomX; // Assume robot is at costmap center in odom
            const robotOdomY = costmapCenterOdomY;

            // Offset from costmap center to costmap origin
            const offsetX = costmap.info.origin.position.x - robotOdomX;
            const offsetY = costmap.info.origin.position.y - robotOdomY;

            // Position costmap origin in map frame (robot position + offset)
            const costmapMapOriginX = currentPose.x + offsetX;
            const costmapMapOriginY = currentPose.y + offsetY;

            // Convert to canvas coordinates
            const costmapPixelX = (costmapMapOriginX - occupancyGrid.info.origin.position.x) / resolution;
            const costmapPixelY = (costmapMapOriginY - occupancyGrid.info.origin.position.y) / resolution;

            drawX = mapOffsetX + costmapPixelX;
            drawY = mapOffsetY + (mapHeight - costmapPixelY - costmapHeight * scaleFactor);
          } else {
            // Fallback: direct origin comparison (may be off due to frame difference)
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
      
      ctx.globalAlpha = 1; // Reset alpha
    }

    // Draw single navigation goal waypoint
    if (waypoint && occupancyGrid.info.origin) {
      const x = mapOffsetX + (waypoint.x - occupancyGrid.info.origin.position.x) / resolution;
      const y = mapOffsetY + (mapHeight - (waypoint.y - occupancyGrid.info.origin.position.y) / resolution);

      ctx.save();
      ctx.translate(x, y);

      // Goal marker size
      const markerSize = 12 / viewState.scale;

      // Draw goal circle
      ctx.fillStyle = isNavigating ? '#f59e0b' : '#3b82f6'; // Orange when navigating, blue otherwise
      ctx.strokeStyle = 'white';
      ctx.lineWidth = 2 / viewState.scale;
      ctx.beginPath();
      ctx.arc(0, 0, markerSize, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Draw direction indicator if waypoint has orientation
      if (waypoint.theta !== undefined) {
        ctx.rotate(-waypoint.theta);
        ctx.strokeStyle = isNavigating ? '#f59e0b' : '#3b82f6';
        ctx.lineWidth = 2 / viewState.scale;
        ctx.beginPath();
        ctx.moveTo(markerSize, 0);
        ctx.lineTo(markerSize * 2, 0);
        ctx.stroke();

        // Arrow head
        ctx.fillStyle = isNavigating ? '#f59e0b' : '#3b82f6';
        ctx.beginPath();
        ctx.moveTo(markerSize * 2, 0);
        ctx.lineTo(markerSize * 1.7, -markerSize * 0.3);
        ctx.lineTo(markerSize * 1.7, markerSize * 0.3);
        ctx.closePath();
        ctx.fill();
      }

      ctx.restore();
    }

    // Draw mission waypoints (view-only, from context)
    if (isMissionActive && missionWaypoints.length > 0 && occupancyGrid.info.origin) {
      // Draw path between mission waypoints
      if (missionWaypoints.length > 1) {
        ctx.strokeStyle = '#a855f7'; // Purple for missions
        ctx.lineWidth = 2 / viewState.scale;
        ctx.globalAlpha = 0.6;
        ctx.setLineDash([5 / viewState.scale, 5 / viewState.scale]);

        ctx.beginPath();
        missionWaypoints.forEach((wp, i) => {
          const wpX = mapOffsetX + (wp.x - occupancyGrid.info.origin.position.x) / resolution;
          const wpY = mapOffsetY + (mapHeight - (wp.y - occupancyGrid.info.origin.position.y) / resolution);
          if (i === 0) ctx.moveTo(wpX, wpY);
          else ctx.lineTo(wpX, wpY);
        });
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.globalAlpha = 1;
      }

      // Draw mission waypoint markers
      missionWaypoints.forEach((wp, index) => {
        const wpX = mapOffsetX + (wp.x - occupancyGrid.info.origin.position.x) / resolution;
        const wpY = mapOffsetY + (mapHeight - (wp.y - occupancyGrid.info.origin.position.y) / resolution);

        ctx.save();
        ctx.translate(wpX, wpY);

        const markerSize = 10 / viewState.scale;
        const isCurrentMissionWaypoint = index === missionWaypointIndex;
        const isReached = index < missionWaypointIndex;

        // Color: green if reached, orange if current, purple if pending
        ctx.fillStyle = isReached ? '#22c55e' : (isCurrentMissionWaypoint ? '#f59e0b' : '#a855f7');
        ctx.strokeStyle = 'white';
        ctx.lineWidth = 2 / viewState.scale;
        ctx.beginPath();
        ctx.arc(0, 0, markerSize, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // Draw number
        ctx.fillStyle = 'white';
        ctx.font = `bold ${8 / viewState.scale}px Arial`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText((index + 1).toString(), 0, 0);

        // Draw direction arrow
        if (wp.theta !== undefined) {
          ctx.rotate(-wp.theta);
          ctx.strokeStyle = isReached ? '#22c55e' : (isCurrentMissionWaypoint ? '#f59e0b' : '#a855f7');
          ctx.lineWidth = 2 / viewState.scale;
          ctx.beginPath();
          ctx.moveTo(markerSize, 0);
          ctx.lineTo(markerSize * 1.8, 0);
          ctx.stroke();
        }

        ctx.restore();
      });
    }

    // Draw temporary home position while dragging
    if (tempHomePose && occupancyGrid.info.origin) {
      const x = mapOffsetX + (tempHomePose.x - occupancyGrid.info.origin.position.x) / resolution;
      const y = mapOffsetY + (mapHeight - (tempHomePose.y - occupancyGrid.info.origin.position.y) / resolution);

      ctx.save();
      ctx.translate(x, y);

      // Temporary home marker
      const markerSize = 14 / viewState.scale;
      
      // Draw house icon
      ctx.fillStyle = 'rgba(34, 197, 94, 0.5)'; // Semi-transparent green
      ctx.strokeStyle = '#22c55e';
      ctx.lineWidth = 2 / viewState.scale;
      ctx.setLineDash([3 / viewState.scale, 3 / viewState.scale]);
      
      // Draw circle background
      ctx.beginPath();
      ctx.arc(0, 0, markerSize, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.setLineDash([]);

      // Draw house symbol
      ctx.fillStyle = 'white';
      ctx.strokeStyle = 'white';
      ctx.lineWidth = 1.5 / viewState.scale;
      const houseSize = markerSize * 0.6;
      
      // House body
      ctx.fillRect(-houseSize/2, -houseSize/4, houseSize, houseSize/2);
      
      // Roof
      ctx.beginPath();
      ctx.moveTo(-houseSize * 0.7, -houseSize/4);
      ctx.lineTo(0, -houseSize * 0.7);
      ctx.lineTo(houseSize * 0.7, -houseSize/4);
      ctx.closePath();
      ctx.fill();

      // Draw direction if set
      if (tempHomePose.theta !== undefined) {
        ctx.rotate(-tempHomePose.theta);
        ctx.strokeStyle = '#22c55e';
        ctx.lineWidth = 2 / viewState.scale;
        ctx.beginPath();
        ctx.moveTo(markerSize, 0);
        ctx.lineTo(markerSize * 2.5, 0);
        ctx.stroke();
        
        // Arrow head
        ctx.fillStyle = '#22c55e';
        ctx.beginPath();
        ctx.moveTo(markerSize * 2.5, 0);
        ctx.lineTo(markerSize * 2.2, -markerSize * 0.3);
        ctx.lineTo(markerSize * 2.2, markerSize * 0.3);
        ctx.closePath();
        ctx.fill();
      }

      ctx.restore();
    }

    // Draw temporary waypoint while dragging
    if (tempWaypoint && occupancyGrid.info.origin) {
      const x = mapOffsetX + (tempWaypoint.x - occupancyGrid.info.origin.position.x) / resolution;
      const y = mapOffsetY + (mapHeight - (tempWaypoint.y - occupancyGrid.info.origin.position.y) / resolution);

      ctx.save();
      ctx.translate(x, y);

      // Temporary waypoint marker
      const markerSize = 12 / viewState.scale;
      
      ctx.fillStyle = 'rgba(59, 130, 246, 0.5)'; // Semi-transparent blue
      ctx.strokeStyle = '#3b82f6';
      ctx.lineWidth = 2 / viewState.scale;
      ctx.setLineDash([3 / viewState.scale, 3 / viewState.scale]);
      ctx.beginPath();
      ctx.arc(0, 0, markerSize, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.setLineDash([]);

      // Draw direction if set
      if (tempWaypoint.theta !== undefined) {
        ctx.rotate(-tempWaypoint.theta);
        ctx.strokeStyle = '#3b82f6';
        ctx.lineWidth = 2 / viewState.scale;
        ctx.beginPath();
        ctx.moveTo(markerSize, 0);
        ctx.lineTo(markerSize * 2.5, 0);
        ctx.stroke();
      }

      ctx.restore();
    }

    // Restore context state
    ctx.restore();
  }, [occupancyGrid, viewState, waypoint, tempWaypoint, isDraggingWaypoint, showNavPlan, navPlan, isNavigating, tempHomePose, showLocalCostmap, isMissionActive, missionWaypoints, missionWaypointIndex]);

  // Initial scale calculation when map loads
  useEffect(() => {
    if (!occupancyGrid || !containerRef.current) return;

    const container = containerRef.current;
    const mapWidth = occupancyGrid.info.width;
    const mapHeight = occupancyGrid.info.height;

    // Calculate scale so map fits exactly with at least one edge touching the container
    const scaleX = container.clientWidth / mapWidth;
    const scaleY = container.clientHeight / mapHeight;
    const initialScale = Math.min(scaleX, scaleY);

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

  // Handle ESC key to cancel waypoint dragging
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        if (isDraggingWaypoint || isWaypointMode) {
          setIsDraggingWaypoint(false);
          setTempWaypoint(null);
          setIsWaypointMode(false);
        }
        if (isDraggingHome || isSetHomeMode) {
          setIsDraggingHome(false);
          setTempHomePose(null);
          setIsSetHomeMode(false);
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isDraggingWaypoint, isWaypointMode, isDraggingHome, isSetHomeMode]);

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

  // Mouse event handlers
  const handleMouseDown = (e: React.MouseEvent) => {
    e.preventDefault();

    if (isSetHomeMode && occupancyGrid) {
      // Start setting home position with drag for orientation
      const worldPos = screenToWorld(e.clientX, e.clientY);
      if (worldPos) {
        const homePose = { x: worldPos.x, y: worldPos.y, theta: 0 };
        setTempHomePose(homePose);
        setIsDraggingHome(true);
      }
    } else if (isWaypointMode && occupancyGrid) {
      // Single waypoint mode - set new waypoint with drag for orientation
      const worldPos = screenToWorld(e.clientX, e.clientY);
      if (worldPos) {
        const newWaypoint = { x: worldPos.x, y: worldPos.y, theta: 0 };
        setTempWaypoint(newWaypoint);
        setIsDraggingWaypoint(true);
      }
    } else {
      // Normal pan mode
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
    
    if (isDraggingHome && tempHomePose) {
      // Update orientation based on mouse position
      const worldPos = screenToWorld(e.clientX, e.clientY);
      if (worldPos) {
        const dx = worldPos.x - tempHomePose.x;
        const dy = worldPos.y - tempHomePose.y;
        const theta = Math.atan2(dy, dx);
        setTempHomePose(prev => prev ? { ...prev, theta } : null);
      }
    } else if (isDraggingWaypoint && tempWaypoint) {
      // Update orientation based on mouse position
      const worldPos = screenToWorld(e.clientX, e.clientY);
      if (worldPos) {
        const dx = worldPos.x - tempWaypoint.x;
        const dy = worldPos.y - tempWaypoint.y;
        const theta = Math.atan2(dy, dx);
        setTempWaypoint(prev => prev ? { ...prev, theta } : null);
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
    if (isDraggingHome && tempHomePose) {
      // Set home position
      const success = publishInitialPose(tempHomePose);
      if (success) {
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'success',
            title: 'Home Position Set',
            message: `Robot home position set at (${tempHomePose.x.toFixed(2)}, ${tempHomePose.y.toFixed(2)})`
          }
        });
      } else {
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'error',
            title: 'Failed to Set Home',
            message: 'Could not set home position. Please check ROS connection.'
          }
        });
      }
      setIsDraggingHome(false);
      setTempHomePose(null);
      setIsSetHomeMode(false);
    } else if (isDraggingWaypoint && tempWaypoint) {
      // Set single waypoint (replaces any existing waypoint)
      setWaypoint(tempWaypoint);
      setIsDraggingWaypoint(false);
      setTempWaypoint(null);
      // Exit waypoint mode after placing the goal
      setIsWaypointMode(false);
    } else {
      // Normal pan mode
      setViewState(prev => ({
        ...prev,
        isDragging: false
      }));
    }
  };

  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    const delta = e.deltaY > 0 ? 0.9 : 1.1;
    setViewState(prev => ({
      ...prev,
      scale: Math.max(0.1, Math.min(10, prev.scale * delta))
    }));
  };

  // Touch event handlers for mobile
  const touchStartRef = useRef<{ distance: number; scale: number }>({ distance: 0, scale: 1 });

  const handleTouchStart = (e: React.TouchEvent) => {
    if (e.touches.length === 2) {
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      touchStartRef.current = {
        distance: Math.sqrt(dx * dx + dy * dy),
        scale: viewState.scale
      };
    } else if (e.touches.length === 1) {
      setViewState(prev => ({
        ...prev,
        isDragging: true,
        dragStartX: e.touches[0].clientX - prev.translateX,
        dragStartY: e.touches[0].clientY - prev.translateY
      }));
    }
  };

  const handleTouchMove = (e: React.TouchEvent) => {
    e.preventDefault();
    
    if (e.touches.length === 2) {
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      const distance = Math.sqrt(dx * dx + dy * dy);
      const scale = touchStartRef.current.scale * (distance / touchStartRef.current.distance);
      
      setViewState(prev => ({
        ...prev,
        scale: Math.max(0.1, Math.min(10, scale))
      }));
    } else if (e.touches.length === 1 && viewState.isDragging) {
      setViewState(prev => ({
        ...prev,
        translateX: e.touches[0].clientX - prev.dragStartX,
        translateY: e.touches[0].clientY - prev.dragStartY
      }));
    }
  };

  const handleTouchEnd = () => {
    setViewState(prev => ({
      ...prev,
      isDragging: false
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

    // Calculate scale so map fits exactly with at least one edge touching the container
    const scaleX = container.clientWidth / mapWidth;
    const scaleY = container.clientHeight / mapHeight;
    const resetScale = Math.min(scaleX, scaleY);

    setViewState({
      scale: resetScale,
      translateX: 0,
      translateY: 0,
      isDragging: false,
      dragStartX: 0,
      dragStartY: 0
    });
  };

  const handleStopNavigation = () => {
    console.log('=== STOP BUTTON CLICKED ===');
    console.log('Current state:', {
      isNavigating,
      hasNavigationStarted,
      currentWaypointIndex,
      hasWaypoint: waypoint !== null,
      nav2Connected
    });

    setIsStopping(true);

    // Cancel navigation with robust method from missions
    try {
      const success = cancelNavigation();
      console.log('Cancel navigation result:', success);

      if (success) {
        console.log('Navigation cancelled successfully');
        setHasNavigationStarted(false);
        // Don't clear waypoint, just stop navigation
        // This allows user to resume or modify the goal
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'info',
            title: 'Navigation Stopped',
            message: 'Robot navigation has been cancelled'
          }
        });
      } else {
        console.error('Failed to cancel navigation - cancelNavigation returned false');
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'error',
            title: 'Stop Failed',
            message: 'Failed to stop navigation. Please try again.'
          }
        });
      }
    } catch (error) {
      console.error('Exception during cancel navigation:', error);
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'error',
          title: 'Stop Failed',
          message: 'An error occurred while stopping navigation.'
        }
      });
    }
    
    // Reset stopping state after a short delay
    setTimeout(() => {
      setIsStopping(false);
    }, 1000);
  };

  const handleSendWaypoint = () => {
    if (waypoint) {
      console.log('Sending waypoint:', waypoint);
      const success = navigateThroughWaypoints([waypoint]);
      if (success) {
        setIsWaypointMode(false);
        setHasNavigationStarted(true);
        // Update navigation targets context for AR overlay
        setSingleGoal({ x: waypoint.x, y: waypoint.y, theta: waypoint.theta });
        console.log('Navigation started successfully');
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'success',
            title: 'Navigation Started',
            message: 'Navigating to destination'
          }
        });
      } else {
        console.error('Failed to start navigation:', navigationError);
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'error',
            title: 'Navigation Failed',
            message: navigationError || 'Failed to start navigation. Please check Nav2 is running.'
          }
        });
      }
    }
  };

  const handleClearWaypoint = () => {
    setWaypoint(null);
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

      {/* Navigation and zoom controls */}
      <div className="absolute top-3 right-3 z-10 flex flex-col gap-2">
        {isNavigating ? (
          <div className="relative group">
            <button
              onClick={handleStopNavigation}
              disabled={isStopping}
              className={`${
                isStopping 
                  ? 'bg-orange-500 dark:bg-orange-600 border-orange-600 dark:border-orange-700' 
                  : 'bg-red-500 dark:bg-red-600 border-red-600 dark:border-red-700 hover:bg-red-600 dark:hover:bg-red-700 active:bg-red-700 dark:active:bg-red-800 animate-pulse'
              } text-white w-12 h-12 rounded-lg border-2 shadow-lg transition-all transform hover:scale-110 active:scale-95 flex items-center justify-center relative z-10`}
              title={isStopping ? "Stopping navigation..." : "Stop navigation - Click to stop the robot"}
            >
              {isStopping ? (
                <div className="w-6 h-6 border-3 border-white border-t-transparent rounded-full animate-spin" />
              ) : (
                <Square className="w-6 h-6" />
              )}
            </button>
            {/* Tooltip */}
            {!isStopping && (
              <div className="absolute top-full mt-2 left-1/2 -translate-x-1/2 opacity-0 group-hover:opacity-100 transition-opacity pointer-events-none z-20">
                <div className="bg-gray-800 text-white text-xs rounded px-2 py-1 whitespace-nowrap">
                  Stop Navigation
                  <div className="absolute -top-1 left-1/2 -translate-x-1/2 w-0 h-0 border-l-4 border-r-4 border-b-4 border-transparent border-b-gray-800"></div>
                </div>
              </div>
            )}
          </div>
        ) : (
          <>
            <button
              onClick={() => {
                setIsSetHomeMode(!isSetHomeMode);
                if (isWaypointMode) setIsWaypointMode(false);
              }}
              className={`bg-white dark:bg-gray-800 text-gray-700 dark:text-white w-8 h-8 rounded-lg border shadow-sm transition-colors flex items-center justify-center ${
                isSetHomeMode 
                  ? 'bg-green-500 dark:bg-green-600 text-white border-green-600 dark:border-green-700' 
                  : 'border-gray-300 dark:border-gray-700 hover:bg-gray-50 dark:hover:bg-gray-700'
              }`}
              title={isSetHomeMode ? "Cancel set home" : "Set home position"}
              disabled={!nav2Connected}
            >
              <Home className="w-4 h-4" />
            </button>
            <button
              onClick={() => {
                setIsWaypointMode(!isWaypointMode);
                if (isSetHomeMode) setIsSetHomeMode(false);
              }}
              className={`bg-white dark:bg-gray-800 text-gray-700 dark:text-white w-8 h-8 rounded-lg border shadow-sm transition-colors flex items-center justify-center ${
                isWaypointMode
                  ? 'bg-blue-500 dark:bg-blue-600 text-white border-blue-600 dark:border-blue-700'
                  : 'border-gray-300 dark:border-gray-700 hover:bg-gray-50 dark:hover:bg-gray-700'
              }`}
              title={isWaypointMode ? "Cancel navigation mode" : "Navigate to location"}
              disabled={!nav2Connected || isNavigating}
            >
              {isWaypointMode ? <X className="w-4 h-4" /> : <MapPin className="w-4 h-4" />}
            </button>
            {waypoint && (
              <>
                <button
                  onClick={handleSendWaypoint}
                  className="bg-emerald-500 dark:bg-emerald-600 text-white w-8 h-8 rounded-lg border border-emerald-600 dark:border-emerald-700 shadow-sm hover:bg-emerald-600 dark:hover:bg-emerald-700 transition-colors flex items-center justify-center"
                  title="Navigate to location"
                  disabled={!nav2Connected}
                >
                  <Send className="w-4 h-4" />
                </button>
                <button
                  onClick={handleClearWaypoint}
                  className="bg-gray-500 dark:bg-gray-600 text-white w-8 h-8 rounded-lg border border-gray-600 dark:border-gray-700 shadow-sm hover:bg-gray-600 dark:hover:bg-gray-700 transition-colors flex items-center justify-center"
                  title="Clear navigation goal"
                >
                  <X className="w-4 h-4" />
                </button>
              </>
            )}
          </>
        )}
        <div className="w-8 h-0.5 bg-gray-300 dark:bg-gray-700" />
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
        <button
          onClick={() => setShowNavPlan(!showNavPlan)}
          className={`w-8 h-8 rounded-lg border shadow-sm transition-colors flex items-center justify-center ${
            showNavPlan
              ? 'bg-green-500 dark:bg-green-600 text-white border-green-600 dark:border-green-700 hover:bg-green-600 dark:hover:bg-green-700'
              : 'bg-white dark:bg-gray-800 text-gray-700 dark:text-white border-gray-300 dark:border-gray-700 hover:bg-gray-50 dark:hover:bg-gray-700'
          }`}
          title={showNavPlan ? "Hide navigation plan" : "Show navigation plan"}
        >
          <Route className="w-4 h-4" />
        </button>
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

      {/* Mission waypoints indicator */}
      {isMissionActive && missionWaypoints.length > 0 && (
        <div className="absolute top-3 left-3 z-10 bg-white dark:bg-gray-800 rounded-lg px-3 py-1 shadow-sm border border-purple-300 dark:border-purple-700">
          <span className="text-sm text-purple-600 dark:text-purple-400">
            Mission: {missionWaypointIndex + 1}/{missionWaypoints.length}
          </span>
        </div>
      )}

      {/* Navigation goal indicator - bottom left */}
      {waypoint && !isNavigating && !isStopping && (
        <div className="absolute bottom-3 left-3 z-10 bg-white dark:bg-gray-800 rounded-lg px-3 py-1 shadow-sm border border-gray-300 dark:border-gray-700">
          <span className="text-sm text-gray-700 dark:text-gray-300">Goal set</span>
        </div>
      )}

      {/* Navigation feedback */}
      {(isNavigating || isStopping) && !isMissionActive && (
        <div className="absolute bottom-3 left-3 z-10 bg-white dark:bg-gray-800 rounded-lg px-3 py-2 shadow-sm border border-gray-300 dark:border-gray-700 max-w-xs">
          <div className="text-xs text-gray-700 dark:text-gray-300 space-y-1">
            {isStopping ? (
              <div className="flex items-center gap-2 text-orange-500">
                <div className="w-3 h-3 border-2 border-orange-500 border-t-transparent rounded-full animate-spin" />
                <span>Stopping navigation...</span>
              </div>
            ) : (
              <>
                <div>Navigating to destination...</div>
                {navigationError && (
                  <div className="text-red-500">{navigationError}</div>
                )}
              </>
            )}
          </div>
        </div>
      )}

      {/* Canvas container */}
      <div 
        ref={containerRef}
        className="w-full h-full relative"
        style={{ 
          cursor: (isDraggingHome || isSetHomeMode) ? 'crosshair' : (isDraggingWaypoint || isWaypointMode) ? 'crosshair' : (viewState.isDragging ? 'grabbing' : 'grab'), 
          userSelect: 'none' 
        }}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        onWheel={handleWheel}
        onTouchStart={handleTouchStart}
        onTouchMove={handleTouchMove}
        onTouchEnd={handleTouchEnd}
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
          <div className="text-center">
            <div className="w-8 h-8 border-4 border-gray-300 border-t-white rounded-full animate-spin mb-2"></div>
            <p className="text-sm text-white">Loading map...</p>
          </div>
        </div>
      )}

      {/* Error message */}
      {error && (
        <div className="absolute inset-0 flex items-center justify-center">
          <div className="text-center text-red-500">
            <MapIcon className="w-12 h-12 mx-auto mb-2" />
            <p className="text-sm">{error}</p>
          </div>
        </div>
      )}

      {/* Legend */}
      {occupancyGrid && (
        <div className="absolute bottom-3 right-3 bg-white dark:bg-gray-800 bg-opacity-90 dark:bg-opacity-90 rounded-lg p-2 text-xs">
          <div className="flex items-center gap-2 mb-1">
            <MapPin className="w-3 h-3 text-purple-500" />
            <span className="text-gray-700 dark:text-gray-300">Robot</span>
          </div>
          {(isWaypointMode || waypoint) && (
            <div className="flex items-center gap-2 mb-1">
              <div className="w-3 h-3 bg-blue-500 rounded-full"></div>
              <span className="text-gray-700 dark:text-gray-300">Goal</span>
            </div>
          )}
          {isMissionActive && missionWaypoints.length > 0 && (
            <div className="flex items-center gap-2 mb-1">
              <div className="w-3 h-3 bg-purple-500 rounded-full"></div>
              <span className="text-gray-700 dark:text-gray-300">Mission</span>
            </div>
          )}
          {showNavPlan && (
            <div className="flex items-center gap-2 mb-1">
              <div className="w-3 h-0.5 bg-green-500"></div>
              <span className="text-gray-700 dark:text-gray-300">Plan</span>
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

      {/* Instructions */}
      {isWaypointMode && (
        <div className="absolute top-14 left-3 z-10 bg-white dark:bg-gray-800 rounded-lg px-3 py-2 shadow-sm border border-gray-300 dark:border-gray-700">
          <p className="text-xs text-gray-700 dark:text-gray-300">
            Click to set navigation goal • Drag to set orientation • ESC to cancel
          </p>
        </div>
      )}
      {isSetHomeMode && (
        <div className="absolute top-14 left-3 z-10 bg-white dark:bg-gray-800 rounded-lg px-3 py-2 shadow-sm border border-gray-300 dark:border-gray-700">
          <p className="text-xs text-gray-700 dark:text-gray-300">
            Click to set home position • Drag to set orientation • ESC to cancel
          </p>
        </div>
      )}
    </div>
  );
}