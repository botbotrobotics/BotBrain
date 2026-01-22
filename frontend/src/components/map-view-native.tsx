'use client';

import React, { useEffect, useRef, useState, useCallback } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { MapPin, Map as MapIcon, ZoomIn, ZoomOut, RotateCcw, Navigation, Square, Route } from 'lucide-react';
import { useLanguage } from '@/contexts/LanguageContext';
import useOccupancyGrid from '@/hooks/ros/useOccupancyGrid';
import useMapPose from '@/hooks/ros/useMapPose';
import useNav2Goal from '@/hooks/ros/useNav2Goal';
import useRosNavPlan from '@/hooks/ros/useRosNavPlan';

interface MapViewNativeProps {
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

interface GoalPosition {
  x: number;
  y: number;
  theta: number;
}

export default function MapViewNative({ className = '' }: MapViewNativeProps) {
  const { connection } = useRobotConnection();
  const { t } = useLanguage();
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
  
  // Navigation goal functionality
  const { publishGoal, cancelNavigation, isConnected: nav2Connected } = useNav2Goal();
  const [isGoalMode, setIsGoalMode] = useState(false);
  const [goalPosition, setGoalPosition] = useState<GoalPosition | null>(null);
  const [isNavigating, setIsNavigating] = useState(false);
  const [isDraggingGoal, setIsDraggingGoal] = useState(false);
  const [tempGoalPosition, setTempGoalPosition] = useState<GoalPosition | null>(null);
  
  // Log pose updates to verify they're coming through
  useEffect(() => {
    if (robotPose) {
      console.log('Robot pose updated:', robotPose);
    }
  }, [robotPose]);

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

    // Draw map centered - IMPORTANT: This offset is applied to the map image position
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

    // Draw robot pose if available (even at 0,0)
    const currentRobotPose = robotPoseRef.current;
    if (currentRobotPose !== null && occupancyGrid.info.origin) {
      // Convert robot world position to map coordinates
      const mapX = (currentRobotPose.x - occupancyGrid.info.origin.position.x) / resolution;
      const mapY = (currentRobotPose.y - occupancyGrid.info.origin.position.y) / resolution;
      
      // Debug logging
      if (Math.random() < 0.01) { // Log occasionally to avoid spam
        console.log('Robot position debug:', {
          robotWorld: { x: currentRobotPose.x, y: currentRobotPose.y },
          mapOrigin: { x: occupancyGrid.info.origin.position.x, y: occupancyGrid.info.origin.position.y },
          mapCoords: { x: mapX, y: mapY },
          mapDimensions: { width: mapWidth, height: mapHeight },
          resolution: resolution,
          frameId: currentRobotPose.frameId
        });
      }
      
      // Convert to canvas coordinates (with Y flip)
      const canvasX = mapOffsetX + mapX;
      const canvasY = mapOffsetY + (mapHeight - mapY);

      // Draw robot marker
      ctx.save();
      ctx.translate(canvasX, canvasY);
      ctx.rotate(-currentRobotPose.theta); // Negative for correct orientation

      // Robot size scaled appropriately (reduced by ~35%)
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

      // Draw inner circle for depth
      ctx.fillStyle = '#a78bfa'; // Lighter purple (violet-400)
      ctx.beginPath();
      ctx.arc(0, 0, robotSize * 0.7, 0, Math.PI * 2);
      ctx.fill();

      // Draw direction arrow (pointing forward)
      ctx.fillStyle = 'white';
      ctx.strokeStyle = '#8b5cf6';
      ctx.lineWidth = 1.5 / viewState.scale;
      ctx.beginPath();
      // Arrow pointing to the right (robot's forward direction)
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
      
      // Draw small circles at each pose for visibility
      ctx.fillStyle = '#22c55e';
      for (let i = 0; i < navPlan.poses.length; i += 5) { // Draw every 5th point to avoid clutter
        const pose = navPlan.poses[i].pose.position;
        const x = mapOffsetX + (pose.x - occupancyGrid.info.origin.position.x) / resolution;
        const y = mapOffsetY + (mapHeight - (pose.y - occupancyGrid.info.origin.position.y) / resolution);
        
        ctx.beginPath();
        ctx.arc(x, y, 2 / viewState.scale, 0, Math.PI * 2);
        ctx.fill();
      }
      
      ctx.globalAlpha = 1; // Reset alpha
    }

    // Draw navigation goal if set (or temporary goal while dragging)
    const displayGoal = isDraggingGoal ? tempGoalPosition : goalPosition;
    if (displayGoal && occupancyGrid.info.origin) {
      // Convert goal world position to map coordinates
      const goalMapX = (displayGoal.x - occupancyGrid.info.origin.position.x) / resolution;
      const goalMapY = (displayGoal.y - occupancyGrid.info.origin.position.y) / resolution;
      
      // Convert to canvas coordinates (with Y flip)
      const goalCanvasX = mapOffsetX + goalMapX;
      const goalCanvasY = mapOffsetY + (mapHeight - goalMapY);

      // Draw goal marker
      ctx.save();
      ctx.translate(goalCanvasX, goalCanvasY);

      // Goal marker size
      const goalSize = 15 / viewState.scale;
      
      // Draw goal circle with pulsing effect
      const animationSpeed = isNavigating ? 0.006 : 0.003;
      const pulse = Math.sin(Date.now() * animationSpeed) * 0.2 + 0.8;
      
      // Outer ring
      ctx.strokeStyle = isDraggingGoal ? '#3b82f6' : (isNavigating ? '#f59e0b' : '#10b981'); // Blue while dragging
      ctx.lineWidth = 3 / viewState.scale;
      ctx.globalAlpha = isDraggingGoal ? 0.8 : pulse;
      ctx.beginPath();
      ctx.arc(0, 0, goalSize * 1.5, 0, Math.PI * 2);
      ctx.stroke();
      
      // Inner circle
      ctx.fillStyle = isDraggingGoal ? '#3b82f6' : (isNavigating ? '#f59e0b' : '#10b981');
      ctx.globalAlpha = isDraggingGoal ? 0.2 : 0.3;
      ctx.beginPath();
      ctx.arc(0, 0, goalSize, 0, Math.PI * 2);
      ctx.fill();
      
      // Center dot
      ctx.fillStyle = isDraggingGoal ? '#3b82f6' : (isNavigating ? '#f59e0b' : '#10b981');
      ctx.globalAlpha = 1;
      ctx.beginPath();
      ctx.arc(0, 0, goalSize * 0.3, 0, Math.PI * 2);
      ctx.fill();
      
      // Direction indicator
      ctx.rotate(-displayGoal.theta);
      ctx.strokeStyle = isDraggingGoal ? '#3b82f6' : (isNavigating ? '#f59e0b' : '#10b981'); // Blue while dragging
      ctx.lineWidth = isDraggingGoal ? 3 / viewState.scale : 2 / viewState.scale;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(goalSize * 2, 0); // Longer arrow while dragging
      ctx.stroke();
      
      // Arrow head
      ctx.beginPath();
      ctx.moveTo(goalSize * 2, 0);
      ctx.lineTo(goalSize * 1.6, -goalSize * 0.4);
      ctx.lineTo(goalSize * 1.6, goalSize * 0.4);
      ctx.closePath();
      ctx.fill();

      ctx.restore();
    }

    // Restore context state
    ctx.restore();

    // Draw grid origin if visible
    if (occupancyGrid.info.origin) {
      ctx.save();
      ctx.translate(containerWidth / 2, containerHeight / 2);
      ctx.translate(viewState.translateX, viewState.translateY);
      ctx.scale(viewState.scale, viewState.scale);

      // Draw origin marker
      const originX = mapOffsetX;
      const originY = mapOffsetY + mapHeight;
      
      ctx.strokeStyle = 'red';
      ctx.lineWidth = 2 / viewState.scale;
      ctx.beginPath();
      ctx.moveTo(originX - 10 / viewState.scale, originY);
      ctx.lineTo(originX + 10 / viewState.scale, originY);
      ctx.moveTo(originX, originY - 10 / viewState.scale);
      ctx.lineTo(originX, originY + 10 / viewState.scale);
      ctx.stroke();
      
      ctx.restore();
    }
  }, [occupancyGrid, viewState, goalPosition, isNavigating, tempGoalPosition, isDraggingGoal, showNavPlan, navPlan]);

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

  // Handle ESC key to cancel goal dragging
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && (isDraggingGoal || isGoalMode)) {
        setIsDraggingGoal(false);
        setTempGoalPosition(null);
        setIsGoalMode(false);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isDraggingGoal, isGoalMode]);

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
    
    if (isGoalMode && occupancyGrid) {
      // Start goal setting with drag for orientation
      const worldPos = screenToWorld(e.clientX, e.clientY);
      if (worldPos) {
        const goal = { x: worldPos.x, y: worldPos.y, theta: 0 };
        setTempGoalPosition(goal);
        setIsDraggingGoal(true);
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
    
    if (isDraggingGoal && tempGoalPosition) {
      // Update orientation based on mouse position
      const worldPos = screenToWorld(e.clientX, e.clientY);
      if (worldPos) {
        const dx = worldPos.x - tempGoalPosition.x;
        const dy = worldPos.y - tempGoalPosition.y;
        const theta = Math.atan2(dy, dx);
        setTempGoalPosition(prev => prev ? { ...prev, theta } : null);
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
    if (isDraggingGoal && tempGoalPosition) {
      // Finalize goal with orientation
      setGoalPosition(tempGoalPosition);
      if (publishGoal(tempGoalPosition.x, tempGoalPosition.y, tempGoalPosition.theta)) {
        setIsNavigating(true);
      }
      setIsGoalMode(false);
      setIsDraggingGoal(false);
      setTempGoalPosition(null);
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
    if (cancelNavigation()) {
      setIsNavigating(false);
      setGoalPosition(null);
    }
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
          <button
            onClick={handleStopNavigation}
            className="bg-red-500 dark:bg-red-600 text-white w-8 h-8 rounded-lg border border-red-600 dark:border-red-700 shadow-sm hover:bg-red-600 dark:hover:bg-red-700 transition-colors flex items-center justify-center"
            title="Stop navigation"
          >
            <Square className="w-4 h-4" />
          </button>
        ) : (
          <button
            onClick={() => setIsGoalMode(!isGoalMode)}
            className={`bg-white dark:bg-gray-800 text-gray-700 dark:text-white w-8 h-8 rounded-lg border shadow-sm transition-colors flex items-center justify-center ${
              isGoalMode 
                ? 'bg-emerald-500 dark:bg-emerald-600 text-white border-emerald-600 dark:border-emerald-700' 
                : 'border-gray-300 dark:border-gray-700 hover:bg-gray-50 dark:hover:bg-gray-700'
            }`}
            title={isGoalMode ? "Cancel goal setting" : "Set navigation goal"}
            disabled={!nav2Connected || isNavigating}
          >
            <Navigation className="w-4 h-4" />
          </button>
        )}
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
      </div>

      {/* Canvas container */}
      <div 
        ref={containerRef}
        className="w-full h-full relative"
        style={{ 
          cursor: isDraggingGoal ? 'crosshair' : (isGoalMode ? 'crosshair' : (viewState.isDragging ? 'grabbing' : 'grab')), 
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
      {occupancyGrid && !isLoading && (
        <div className="absolute bottom-3 left-3 bg-white dark:bg-gray-800 bg-opacity-90 dark:bg-opacity-90 rounded-lg p-2 text-xs">
          <div className="flex items-center gap-2 mb-1">
            <MapPin className="w-3 h-3 text-violet-500" />
            <span className="text-gray-700 dark:text-gray-300">Robot Position</span>
          </div>
          <div className="flex items-center gap-2 mb-1">
            <div className="w-3 h-3 bg-gray-700 dark:bg-gray-300"></div>
            <span className="text-gray-700 dark:text-gray-300">Walls</span>
          </div>
          {goalPosition && (
            <div className="flex items-center gap-2">
              <Navigation className="w-3 h-3 text-emerald-500" />
              <span className="text-gray-700 dark:text-gray-300">
                {isNavigating ? 'Navigating...' : 'Nav Goal'}
              </span>
            </div>
          )}
        </div>
      )}
      
      {/* Goal mode indicator */}
      {isGoalMode && (
        <div className="absolute top-3 left-3 bg-emerald-500 dark:bg-emerald-600 text-white rounded-lg px-3 py-2 text-sm shadow-lg">
          <div className="flex items-center gap-2">
            <Navigation className="w-4 h-4" />
            <span>Click on map to set navigation goal</span>
          </div>
        </div>
      )}
    </div>
  );
}