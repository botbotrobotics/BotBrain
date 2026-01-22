'use client';

import React, { useEffect, useRef, useState, useCallback } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { ZoomIn, ZoomOut, RotateCcw, Navigation, Home } from 'lucide-react';
import useOccupancyGrid from '@/hooks/ros/useOccupancyGrid';
import useMapPose from '@/hooks/ros/useMapPose';
import useInitialPose from '@/hooks/ros/useInitialPose';
import { useNotifications } from '@/contexts/NotificationsContext';

interface SimpleMapViewProps {
  className?: string;
  isMapping?: boolean;
  isSettingHome?: boolean;
  onHomeSet?: (position: {x: number, y: number, theta: number}) => void;
}

interface ViewState {
  scale: number;
  translateX: number;
  translateY: number;
  isDragging: boolean;
  dragStartX: number;
  dragStartY: number;
}

export default function SimpleMapView({ className = '', isMapping = false, isSettingHome = false, onHomeSet }: SimpleMapViewProps) {
  const { connection } = useRobotConnection();
  const { dispatch: notificationDispatch } = useNotifications();
  const { publishInitialPose } = useInitialPose();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const animationFrameRef = useRef<number | undefined>(undefined);
  const mapTopic = '/map';
  const { occupancyGrid, isLoading, error } = useOccupancyGrid({ topicName: mapTopic });
  
  // Get robot position in map frame
  const robotPose = useMapPose();
  
  const [viewState, setViewState] = useState<ViewState>({
    scale: 2,  // Start with 2x zoom for better visibility
    translateX: 0,
    translateY: 0,
    isDragging: false,
    dragStartX: 0,
    dragStartY: 0
  });

  const [hasInitialized, setHasInitialized] = useState(false);
  
  // Set Home mode state
  const [isDraggingHome, setIsDraggingHome] = useState(false);
  const [tempHomePose, setTempHomePose] = useState<{ x: number; y: number; theta: number } | null>(null);

  // Store robot pose in a ref to access in render loop without causing re-creation
  const robotPoseRef = useRef(robotPose);

  useEffect(() => {
    robotPoseRef.current = robotPose;
  }, [robotPose]);

  // Auto-fit map when it first loads
  useEffect(() => {
    if (occupancyGrid && containerRef.current && !hasInitialized) {
      const container = containerRef.current;
      const containerWidth = container.clientWidth;
      const containerHeight = container.clientHeight;

      const mapWidth = occupancyGrid.info.width;
      const mapHeight = occupancyGrid.info.height;

      // Calculate scale so map fits exactly with at least one edge touching the container
      const scaleX = containerWidth / mapWidth;
      const scaleY = containerHeight / mapHeight;
      const scale = Math.min(scaleX, scaleY);

      setViewState(prev => ({
        ...prev,
        scale: scale,
        translateX: 0,
        translateY: 0
      }));

      setHasInitialized(true);
    }
  }, [occupancyGrid, hasInitialized]);

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

    // Create offscreen canvas for the map
    const offscreenCanvas = document.createElement('canvas');
    offscreenCanvas.width = mapWidth;
    offscreenCanvas.height = mapHeight;
    const offscreenCtx = offscreenCanvas.getContext('2d');
    
    if (offscreenCtx) {
      // Create image data for the map
      const imageData = offscreenCtx.createImageData(mapWidth, mapHeight);
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
          } else if (value === 0) {
            // Free space - light gray
            r = g = b = 200;
          } else {
            // Occupied - black
            r = g = b = 50;
          }
          
          data[dstIdx] = r;
          data[dstIdx + 1] = g;
          data[dstIdx + 2] = b;
          data[dstIdx + 3] = 255;
        }
      }

      // Put image data on offscreen canvas
      offscreenCtx.putImageData(imageData, 0, 0);
      
      // Draw the offscreen canvas to the main canvas
      ctx.drawImage(offscreenCanvas, mapOffsetX, mapOffsetY);
    }

    // Draw robot position if available
    if (robotPoseRef.current && occupancyGrid) {
      const origin = occupancyGrid.info.origin;
      const robotX = (robotPoseRef.current.x - origin.position.x) / resolution;
      const robotY = (robotPoseRef.current.y - origin.position.y) / resolution;
      
      // Convert to canvas coordinates (with Y flip for correct orientation)
      const canvasX = mapOffsetX + robotX;
      const canvasY = mapOffsetY + (mapHeight - robotY);
      
      // Draw robot
      ctx.save();
      ctx.translate(canvasX, canvasY);
      
      const robotSize = 15 / viewState.scale;
      const time = Date.now() * 0.001; // Convert to seconds for smoother animation
      
      // Animation values - different for mapping mode
      const breathingScale = isMapping 
        ? 1 + Math.sin(time * 4) * 0.15  // Faster, more pronounced breathing when mapping
        : 1 + Math.sin(time * 2) * 0.1;  // Gentle breathing effect normally
      
      const glowIntensity = isMapping
        ? 0.5 + Math.sin(time * 6) * 0.4  // Stronger, faster flashing when mapping
        : 0.4 + Math.sin(time * 3) * 0.2; // Varying glow intensity normally
      
      // Color scheme - green for mapping, purple for normal
      const primaryColor = isMapping ? '#10b981' : '#8b5cf6';     // emerald-500 : violet-500
      const lightColor = isMapping ? '#34d399' : '#c084fc';       // emerald-400 : purple-400
      const mediumColor = isMapping ? '#059669' : '#a855f7';      // emerald-600 : purple-500
      const darkColor = isMapping ? '#047857' : '#9333ea';        // emerald-700 : purple-600
      const deepColor = isMapping ? '#065f46' : '#7c3aed';        // emerald-800 : purple-700
      
      // Draw multiple glow layers for depth
      // Outer glow layer 1 (largest, most transparent)
      ctx.save();
      ctx.globalAlpha = glowIntensity * 0.15;
      ctx.fillStyle = lightColor;
      ctx.shadowColor = lightColor;
      ctx.shadowBlur = isMapping ? 50 / viewState.scale : 40 / viewState.scale;
      ctx.beginPath();
      ctx.arc(0, 0, robotSize * 3.5 * breathingScale, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      
      // Outer glow layer 2
      ctx.save();
      ctx.globalAlpha = glowIntensity * 0.25;
      ctx.fillStyle = mediumColor;
      ctx.shadowColor = mediumColor;
      ctx.shadowBlur = isMapping ? 35 / viewState.scale : 25 / viewState.scale;
      ctx.beginPath();
      ctx.arc(0, 0, robotSize * 2.5 * breathingScale, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      
      // Middle glow layer
      ctx.save();
      ctx.globalAlpha = glowIntensity * 0.4;
      ctx.fillStyle = darkColor;
      ctx.shadowColor = darkColor;
      ctx.shadowBlur = isMapping ? 20 / viewState.scale : 15 / viewState.scale;
      ctx.beginPath();
      ctx.arc(0, 0, robotSize * 1.8 * breathingScale, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      
      // Inner glow layer
      ctx.save();
      ctx.globalAlpha = glowIntensity * 0.6;
      const gradient = ctx.createRadialGradient(0, 0, 0, 0, 0, robotSize * 1.5);
      gradient.addColorStop(0, mediumColor);
      gradient.addColorStop(0.5, darkColor);
      gradient.addColorStop(1, isMapping ? 'rgba(16, 185, 129, 0)' : 'rgba(147, 51, 234, 0)');
      ctx.fillStyle = gradient;
      ctx.beginPath();
      ctx.arc(0, 0, robotSize * 1.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      
      // Draw robot body with enhanced styling
      ctx.save();
      ctx.scale(breathingScale, breathingScale);
      
      // Robot shadow for depth
      ctx.fillStyle = 'rgba(0, 0, 0, 0.3)';
      ctx.beginPath();
      ctx.arc(2 / viewState.scale, 2 / viewState.scale, robotSize, 0, Math.PI * 2);
      ctx.fill();
      
      // Main robot body with gradient
      const bodyGradient = ctx.createRadialGradient(
        -robotSize * 0.3, -robotSize * 0.3, 0,
        0, 0, robotSize
      );
      bodyGradient.addColorStop(0, mediumColor); // Lighter at highlight
      bodyGradient.addColorStop(0.7, primaryColor); // Main color
      bodyGradient.addColorStop(1, deepColor); // Darker at edge
      ctx.fillStyle = bodyGradient;
      ctx.strokeStyle = 'white';
      ctx.lineWidth = isMapping ? 3 / viewState.scale : 2 / viewState.scale;
      ctx.shadowColor = primaryColor;
      ctx.shadowBlur = isMapping ? 15 / viewState.scale : 10 / viewState.scale;
      ctx.beginPath();
      ctx.arc(0, 0, robotSize, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      
      // Inner highlight for glossy effect
      ctx.save();
      ctx.globalAlpha = 0.4;
      ctx.fillStyle = 'white';
      ctx.beginPath();
      ctx.arc(-robotSize * 0.3, -robotSize * 0.3, robotSize * 0.4, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      
      ctx.restore();
      
      // Draw direction arrow with glow
      ctx.rotate(-robotPoseRef.current.theta);
      
      // Arrow glow
      ctx.save();
      ctx.globalAlpha = glowIntensity;
      ctx.strokeStyle = lightColor;
      ctx.lineWidth = isMapping ? 8 / viewState.scale : 6 / viewState.scale;
      ctx.shadowColor = lightColor;
      ctx.shadowBlur = isMapping ? 12 / viewState.scale : 8 / viewState.scale;
      ctx.beginPath();
      const arrowLength = robotSize * 2;
      const arrowWidth = robotSize * 0.6;
      ctx.moveTo(robotSize * 0.8, 0);
      ctx.lineTo(arrowLength, 0);
      ctx.stroke();
      ctx.restore();
      
      // Main arrow
      ctx.fillStyle = 'white';
      ctx.strokeStyle = primaryColor;
      ctx.lineWidth = 1.5 / viewState.scale;
      ctx.shadowColor = primaryColor;
      ctx.shadowBlur = isMapping ? 6 / viewState.scale : 4 / viewState.scale;
      ctx.beginPath();
      ctx.moveTo(arrowLength + arrowWidth * 0.3, 0);
      ctx.lineTo(arrowLength - arrowWidth * 0.7, -arrowWidth);
      ctx.lineTo(arrowLength - arrowWidth * 0.7, -arrowWidth * 0.3);
      ctx.lineTo(robotSize * 0.8, -arrowWidth * 0.3);
      ctx.lineTo(robotSize * 0.8, arrowWidth * 0.3);
      ctx.lineTo(arrowLength - arrowWidth * 0.7, arrowWidth * 0.3);
      ctx.lineTo(arrowLength - arrowWidth * 0.7, arrowWidth);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      
      // Particle effect - small glowing dots around robot
      const particleCount = isMapping ? 8 : 6; // More particles when mapping
      for (let i = 0; i < particleCount; i++) {
        const angle = (i / particleCount) * Math.PI * 2 + time * (isMapping ? 2 : 1);
        const distance = robotSize * 2.5 + Math.sin(time * (isMapping ? 3 : 2) + i) * robotSize * 0.5;
        const particleX = Math.cos(angle) * distance;
        const particleY = Math.sin(angle) * distance;
        const particleSize = (2 + Math.sin(time * (isMapping ? 5 : 4) + i * 2) * 1) / viewState.scale;
        
        ctx.save();
        ctx.globalAlpha = isMapping 
          ? 0.4 + Math.sin(time * 5 + i) * 0.3  // More visible and dynamic when mapping
          : 0.3 + Math.sin(time * 3 + i) * 0.2;
        ctx.fillStyle = lightColor;
        ctx.shadowColor = lightColor;
        ctx.shadowBlur = isMapping ? 8 / viewState.scale : 5 / viewState.scale;
        ctx.beginPath();
        ctx.arc(particleX, particleY, particleSize, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
      }
      
      // Add extra scanning effect when mapping
      if (isMapping) {
        ctx.save();
        const scanAngle = time * 2; // Rotating scan line
        ctx.strokeStyle = lightColor;
        ctx.lineWidth = 2 / viewState.scale;
        ctx.globalAlpha = 0.3 + Math.sin(time * 4) * 0.2;
        ctx.setLineDash([5 / viewState.scale, 5 / viewState.scale]);
        
        // Draw scanning beam
        ctx.beginPath();
        ctx.moveTo(0, 0);
        const scanLength = robotSize * 5;
        ctx.lineTo(Math.cos(scanAngle) * scanLength, Math.sin(scanAngle) * scanLength);
        ctx.stroke();
        
        // Draw second scanning beam (opposite direction)
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(Math.cos(scanAngle + Math.PI) * scanLength, Math.sin(scanAngle + Math.PI) * scanLength);
        ctx.stroke();
        
        ctx.restore();
      }
      
      ctx.restore();
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

    // Restore context state
    ctx.restore();
  }, [occupancyGrid, viewState, isMapping, tempHomePose]);

  // Animation loop
  useEffect(() => {
    const animate = () => {
      renderMap();
      animationFrameRef.current = requestAnimationFrame(animate);
    };

    animate();

    return () => {
      if (animationFrameRef.current !== undefined) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [renderMap]);

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

  // Handle ESC key to cancel home setting
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        if (isDraggingHome || isSettingHome) {
          setIsDraggingHome(false);
          setTempHomePose(null);
          if (onHomeSet) {
            onHomeSet({ x: 0, y: 0, theta: 0 }); // Signal cancellation
          }
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isDraggingHome, isSettingHome, onHomeSet]);

  // Mouse handlers for panning
  const handleMouseDown = (e: React.MouseEvent) => {
    e.preventDefault();
    const rect = canvasRef.current?.getBoundingClientRect();
    if (!rect) return;
    
    if (isSettingHome && occupancyGrid) {
      // Start setting home position with drag for orientation
      const worldPos = screenToWorld(e.clientX, e.clientY);
      if (worldPos) {
        const homePose = { x: worldPos.x, y: worldPos.y, theta: 0 };
        setTempHomePose(homePose);
        setIsDraggingHome(true);
      }
    } else {
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
    } else if (viewState.isDragging) {
      setViewState(prev => ({
        ...prev,
        translateX: e.clientX - prev.dragStartX,
        translateY: e.clientY - prev.dragStartY
      }));
    }
  };

  const handleMouseUp = (e: React.MouseEvent) => {
    e.preventDefault();
    
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
        if (onHomeSet) {
          onHomeSet(tempHomePose);
        }
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
    } else {
      setViewState(prev => ({
        ...prev,
        isDragging: false
      }));
    }
  };

  const handleMouseLeave = () => {
    setViewState(prev => ({
      ...prev,
      isDragging: false
    }));
  };

  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    
    const rect = canvasRef.current?.getBoundingClientRect();
    if (!rect) return;
    
    // Get mouse position relative to canvas
    const mouseX = e.clientX - rect.left - rect.width / 2;
    const mouseY = e.clientY - rect.top - rect.height / 2;
    
    const scaleFactor = e.deltaY > 0 ? 0.9 : 1.1;
    const newScale = Math.min(Math.max(viewState.scale * scaleFactor, 0.1), 10);
    
    // Adjust translation to zoom towards mouse position
    const scaleChange = newScale / viewState.scale;
    const newTranslateX = mouseX - (mouseX - viewState.translateX) * scaleChange;
    const newTranslateY = mouseY - (mouseY - viewState.translateY) * scaleChange;
    
    setViewState(prev => ({
      ...prev,
      scale: newScale,
      translateX: newTranslateX,
      translateY: newTranslateY
    }));
  };

  // Control functions
  const handleZoomIn = () => {
    setViewState(prev => ({
      ...prev,
      scale: Math.min(prev.scale * 1.2, 10)
    }));
  };

  const handleZoomOut = () => {
    setViewState(prev => ({
      ...prev,
      scale: Math.max(prev.scale * 0.8, 0.1)
    }));
  };

  const handleReset = () => {
    if (occupancyGrid && containerRef.current) {
      const container = containerRef.current;
      const containerWidth = container.clientWidth;
      const containerHeight = container.clientHeight;

      const mapWidth = occupancyGrid.info.width;
      const mapHeight = occupancyGrid.info.height;

      // Calculate scale so map fits exactly with at least one edge touching the container
      const scaleX = containerWidth / mapWidth;
      const scaleY = containerHeight / mapHeight;
      const scale = Math.min(scaleX, scaleY);

      setViewState({
        scale: scale,
        translateX: 0,
        translateY: 0,
        isDragging: false,
        dragStartX: 0,
        dragStartY: 0
      });
    } else {
      // Fallback to default reset
      setViewState({
        scale: 1,
        translateX: 0,
        translateY: 0,
        isDragging: false,
        dragStartX: 0,
        dragStartY: 0
      });
    }
  };

  const handleCenterRobot = () => {
    if (!robotPose || !occupancyGrid) return;
    
    const resolution = occupancyGrid.info.resolution;
    const origin = occupancyGrid.info.origin;
    const mapWidth = occupancyGrid.info.width;
    const mapHeight = occupancyGrid.info.height;
    
    const robotX = (robotPose.x - origin.position.x) / resolution;
    const robotY = (robotPose.y - origin.position.y) / resolution;
    
    // Convert to canvas coordinates (with Y flip)
    const mapOffsetX = -mapWidth / 2;
    const mapOffsetY = -mapHeight / 2;
    const canvasX = mapOffsetX + robotX;
    const canvasY = mapOffsetY + (mapHeight - robotY);
    
    setViewState(prev => ({
      ...prev,
      translateX: -canvasX * prev.scale,
      translateY: -canvasY * prev.scale
    }));
  };

  return (
    <div className={`relative w-full h-full ${className}`}>
      <div ref={containerRef} className="w-full h-full bg-gray-900 rounded-lg overflow-hidden">
        <canvas
          ref={canvasRef}
          className="w-full h-full"
          style={{ cursor: (isDraggingHome || isSettingHome) ? 'crosshair' : (viewState.isDragging ? 'grabbing' : 'grab') }}
          onMouseDown={handleMouseDown}
          onMouseMove={handleMouseMove}
          onMouseUp={handleMouseUp}
          onMouseLeave={handleMouseLeave}
          onWheel={handleWheel}
        />
        
        {/* Loading overlay */}
        {isLoading && (
          <div className="absolute inset-0 flex items-center justify-center bg-black/50">
            <div className="text-white">Loading map...</div>
          </div>
        )}
        
        {/* Error overlay */}
        {error && (
          <div className="absolute inset-0 flex items-center justify-center bg-black/50">
            <div className="text-red-400 text-center">
              <p>Failed to load map</p>
              <p className="text-sm mt-2">{error}</p>
            </div>
          </div>
        )}
        
        {/* Instructions for Set Home mode */}
        {isSettingHome && (
          <div className="absolute top-4 left-4 bg-green-500/90 text-white px-3 py-2 rounded-lg backdrop-blur-sm">
            <p className="text-sm font-medium">Setting Home Position</p>
            <p className="text-xs mt-1">Click on map to place • Drag to set orientation • ESC to cancel</p>
          </div>
        )}

        {/* Robot Position Display */}
        {robotPose && (
          <div className="absolute bottom-4 left-4 bg-black/70 text-white px-3 py-2 rounded-lg backdrop-blur-sm">
            <p className="text-xs font-mono">
              Position: ({robotPose.x.toFixed(2)}, {robotPose.y.toFixed(2)})
            </p>
          </div>
        )}

        {/* Controls */}
        <div className="absolute top-4 right-4 flex flex-col gap-2">
          <button
            onClick={() => {
              if (onHomeSet) {
                onHomeSet({ x: 0, y: 0, theta: 0 }); // Signal to parent to start home setting
              }
            }}
            className={`p-2 backdrop-blur-sm rounded-lg transition-colors ${
              isSettingHome
                ? 'bg-green-500/80 hover:bg-green-600/80 text-white'
                : 'bg-white/10 hover:bg-white/20 text-white'
            }`}
            title={isSettingHome ? "Cancel set home" : "Set home position"}
          >
            <Home className="w-5 h-5" />
          </button>
          <div className="w-full h-px bg-white/20" />
          <button
            onClick={handleZoomIn}
            className="p-2 bg-white/10 hover:bg-white/20 backdrop-blur-sm rounded-lg transition-colors"
            title="Zoom In"
          >
            <ZoomIn className="w-5 h-5 text-white" />
          </button>
          <button
            onClick={handleZoomOut}
            className="p-2 bg-white/10 hover:bg-white/20 backdrop-blur-sm rounded-lg transition-colors"
            title="Zoom Out"
          >
            <ZoomOut className="w-5 h-5 text-white" />
          </button>
          <button
            onClick={handleReset}
            className="p-2 bg-white/10 hover:bg-white/20 backdrop-blur-sm rounded-lg transition-colors"
            title="Reset View"
          >
            <RotateCcw className="w-5 h-5 text-white" />
          </button>
          <button
            onClick={handleCenterRobot}
            className="p-2 bg-white/10 hover:bg-white/20 backdrop-blur-sm rounded-lg transition-colors"
            title="Center on Robot"
          >
            <Navigation className="w-5 h-5 text-white" />
          </button>
        </div>
        
        {/* Status indicators */}
        <div className="absolute bottom-4 left-4 flex flex-col gap-2">
          {/* Connection status */}
          {connection && (
            <div className="px-3 py-1 bg-green-500/20 backdrop-blur-sm rounded-lg">
              <span className="text-xs text-green-400">Map Connected</span>
            </div>
          )}
          {/* Instructions when setting home */}
          {isSettingHome && (
            <div className="px-3 py-2 bg-blue-500/20 backdrop-blur-sm rounded-lg">
              <p className="text-xs text-blue-300">
                Click to set home position • Drag to set orientation • ESC to cancel
              </p>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}