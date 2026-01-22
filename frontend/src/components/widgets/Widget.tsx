'use client';

import { useState, useEffect, useRef, useCallback } from 'react';
import { motion, useDragControls } from 'framer-motion';
import { Grip, X, Maximize, Minimize, Settings } from 'lucide-react';
import { useDashboard } from '@/contexts/DashboardContext';
import { cn } from '@/utils/cn';

export interface WidgetProps {
  id: string;
  title: string | React.ReactNode;
  children: React.ReactNode;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  minWidth?: number;
  minHeight?: number;
  onRemove: (id: string) => void;
  onSettingsClick?: () => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  onResize?: (size: { width: number; height: number }) => void;
}

export function Widget({
  id,
  title,
  children,
  initialPosition = { x: 0, y: 0 },
  initialSize = { width: 300, height: 200 },
  minWidth = 200,
  minHeight = 150,
  onRemove,
  onSettingsClick,
  onStartDrag,
  onEndDrag,
  onResize,
}: WidgetProps) {
  // State for position and size
  const [position, setPosition] = useState(initialPosition);
  const [size, setSize] = useState(initialSize);
  const [isDragging, setIsDragging] = useState(false);
  const [isResizing, setIsResizing] = useState(false);
  const [isMaximized, setIsMaximized] = useState(false);
  const [prevSize, setPrevSize] = useState(initialSize);
  const [prevPosition, setPrevPosition] = useState(initialPosition);
  
  // Refs
  const widgetRef = useRef<HTMLDivElement>(null);
  const containerRef = useRef<HTMLDivElement | null>(null);
  
  // Context
  const { updateWidgetPosition, updateWidgetSize } = useDashboard();
  const dragControls = useDragControls();

  // Find container on mount
  useEffect(() => {
    containerRef.current = document.querySelector('[data-dashboard-container]');
  }, []);

  // Update position when dragging ends
  const handleDragEnd = useCallback(() => {
    const container = containerRef.current;
    const widget = widgetRef.current;
    if (!container || !widget) return;

    const containerRect = container.getBoundingClientRect();
    const widgetRect = widget.getBoundingClientRect();

    // Calculate position based on widget's actual position relative to container
    let newX = widgetRect.left - containerRect.left;
    let newY = widgetRect.top - containerRect.top;

    // Clamp within bounds
    newX = Math.max(0, Math.min(newX, containerRect.width - size.width));
    newY = Math.max(0, Math.min(newY, containerRect.height - size.height));

    const newPosition = { x: newX, y: newY };
    setPosition(newPosition);
    updateWidgetPosition(id, newPosition);
    setIsDragging(false);
    onEndDrag?.(id);
  }, [id, size.width, size.height, updateWidgetPosition, onEndDrag]);

  // Handle maximize/minimize
  const handleMaximize = useCallback(() => {
    const container = containerRef.current;
    if (!container) return;

    if (!isMaximized) {
      setPrevSize(size);
      setPrevPosition(position);
      
      const rect = container.getBoundingClientRect();
      const newSize = { width: rect.width - 40, height: rect.height - 40 };
      const newPosition = { x: 20, y: 20 };
      
      setSize(newSize);
      setPosition(newPosition);
      updateWidgetSize(id, newSize);
      updateWidgetPosition(id, newPosition);
      setIsMaximized(true);
    } else {
      setSize(prevSize);
      setPosition(prevPosition);
      updateWidgetSize(id, prevSize);
      updateWidgetPosition(id, prevPosition);
      setIsMaximized(false);
    }
  }, [id, isMaximized, position, size, prevPosition, prevSize, updateWidgetPosition, updateWidgetSize]);

  // Handle resize
  const startResize = useCallback((e: React.PointerEvent) => {
    e.preventDefault();
    e.stopPropagation();
    setIsResizing(true);

    const startX = e.clientX;
    const startY = e.clientY;
    const startWidth = size.width;
    const startHeight = size.height;

    const handleMouseMove = (moveEvent: PointerEvent) => {
      const dx = moveEvent.clientX - startX;
      const dy = moveEvent.clientY - startY;
      
      const container = containerRef.current;
      if (!container) return;
      
      const rect = container.getBoundingClientRect();
      const maxWidth = rect.width - position.x;
      const maxHeight = rect.height - position.y;

      const newWidth = Math.max(minWidth, Math.min(startWidth + dx, maxWidth));
      const newHeight = Math.max(minHeight, Math.min(startHeight + dy, maxHeight));

      setSize({ width: newWidth, height: newHeight });
      onResize?.({ width: newWidth, height: newHeight });
    };

    const handleMouseUp = () => {
      document.removeEventListener('pointermove', handleMouseMove);
      document.removeEventListener('pointerup', handleMouseUp);
      setIsResizing(false);
      // Get the final size from state
      setSize((currentSize) => {
        updateWidgetSize(id, currentSize);
        return currentSize;
      });
    };

    document.addEventListener('pointermove', handleMouseMove);
    document.addEventListener('pointerup', handleMouseUp);
  }, [id, size, position.x, position.y, minWidth, minHeight, updateWidgetSize]);

  // Start drag
  const startDrag = useCallback((event: React.PointerEvent) => {
    if (isMaximized) return;
    event.preventDefault(); // Prevent text selection on touch devices
    onStartDrag?.(id);
    setIsDragging(true);
    dragControls.start(event);
  }, [dragControls, id, isMaximized, onStartDrag]);

  // Get container constraints
  const getConstraints = () => {
    const container = containerRef.current;
    if (!container) return undefined;
    
    const rect = container.getBoundingClientRect();
    return {
      left: 0,
      top: 0,
      right: rect.width - size.width,
      bottom: rect.height - size.height,
    };
  };

  return (
    <motion.div
      ref={widgetRef}
      drag={!isMaximized}
      dragControls={dragControls}
      dragListener={false}
      dragMomentum={false}
      dragElastic={0}
      dragConstraints={getConstraints()}
      onDragEnd={handleDragEnd}
      whileDrag={{ cursor: 'grabbing' }}
      animate={{
        x: position.x,
        y: position.y,
        width: size.width,
        height: size.height,
      }}
      transition={{ 
        type: 'tween', 
        duration: isMaximized ? 0.3 : 0,
        ease: 'easeOut'
      }}
      style={{
        position: 'absolute',
        zIndex: isDragging || isResizing ? 50 : 10,
      }}
      className={cn(
        'bg-white dark:bg-botbot-dark rounded-lg shadow-lg overflow-hidden hover:shadow-xl transition-shadow',
        isDragging && 'opacity-90'
      )}
    >
      {/* Widget header */}
      <div
        className="flex items-center justify-between p-2 bg-gray-100 dark:bg-botbot-darker cursor-grab active:cursor-grabbing select-none"
        onPointerDown={startDrag}
        style={{ touchAction: 'none' }}
      >
        <div className="flex items-center">
          <Grip className="h-4 w-4 mr-2 text-gray-500" />
          <h3 className="text-sm font-medium text-gray-700 dark:text-white">
            {title}
          </h3>
        </div>
        <div className="flex items-center space-x-1">
          {onSettingsClick && (
            <button
              onClick={(e) => {
                e.stopPropagation();
                onSettingsClick();
              }}
              className="text-gray-500 hover:text-gray-700 dark:text-gray-400 dark:hover:text-white p-1"
            >
              <Settings className="h-4 w-4" />
            </button>
          )}
          <button
            onClick={handleMaximize}
            className="text-gray-500 hover:text-gray-700 dark:text-gray-400 dark:hover:text-white p-1"
          >
            {isMaximized ? (
              <Minimize className="h-4 w-4" />
            ) : (
              <Maximize className="h-4 w-4" />
            )}
          </button>
          <button
            onClick={() => onRemove(id)}
            className="text-gray-500 hover:text-red-500 dark:text-gray-400 dark:hover:text-red-400 p-1"
          >
            <X className="h-4 w-4" />
          </button>
        </div>
      </div>

      {/* Widget content */}
      <div className="p-4 h-[calc(100%-2.5rem)] overflow-auto">{children}</div>

      {/* Resize handle */}
      {!isMaximized && (
        <div
          className="absolute bottom-0 right-0 w-4 h-4 cursor-nwse-resize hover:bg-blue-500/20 transition-colors"
          onPointerDown={startResize}
          style={{ touchAction: 'none' }}
        >
          <svg
            className="absolute bottom-0 right-0 w-4 h-4 text-gray-400"
            fill="currentColor"
            viewBox="0 0 16 16"
          >
            <path d="M16 16L8 16L16 8Z" />
          </svg>
        </div>
      )}
    </motion.div>
  );
}