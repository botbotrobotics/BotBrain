'use client';

import { useState, useRef, useEffect } from 'react';
import { Widget } from './Widget';
import { Robot3DViewer } from '../robot-3d-viewer';
import useCustomLaserScan from '@/hooks/ros/useCustomLaserScan';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { LaserScan } from '@/interfaces/ros/LaserScan';
// import { Maximize, Minimize } from 'lucide-react';
import { useLanguage } from '@/contexts/LanguageContext';

// Extend the Window interface to include our custom method
declare global {
  interface Window {
    updateLaserScan?: (laserScan: LaserScan) => void;
    resizeViewer?: (width: number, height: number) => void;
  }
}

interface Visualizer3DWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  modelUrl?: string;
}

export function Visualizer3DWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 500, height: 500 },
  modelUrl: _modelUrl,
}: Visualizer3DWidgetProps) {
  const { connection: _connection } = useRobotConnection();
  const [rosTopic, setRosTopic] = useState<string>('/pointcloud');
  const [isEditing, setIsEditing] = useState(false);
  const [inputValue, setInputValue] = useState(rosTopic);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [containerSize, setContainerSize] = useState({
    width: initialSize.width,
    height: initialSize.height,
  });
  const topicInputRef = useRef<HTMLInputElement>(null);
  const viewerRef = useRef<HTMLDivElement>(null);
  const viewerContainerRef = useRef<HTMLDivElement>(null);
  const widgetContainerRef = useRef<HTMLDivElement>(null);
  const { t } = useLanguage();

  // Get laser scan data from the custom topic
  const laserScan = useCustomLaserScan(rosTopic);

  // Function to update robot-3d-viewer with new laser scan data
  useEffect(() => {
    if (laserScan && laserScan.ranges.length > 0 && window.updateLaserScan) {
      window.updateLaserScan(laserScan);
    }
  }, [laserScan]);

  // Set up resize observer to detect widget size changes
  useEffect(() => {
    const container = widgetContainerRef.current;
    if (!container) return;

    const resizeObserver = new ResizeObserver((entries) => {
      if (entries.length > 0) {
        const { width, height } = entries[0].contentRect;
        setContainerSize({ width, height });

        // Force resize of the 3D viewer if there's a resizeViewer function exposed on window
        if (window.resizeViewer && viewerContainerRef.current) {
          const newHeight = height - 80; // Account for header and footer
          window.resizeViewer(width, newHeight);
        }
      }
    });

    resizeObserver.observe(container);

    return () => {
      resizeObserver.disconnect();
    };
  }, []);

  const handleTopicFocus = () => {
    setIsEditing(true);
  };

  const handleTopicBlur = () => {
    setIsEditing(false);
    if (inputValue.trim() !== rosTopic.trim()) {
      setRosTopic(inputValue.trim());
    }
  };

  const handleTopicChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setInputValue(e.target.value);
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      topicInputRef.current?.blur();
    }
  };

  const toggleFullscreen = () => {
    const container = viewerContainerRef.current;
    if (!container) return;

    if (!isFullscreen) {
      container.classList.add('fullscreen');
    } else {
      container.classList.remove('fullscreen');
    }

    setIsFullscreen(!isFullscreen);
  };

  return (
    <Widget
      id={id}
      title="3D Visualization"
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      initialPosition={initialPosition}
      initialSize={initialSize}
      minWidth={400}
      minHeight={400}
    >
      <div ref={widgetContainerRef} className="h-full flex flex-col">
        <div className="p-2 border-b border-gray-200 dark:border-gray-800">
          <div className="flex items-center">
            <span className="text-sm text-gray-500 dark:text-gray-400 mr-2">
              ROS Topic:
            </span>
            <input
              ref={topicInputRef}
              type="text"
              value={inputValue}
              onChange={handleTopicChange}
              onFocus={handleTopicFocus}
              onBlur={handleTopicBlur}
              onKeyDown={handleKeyDown}
              className="flex-1 text-sm bg-gray-100 dark:bg-gray-800 border border-gray-300 dark:border-gray-700 rounded px-2 py-1 focus:outline-none focus:ring-2 focus:ring-primary"
              placeholder="Enter ROS topic (e.g., /scan)"
            />
          </div>
        </div>

        <div
          ref={viewerRef}
          className="flex-1 relative bg-gray-900 dark:bg-black rounded-md overflow-hidden"
        >
          <div ref={viewerContainerRef} className="w-full h-full">
            <Robot3DViewer canvasInitialHeight={containerSize.height - 80} />
          </div>

          {/* Fullscreen toggle */}
          {/* <button
            onClick={toggleFullscreen}
            className="absolute top-2 right-2 bg-black/50 text-white p-2 rounded-full hover:bg-primary z-10"
          >
            {isFullscreen ? (
              <Minimize className="w-4 h-4" />
            ) : (
              <Maximize className="w-4 h-4" />
            )}
          </button> */}
        </div>

        <div className="mt-2 text-center text-xs text-gray-500 dark:text-gray-400">
          {t('myUI', 'topic')}: {rosTopic} {isEditing ? '(editing...)' : ''}
        </div>
      </div>
    </Widget>
  );
}
