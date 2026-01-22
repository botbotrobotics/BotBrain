'use client';

import { useState } from 'react';
import { Widget } from './Widget';
import { Play, Pause, Brain, Eye, EyeOff } from 'lucide-react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import RosCameraImg from '../ros-camera-img';
import { useLanguage } from '@/contexts/LanguageContext';

interface AIStreamWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  props?: {
    showOverlay?: boolean;
  };
}

export function AIStreamWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 400, height: 350 },
  props,
}: AIStreamWidgetProps) {
  const [isPlaying, setIsPlaying] = useState(true);
  const [showOverlay, setShowOverlay] = useState(props?.showOverlay ?? true);
  const { connection } = useRobotConnection();
  const { t } = useLanguage();

  return (
    <Widget
      id={id}
      title="AI Stream"
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      initialPosition={initialPosition}
      initialSize={initialSize}
      minWidth={320}
      minHeight={300}
    >
      <div className="h-full flex flex-col">
        {/* Stream area */}
        <div className="flex-1 bg-black rounded-md overflow-hidden relative">
          {connection.online && isPlaying ? (
            <div className="w-full h-full">
              <RosCameraImg 
                topicName="/yolo/image_compressed" 
                cameraType="yolo"
                offlineMsg="YOLO stream offline"
              />
              {showOverlay && (
                <div className="absolute top-2 left-2 bg-black/70 text-white px-2 py-1 rounded-md text-xs flex items-center gap-1">
                  <Brain className="w-3 h-3" />
                  YOLO Detection
                </div>
              )}
            </div>
          ) : (
            <div className="w-full h-full flex items-center justify-center text-gray-400">
              <div className="flex flex-col items-center justify-center">
                <Brain className="w-16 h-16 mb-2" />
                <p>
                  {!connection.online ? 'Robot offline' : 'Stream paused'}
                </p>
              </div>
            </div>
          )}
        </div>

        {/* Controls */}
        <div className="mt-2 flex justify-center gap-2">
          <button
            onClick={() => setIsPlaying(!isPlaying)}
            className="p-2 rounded-full bg-gray-200 dark:bg-botbot-darker hover:bg-gray-300 dark:hover:bg-botbot-dark transition"
            title={isPlaying ? 'Pause stream' : 'Play stream'}
          >
            {isPlaying ? (
              <Pause className="w-5 h-5 text-gray-700 dark:text-white" />
            ) : (
              <Play className="w-5 h-5 text-gray-700 dark:text-white" />
            )}
          </button>
          <button
            onClick={() => setShowOverlay(!showOverlay)}
            className="p-2 rounded-full bg-gray-200 dark:bg-botbot-darker hover:bg-gray-300 dark:hover:bg-botbot-dark transition"
            title={showOverlay ? 'Hide overlay' : 'Show overlay'}
          >
            {showOverlay ? (
              <EyeOff className="w-5 h-5 text-gray-700 dark:text-white" />
            ) : (
              <Eye className="w-5 h-5 text-gray-700 dark:text-white" />
            )}
          </button>
        </div>
      </div>
    </Widget>
  );
}