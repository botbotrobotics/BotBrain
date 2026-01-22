'use client';

import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useEffect, useRef, useState } from 'react';
import RobotOffline from './robot-offline';
import { Loader, Maximize2, Minimize2 } from 'lucide-react';
import { useLanguage } from '@/contexts/LanguageContext';
import * as ROSLIB from 'roslib';

interface RawImage {
  header: {
    frame_id: string;
    seq: number;
    stamp: {
      nanosec: number;
      sec: number;
    };
  };
  height: number;
  width: number;
  encoding: string;
  is_bigendian: number;
  step: number;
  data: any;
}

export default function RosCameraRaw({
  topicName,
  offlineMsg,
}: {
  topicName: string;
  offlineMsg?: string;
}) {
  const [isFullScreen, setIsFullScreen] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const containerRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { connection } = useRobotConnection();
  const { t } = useLanguage();
  const topicRef = useRef<ROSLIB.Topic<unknown> | null>(null);

  const toggleFullScreen = () => {
    if (!containerRef.current) return;
    
    if (!isFullScreen) {
      if (containerRef.current.requestFullscreen) {
        containerRef.current.requestFullscreen().then(() => {
          setIsFullScreen(true);
        }).catch((err) => {
          console.error('Error attempting to enable full-screen mode:', err);
        });
      }
    } else {
      if (document.exitFullscreen) {
        document.exitFullscreen().then(() => {
          setIsFullScreen(false);
        }).catch((err) => {
          console.error('Error attempting to exit full-screen mode:', err);
        });
      }
    }
  };

  useEffect(() => {
    const handleFullScreenChange = () => {
      if (document.fullscreenElement === null) {
        setIsFullScreen(false);
      }
    };

    document.addEventListener('fullscreenchange', handleFullScreenChange);
    return () => {
      document.removeEventListener('fullscreenchange', handleFullScreenChange);
    };
  }, []);

  const processRawImage = (msg: RawImage) => {
    if (!canvasRef.current) return;
    
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    setIsLoading(false);

    // Set canvas dimensions
    canvas.width = msg.width;
    canvas.height = msg.height;

    // Create ImageData object
    const imageData = ctx.createImageData(msg.width, msg.height);
    
    // Convert ROS image data to canvas format
    let data: Uint8Array;
    
    if (msg.data instanceof Uint8Array) {
      data = msg.data;
    } else if (msg.data instanceof ArrayBuffer) {
      data = new Uint8Array(msg.data);
    } else if (Array.isArray(msg.data)) {
      data = new Uint8Array(msg.data);
    } else if (typeof msg.data === 'string') {
      // Base64 encoded data
      const binaryString = atob(msg.data);
      data = new Uint8Array(binaryString.length);
      for (let i = 0; i < binaryString.length; i++) {
        data[i] = binaryString.charCodeAt(i);
      }
    } else {
      console.error('Unsupported image data format');
      return;
    }

    // Handle different encodings
    if (msg.encoding === 'rgb8') {
      // RGB8 encoding - convert to RGBA
      for (let i = 0, j = 0; i < data.length; i += 3, j += 4) {
        imageData.data[j] = data[i];       // R
        imageData.data[j + 1] = data[i + 1]; // G
        imageData.data[j + 2] = data[i + 2]; // B
        imageData.data[j + 3] = 255;         // A
      }
    } else if (msg.encoding === 'bgr8') {
      // BGR8 encoding - convert to RGBA
      for (let i = 0, j = 0; i < data.length; i += 3, j += 4) {
        imageData.data[j] = data[i + 2];     // R (from B position)
        imageData.data[j + 1] = data[i + 1]; // G
        imageData.data[j + 2] = data[i];     // B (from R position)
        imageData.data[j + 3] = 255;         // A
      }
    } else if (msg.encoding === 'rgba8') {
      // RGBA8 encoding - direct copy
      imageData.data.set(data);
    } else if (msg.encoding === 'bgra8') {
      // BGRA8 encoding - swap R and B
      for (let i = 0; i < data.length; i += 4) {
        imageData.data[i] = data[i + 2];     // R (from B position)
        imageData.data[i + 1] = data[i + 1]; // G
        imageData.data[i + 2] = data[i];     // B (from R position)
        imageData.data[i + 3] = data[i + 3]; // A
      }
    } else if (msg.encoding === 'mono8') {
      // Grayscale - copy to all RGB channels
      for (let i = 0, j = 0; i < data.length; i++, j += 4) {
        imageData.data[j] = data[i];     // R
        imageData.data[j + 1] = data[i]; // G
        imageData.data[j + 2] = data[i]; // B
        imageData.data[j + 3] = 255;     // A
      }
    } else {
      console.warn(`Unsupported encoding: ${msg.encoding}, attempting RGB8`);
      // Attempt RGB8 as fallback
      for (let i = 0, j = 0; i < data.length && j < imageData.data.length; i += 3, j += 4) {
        imageData.data[j] = data[i];       // R
        imageData.data[j + 1] = data[i + 1]; // G
        imageData.data[j + 2] = data[i + 2]; // B
        imageData.data[j + 3] = 255;         // A
      }
    }

    // Draw the image data to canvas
    ctx.putImageData(imageData, 0, 0);
  };

  useEffect(() => {
    if (!connection.ros || !connection.online) return;

    // Create topic subscription for raw images
    const topic = new ROSLIB.Topic({
      ros: connection.ros,
      name: topicName.startsWith('/') ? topicName : `/${topicName}`,
      messageType: 'sensor_msgs/Image',
      compression: 'cbor',
      throttle_rate: 33, // ~30 FPS
      queue_size: 1,
    });

    topicRef.current = topic;

    topic.subscribe((msg: unknown) => {
      processRawImage(msg as unknown as RawImage);
    });

    return () => {
      if (topicRef.current) {
        topicRef.current.unsubscribe();
        topicRef.current = null;
      }
    };
  }, [connection.ros, connection.online, topicName]);

  return (
    <div 
      className={`w-full h-full relative flex items-center justify-center rounded-lg ${isFullScreen ? 'fullscreen-camera' : ''}`}
      ref={containerRef}
    >
      {connection.online ? (
        <div className="relative w-full h-full">
          {isLoading && (
            <div className="absolute inset-0 flex flex-col items-center justify-center text-center z-10 bg-gray-100 dark:bg-gray-900 rounded-lg">
              <Loader className="w-5 h-5 animate-spin mb-2" />
              <span className="text-sm text-gray-600 dark:text-gray-400">
                {t('robotCams', 'initializingCamera')}
              </span>
            </div>
          )}
          <canvas
            className="rounded-lg border-2 border-gray-200 dark:border-black w-full h-full"
            style={{ 
              objectFit: 'contain',
            }}
            ref={canvasRef}
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
      ) : (
        <RobotOffline msg={offlineMsg} useBorder />
      )}
    </div>
  );
}