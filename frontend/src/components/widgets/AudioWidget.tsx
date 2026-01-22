'use client';

import { Widget } from './Widget';
import { useState, useRef, useEffect, useCallback } from 'react';
import { Volume2, VolumeX } from 'lucide-react';
import useRosAudioStream from '@/hooks/useRosAudioStream';
import { useDashboard } from '@/contexts/DashboardContext';
import { useLanguage } from '@/contexts/LanguageContext';
import { auditLogger } from '@/utils/audit-logger';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';

// Available audio topics
const AUDIO_TOPICS = [
  { label: 'Channel 1 (Main)', value: '/audio/channel1' },
  { label: 'Channel 2', value: '/audio/channel2' },
  { label: 'Channel 3', value: '/audio/channel3' },
  { label: 'Channel 4', value: '/audio/channel4' },
  { label: 'Channel 5', value: '/audio/channel5' },
  { label: 'Raw Audio', value: '/audio/raw' },
];

interface AudioWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  title?: string;
  props?: {
    title?: string;
    topic?: string;
  };
}

export function AudioWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 350, height: 250 },
  title = 'Audio Stream',
  props,
}: AudioWidgetProps) {
  const { updateWidgetProps } = useDashboard();
  const { t } = useLanguage();
  const { connection } = useRobotConnection();
  
  // Widget state
  const [currentTitle, setCurrentTitle] = useState(props?.title || title);
  const [currentTopic, setCurrentTopic] = useState(props?.topic || '/audio/channel1');
  const [showSettings, setShowSettings] = useState(false);
  
  // Audio stream hook
  const { isPlaying, error, toggleStreaming } = useRosAudioStream(currentTopic, false);
  
  // Canvas and animation refs
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationRef = useRef<number | null>(null);
  const analyserRef = useRef<AnalyserNode | null>(null);
  const audioContextRef = useRef<AudioContext | null>(null);
  const dataArrayRef = useRef<Uint8Array | null>(null);
  
  // Update dashboard context when settings change
  const updateSettings = (updates: any) => {
    const newProps = {
      ...props,
      ...updates,
    };
    updateWidgetProps(id, newProps);
    return newProps;
  };

  const handleTitleChange = (newTitle: string) => {
    setCurrentTitle(newTitle);
    updateSettings({ title: newTitle });
  };

  const handleTopicChange = (newTopic: string) => {
    // Stop current stream if playing
    if (isPlaying) {
      toggleStreaming();
    }
    setCurrentTopic(newTopic);
    updateSettings({ topic: newTopic });
  };

  // Handle audio toggle with logging
  const handleAudioToggle = useCallback(() => {
    if (isPlaying) {
      // Log audio streaming stop
      auditLogger.log({
        event_type: 'system',
        event_action: 'settings_updated',
        event_details: {
          action: 'audio_streaming_stopped',
          topic: currentTopic,
          widget_id: id,
          robot_id: connection.connectedRobot?.id,
          robot_name: connection.connectedRobot?.name
        }
      });
    } else {
      // Log audio streaming start
      auditLogger.log({
        event_type: 'system',
        event_action: 'settings_updated',
        event_details: {
          action: 'audio_streaming_started',
          topic: currentTopic,
          widget_id: id,
          robot_id: connection.connectedRobot?.id,
          robot_name: connection.connectedRobot?.name
        }
      });
    }
    
    toggleStreaming();
  }, [isPlaying, toggleStreaming, currentTopic, id, connection.connectedRobot]);

  // Initialize audio visualization
  useEffect(() => {
    if (!audioContextRef.current) {
      try {
        audioContextRef.current = new (window.AudioContext || (window as any).webkitAudioContext)();
        analyserRef.current = audioContextRef.current.createAnalyser();
        analyserRef.current.fftSize = 2048;
        analyserRef.current.smoothingTimeConstant = 0.8;
        
        const bufferLength = analyserRef.current.frequencyBinCount;
        dataArrayRef.current = new Uint8Array(bufferLength);
        
        // Connect to the audio context destination for visualization
        // Note: The actual audio playback is handled by useRosAudioStream
        // This is just for visualization
      } catch (err) {
        console.error('Failed to initialize audio visualization:', err);
      }
    }
  }, []);

  // Draw waveform
  const drawWaveform = useCallback(() => {
    if (!canvasRef.current || !analyserRef.current || !dataArrayRef.current) return;
    
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    
    // Set canvas size if needed
    if (canvas.width !== canvas.offsetWidth || canvas.height !== canvas.offsetHeight) {
      canvas.width = canvas.offsetWidth;
      canvas.height = canvas.offsetHeight;
    }
    
    const width = canvas.width;
    const height = canvas.height;
    
    // Clear canvas with transparent background
    ctx.clearRect(0, 0, width, height);
    
    if (isPlaying) {
      // Generate simulated waveform data for visualization
      // In a real implementation, you'd get this from the actual audio stream
      const bufferLength = 64; // Further reduced for cleaner, simpler visualization
      const dataArray = new Float32Array(bufferLength);
      
      // Simulate audio data with realistic speech patterns
      const time = Date.now() / 1000;
      const speechActivity = Math.sin(time * 0.5) > 0 ? 1 : 0.2; // Simulate speech bursts
      
      for (let i = 0; i < bufferLength; i++) {
        // Create a more speech-like waveform
        const t = (i / bufferLength) * 2 * Math.PI + time * 3;
        
        // Base wave with speech activity
        const baseWave = Math.sin(t) * 0.2 * speechActivity;
        
        // Occasional peaks to simulate consonants/emphasis
        const peakChance = speechActivity > 0.5 ? 0.08 : 0.02;
        const spike = Math.random() < peakChance ? (Math.random() * 0.5 + 0.3) * speechActivity : 0;
        
        // Very minimal noise
        const noise = (Math.random() - 0.5) * 0.05 * speechActivity;
        
        dataArray[i] = baseWave + spike + noise;
      }
      
      // Apply strong smoothing for cleaner visualization
      const smoothedData = new Float32Array(bufferLength);
      const smoothingFactor = 2;
      for (let i = 0; i < bufferLength; i++) {
        let sum = 0;
        let count = 0;
        for (let j = Math.max(0, i - smoothingFactor); j <= Math.min(bufferLength - 1, i + smoothingFactor); j++) {
          sum += dataArray[j];
          count++;
        }
        smoothedData[i] = sum / count;
      }
      
      // Draw the waveform as a filled area for cleaner look
      ctx.fillStyle = 'rgba(177, 128, 215, 0.3)';
      ctx.strokeStyle = 'rgba(177, 128, 215, 0.8)';
      ctx.lineWidth = 2;
      
      // Draw upper waveform
      ctx.beginPath();
      ctx.moveTo(0, height / 2);
      
      const sliceWidth = width / bufferLength;
      
      for (let i = 0; i < bufferLength; i++) {
        const x = i * sliceWidth;
        const v = Math.abs(smoothedData[i]); // Use absolute value for symmetry
        const y = (height / 2) - (v * height * 0.35); // Scale to 35% of height
        
        if (i === 0) {
          ctx.lineTo(x, y);
        } else {
          // Smooth curve between points
          const prevX = (i - 1) * sliceWidth;
          const midX = (prevX + x) / 2;
          const prevY = (height / 2) - (Math.abs(smoothedData[i - 1]) * height * 0.35);
          const midY = (prevY + y) / 2;
          ctx.quadraticCurveTo(prevX, prevY, midX, midY);
        }
      }
      
      ctx.lineTo(width, height / 2);
      
      // Draw lower waveform (mirror)
      for (let i = bufferLength - 1; i >= 0; i--) {
        const x = i * sliceWidth;
        const v = Math.abs(smoothedData[i]);
        const y = (height / 2) + (v * height * 0.35);
        
        if (i === bufferLength - 1) {
          ctx.lineTo(x, y);
        } else {
          const nextX = (i + 1) * sliceWidth;
          const midX = (nextX + x) / 2;
          const nextY = (height / 2) + (Math.abs(smoothedData[i + 1]) * height * 0.35);
          const midY = (nextY + y) / 2;
          ctx.quadraticCurveTo(nextX, nextY, midX, midY);
        }
      }
      
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      
    } else {
      // Draw center line when not playing
      ctx.strokeStyle = 'rgba(177, 128, 215, 0.3)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, height / 2);
      ctx.lineTo(width, height / 2);
      ctx.stroke();
    }
    
    animationRef.current = requestAnimationFrame(drawWaveform);
  }, [isPlaying]);

  // Start/stop animation
  useEffect(() => {
    if (isPlaying) {
      drawWaveform();
    } else {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
        animationRef.current = null;
      }
      // Draw one more frame to show the idle state
      drawWaveform();
    }
    
    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, [isPlaying, drawWaveform]);

  return (
    <Widget
      id={id}
      title={currentTitle}
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      initialPosition={initialPosition}
      initialSize={initialSize}
      minWidth={300}
      minHeight={200}
      onSettingsClick={() => setShowSettings(!showSettings)}
    >
      <div className="h-full flex flex-col">
        {showSettings && (
          <div className="w-full bg-gray-100 dark:bg-botbot-darker rounded-md p-3 mb-4 text-sm">
            <div className="mb-3">
              <label className="block text-gray-700 dark:text-gray-300 mb-1">
                {t('myUI', 'widgetName')}
              </label>
              <input
                type="text"
                value={currentTitle}
                onChange={(e) => handleTitleChange(e.target.value)}
                className="w-full p-2 border rounded-md dark:bg-botbot-dark dark:border-botbot-darker"
                placeholder="Enter widget name"
              />
            </div>

            <div className="mb-3">
              <label className="block text-gray-700 dark:text-gray-300 mb-1">
                {t('myUI', 'audioTopic')}
              </label>
              <div className="relative">
                <select
                  value={currentTopic}
                  onChange={(e) => handleTopicChange(e.target.value)}
                  className="w-full p-2 border rounded-md dark:bg-botbot-dark dark:border-botbot-darker appearance-none text-sm"
                >
                  {AUDIO_TOPICS.map((topic) => (
                    <option key={topic.value} value={topic.value}>
                      {topic.label}
                    </option>
                  ))}
                </select>
                <div className="pointer-events-none absolute inset-y-0 right-0 flex items-center px-2 text-gray-700 dark:text-gray-300">
                  <svg
                    className="fill-current h-4 w-4"
                    xmlns="http://www.w3.org/2000/svg"
                    viewBox="0 0 20 20"
                  >
                    <path d="M9.293 12.95l.707.707L15.657 8l-1.414-1.414L10 10.828 5.757 6.586 4.343 8z" />
                  </svg>
                </div>
              </div>
            </div>
          </div>
        )}

        <div className="flex-1 flex flex-col items-center justify-center gap-4">
          {/* Audio control button */}
          <button
            className={`rounded-lg p-4 border-2 transition-all focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple ${
              isPlaying
                ? "bg-green-600 text-white border-green-700 hover:bg-green-700"
                : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700 hover:bg-gray-100 dark:hover:bg-botbot-darker"
            }`}
            onClick={handleAudioToggle}
            aria-label={isPlaying ? "Stop audio streaming" : "Start audio streaming"}
            title={isPlaying ? "Stop audio streaming" : "Start audio streaming"}
          >
            {isPlaying ? (
              <Volume2 className="w-8 h-8 animate-pulse" />
            ) : (
              <VolumeX className="w-8 h-8" />
            )}
          </button>

          {/* Waveform visualizer */}
          <div className="w-full flex-1 bg-gray-100/50 dark:bg-gray-800/30 rounded-lg overflow-hidden border border-gray-200/50 dark:border-gray-700/50">
            <canvas
              ref={canvasRef}
              className="w-full h-full"
              style={{ minHeight: '80px' }}
            />
          </div>

          {/* Status/Error display */}
          {error && (
            <div className="text-xs text-red-500 dark:text-red-400 text-center">
              {error}
            </div>
          )}
          
          <div className="text-xs text-gray-500 dark:text-gray-400 text-center">
            {isPlaying ? 'Streaming' : 'Stopped'} • {currentTopic}
          </div>
        </div>
      </div>
    </Widget>
  );
} 