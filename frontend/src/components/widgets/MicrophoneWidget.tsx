'use client';

import { Widget } from './Widget';
import { useState, useRef, useEffect, useCallback } from 'react';
import { Mic, MicOff, Radio, Settings } from 'lucide-react';
import useRosMicrophoneStream from '@/hooks/useRosMicrophoneStream';
import useRosPushToTalk from '@/hooks/useRosPushToTalk';
import { useDashboard } from '@/contexts/DashboardContext';
import { useLanguage } from '@/contexts/LanguageContext';
import { auditLogger } from '@/utils/audit-logger';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';

interface MicrophoneWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  title?: string;
  props?: {
    title?: string;
    mode?: 'toggle' | 'push-to-talk' | 'both';
  };
}

export function MicrophoneWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 320, height: 280 },
  title = 'Microphone',
  props,
}: MicrophoneWidgetProps) {
  const { updateWidgetProps } = useDashboard();
  const { t } = useLanguage();
  const { connection } = useRobotConnection();

  // Widget state
  const [currentTitle, setCurrentTitle] = useState(props?.title || title);
  const [mode, setMode] = useState<'toggle' | 'push-to-talk' | 'both'>(props?.mode || 'both');
  const [showSettings, setShowSettings] = useState(false);

  // Microphone streaming (toggle mode)
  const {
    isStreaming: isMicrophoneOn,
    error: microphoneError,
    hasPermission: togglePermission,
    toggleStreaming: toggleMicrophone
  } = useRosMicrophoneStream('/audio_streaming');

  // Push-to-talk
  const {
    isTransmitting: isPushToTalkActive,
    error: pushToTalkError,
    hasPermission: pushToTalkPermission,
    startTransmitting,
    stopTransmitting
  } = useRosPushToTalk('/audio_streaming');

  // Canvas and animation refs for visualization
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationRef = useRef<number | null>(null);

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

  const handleModeChange = (newMode: 'toggle' | 'push-to-talk' | 'both') => {
    // Stop any active streaming when changing modes
    if (isMicrophoneOn) {
      toggleMicrophone();
    }
    if (isPushToTalkActive) {
      stopTransmitting();
    }

    setMode(newMode);
    updateSettings({ mode: newMode });
  };

  // Handle microphone toggle with logging
  const handleMicrophoneToggle = useCallback(async () => {
    auditLogger.log({
      event_type: 'audio',
      event_action: isMicrophoneOn ? 'audio_stopped' : 'audio_played',
      robot_id: connection.connectedRobot?.id,
      robot_name: connection.connectedRobot?.name,
      event_details: {
        source: 'microphone_widget',
        type: 'toggle',
        action: isMicrophoneOn ? 'stopped' : 'started',
        topic: '/audio_streaming',
        widget_id: id
      }
    });

    await toggleMicrophone();
  }, [isMicrophoneOn, toggleMicrophone, connection.connectedRobot, id]);

  // Handle push-to-talk press
  const handlePushToTalkPress = useCallback(async () => {
    auditLogger.log({
      event_type: 'audio',
      event_action: 'audio_played',
      robot_id: connection.connectedRobot?.id,
      robot_name: connection.connectedRobot?.name,
      event_details: {
        source: 'microphone_widget',
        type: 'push_to_talk',
        action: 'started',
        topic: '/audio_streaming',
        widget_id: id
      }
    });

    await startTransmitting();
  }, [startTransmitting, connection.connectedRobot, id]);

  // Handle push-to-talk release
  const handlePushToTalkRelease = useCallback(() => {
    auditLogger.log({
      event_type: 'audio',
      event_action: 'audio_stopped',
      robot_id: connection.connectedRobot?.id,
      robot_name: connection.connectedRobot?.name,
      event_details: {
        source: 'microphone_widget',
        type: 'push_to_talk',
        action: 'stopped',
        topic: '/audio_streaming',
        widget_id: id
      }
    });

    stopTransmitting();
  }, [stopTransmitting, connection.connectedRobot, id]);

  // Draw waveform visualization
  const drawWaveform = useCallback(() => {
    if (!canvasRef.current) return;

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

    // Clear canvas
    ctx.clearRect(0, 0, width, height);

    const isActive = isMicrophoneOn || isPushToTalkActive;

    if (isActive) {
      // Generate simulated waveform for microphone input
      const bufferLength = 32;
      const dataArray = new Float32Array(bufferLength);
      const time = Date.now() / 1000;

      // Simulate microphone input with varying amplitude
      const amplitude = 0.3 + Math.sin(time * 2) * 0.2;

      for (let i = 0; i < bufferLength; i++) {
        const t = (i / bufferLength) * Math.PI * 2 + time * 5;
        dataArray[i] = Math.sin(t) * amplitude + (Math.random() - 0.5) * 0.1;
      }

      // Draw waveform
      const color = isPushToTalkActive ? 'rgba(251, 146, 60, 0.8)' : 'rgba(59, 130, 246, 0.8)'; // Orange for PTT, Blue for toggle
      const bgColor = isPushToTalkActive ? 'rgba(251, 146, 60, 0.2)' : 'rgba(59, 130, 246, 0.2)';

      ctx.fillStyle = bgColor;
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;

      // Draw as bars for a different visual style
      const barWidth = (width / bufferLength) * 0.8;
      const barSpacing = (width / bufferLength) * 0.2;

      for (let i = 0; i < bufferLength; i++) {
        const barHeight = Math.abs(dataArray[i]) * height * 0.8;
        const x = i * (barWidth + barSpacing);
        const y = (height - barHeight) / 2;

        ctx.fillRect(x, y, barWidth, barHeight);
      }
    } else {
      // Draw idle state - single line
      ctx.strokeStyle = 'rgba(156, 163, 175, 0.3)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, height / 2);
      ctx.lineTo(width, height / 2);
      ctx.stroke();
    }

    animationRef.current = requestAnimationFrame(drawWaveform);
  }, [isMicrophoneOn, isPushToTalkActive]);

  // Start/stop animation
  useEffect(() => {
    if (isMicrophoneOn || isPushToTalkActive) {
      drawWaveform();
    } else {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
        animationRef.current = null;
      }
      drawWaveform(); // Draw one more frame for idle state
    }

    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, [isMicrophoneOn, isPushToTalkActive, drawWaveform]);

  // Get current error message
  const currentError = microphoneError || pushToTalkError;

  return (
    <Widget
      id={id}
      title={currentTitle}
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      initialPosition={initialPosition}
      initialSize={initialSize}
      minWidth={280}
      minHeight={220}
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

            <div>
              <label className="block text-gray-700 dark:text-gray-300 mb-1">
                Microphone Mode
              </label>
              <select
                value={mode}
                onChange={(e) => handleModeChange(e.target.value as 'toggle' | 'push-to-talk' | 'both')}
                className="w-full p-2 border rounded-md dark:bg-botbot-dark dark:border-botbot-darker appearance-none"
              >
                <option value="toggle">Toggle On/Off</option>
                <option value="push-to-talk">Push to Talk</option>
                <option value="both">Both Modes</option>
              </select>
            </div>
          </div>
        )}

        <div className="flex-1 flex flex-col items-center justify-center gap-3">
          {/* Control buttons based on mode */}
          <div className="flex gap-3">
            {(mode === 'toggle' || mode === 'both') && (
              <button
                className={`rounded-lg p-3 border-2 transition-all focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple ${
                  isMicrophoneOn
                    ? "bg-blue-600 text-white border-blue-700 hover:bg-blue-700"
                    : microphoneError
                      ? "bg-red-600 text-white border-red-700"
                      : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700 hover:bg-gray-100 dark:hover:bg-botbot-darker"
                }`}
                onClick={handleMicrophoneToggle}
                disabled={isPushToTalkActive} // Disable if push-to-talk is active
                aria-label={isMicrophoneOn ? "Stop microphone" : "Start microphone"}
                title={
                  microphoneError
                    ? `Error: ${microphoneError}`
                    : isMicrophoneOn
                      ? "Stop microphone streaming"
                      : "Start microphone streaming"
                }
              >
                {isMicrophoneOn ? (
                  <Mic className="w-6 h-6 animate-pulse" />
                ) : (
                  <MicOff className="w-6 h-6" />
                )}
              </button>
            )}

            {(mode === 'push-to-talk' || mode === 'both') && (
              <button
                className={`rounded-lg p-3 border-2 transition-all focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple ${
                  isPushToTalkActive
                    ? "bg-orange-600 text-white border-orange-700"
                    : pushToTalkError
                      ? "bg-red-600 text-white border-red-700"
                      : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700 hover:bg-gray-100 dark:hover:bg-botbot-darker"
                }`}
                onMouseDown={handlePushToTalkPress}
                onMouseUp={handlePushToTalkRelease}
                onMouseLeave={handlePushToTalkRelease}
                onTouchStart={handlePushToTalkPress}
                onTouchEnd={handlePushToTalkRelease}
                onTouchCancel={handlePushToTalkRelease}
                disabled={isMicrophoneOn} // Disable if toggle mode is active
                aria-label="Push to talk"
                title={
                  pushToTalkError
                    ? `Error: ${pushToTalkError}`
                    : isPushToTalkActive
                      ? "Release to stop talking"
                      : "Hold to talk"
                }
              >
                <Radio className={`w-6 h-6 ${isPushToTalkActive ? 'animate-pulse' : ''}`} />
              </button>
            )}
          </div>

          {/* Mode label */}
          <div className="text-xs text-gray-500 dark:text-gray-400 text-center">
            {mode === 'both' ? 'Toggle / Push-to-Talk' : mode === 'toggle' ? 'Toggle Mode' : 'Push-to-Talk Mode'}
          </div>

          {/* Waveform visualizer */}
          <div className="w-full flex-1 bg-gray-100/50 dark:bg-gray-800/30 rounded-lg overflow-hidden border border-gray-200/50 dark:border-gray-700/50">
            <canvas
              ref={canvasRef}
              className="w-full h-full"
              style={{ minHeight: '60px' }}
            />
          </div>

          {/* Status display */}
          {currentError && (
            <div className="text-xs text-red-500 dark:text-red-400 text-center px-2">
              {currentError}
            </div>
          )}

          <div className="text-xs text-gray-500 dark:text-gray-400 text-center">
            {isMicrophoneOn ? (
              <span className="text-blue-600 dark:text-blue-400">● Streaming</span>
            ) : isPushToTalkActive ? (
              <span className="text-orange-600 dark:text-orange-400">● Transmitting</span>
            ) : (
              <span>● Ready</span>
            )}
            <span className="ml-2">• /audio_streaming</span>
          </div>

          {/* Permission status */}
          {(togglePermission === false || pushToTalkPermission === false) && (
            <div className="text-xs text-yellow-600 dark:text-yellow-400 text-center">
              Microphone permission required
            </div>
          )}
        </div>
      </div>
    </Widget>
  );
}