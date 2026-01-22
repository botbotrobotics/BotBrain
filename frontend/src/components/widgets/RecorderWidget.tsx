'use client';

import { useState, useEffect, useCallback, useMemo } from 'react';
import { Widget } from './Widget';
import { Circle, Camera, Check, X, Download, Settings2 } from 'lucide-react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useLanguage } from '@/contexts/LanguageContext';
import useVideoRecorder from '@/hooks/useVideoRecorder';
import { getConfig } from '@/utils/config';
import { auditLogger } from '@/utils/audit-logger';

interface RecorderWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  props?: {
    selectedStreams?: string[];
  };
}

interface StreamInfo {
  id: string;
  name: string;
  topic: string;
  type: 'camera' | 'thermal' | 'rgb';
}

const AVAILABLE_STREAMS: StreamInfo[] = [
  { id: 'main-camera', name: 'Main Camera', topic: '/camera_embedded/image_raw/compressed', type: 'camera' },
  { id: 'thermal-camera', name: 'Thermal Camera', topic: '/camera_sideCam/thermal/image_raw/compressed', type: 'thermal' },
  { id: 'rgb-camera', name: 'RGB Camera', topic: '/camera_sideCam/rgb/image_raw/compressed', type: 'rgb' },
];

export function RecorderWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 320, height: 400 },
  props,
}: RecorderWidgetProps) {
  const [selectedStreams, setSelectedStreams] = useState<string[]>(
    props?.selectedStreams || ['main-camera']
  );
  const [showSettings, setShowSettings] = useState(false);
  const [isProcessing, setIsProcessing] = useState(false);
  const [videoFormat, setVideoFormat] = useState<'mp4' | 'webm'>('mp4');
  const { connection } = useRobotConnection();
  const { t } = useLanguage();
  const { isRecording, startRecording, downloadRecording, error } = useVideoRecorder();

  // Load video format from settings
  useEffect(() => {
    const format = getConfig('videoFormat');
    setVideoFormat(format);
  }, []);

  // Report any recording errors
  useEffect(() => {
    if (error) {
      console.error('Recording error:', error);
    }
  }, [error]);

  // Get canvas element for a stream
  const getCanvasForStream = (streamId: string): HTMLCanvasElement | null => {
    const stream = AVAILABLE_STREAMS.find(s => s.id === streamId);
    if (!stream) return null;

    // Map stream ID to canvas ID used by camera components
    let canvasId = '';
    if (streamId === 'main-camera') {
      canvasId = 'camera-canvas-camera';
    } else if (streamId === 'thermal-camera') {
      canvasId = 'camera-canvas-thermal';
    } else if (streamId === 'rgb-camera') {
      canvasId = 'camera-canvas-rgb';
    }

    const element = document.getElementById(canvasId);
    return element instanceof HTMLCanvasElement ? element : null;
  };

  // Check if any selected stream is recording
  const isAnyRecording = useMemo(() => {
    return selectedStreams.some(streamId => isRecording(streamId));
  }, [selectedStreams, isRecording]);

  // Handle starting recordings
  const handleStartRecording = async () => {
    if (isProcessing || !connection.online) return;

    setIsProcessing(true);

    try {
      for (const streamId of selectedStreams) {
        const canvas = getCanvasForStream(streamId);
        const stream = AVAILABLE_STREAMS.find(s => s.id === streamId);

        if (canvas && stream) {
          console.log(`Starting recording for ${stream.name}`);
          await startRecording(streamId, canvas);

          // Log recording start
          auditLogger.logRecordingStarted(
            stream.type,
            connection.connectedRobot?.id,
            connection.connectedRobot?.name
          );
        } else {
          console.warn(`Canvas not found for stream: ${streamId}`);
        }
      }
    } catch (err) {
      console.error('Error starting recordings:', err);
    } finally {
      setIsProcessing(false);
    }
  };

  // Handle stopping and downloading recordings
  const handleStopRecording = async () => {
    if (isProcessing) return;

    setIsProcessing(true);

    try {
      for (const streamId of selectedStreams) {
        if (isRecording(streamId)) {
          const stream = AVAILABLE_STREAMS.find(s => s.id === streamId);

          console.log(`Downloading recording for ${stream?.name || streamId}`);
          await downloadRecording(streamId, `botbot-${streamId}`);

          // Log recording stop
          if (stream) {
            auditLogger.logRecordingStopped(
              stream.type,
              0, // Duration could be tracked if needed
              connection.connectedRobot?.id,
              connection.connectedRobot?.name
            );
          }
        }
      }
    } catch (err) {
      console.error('Error stopping recordings:', err);
    } finally {
      setIsProcessing(false);
    }
  };

  // Toggle stream selection
  const toggleStreamSelection = (streamId: string) => {
    setSelectedStreams(prev => {
      if (prev.includes(streamId)) {
        return prev.filter(id => id !== streamId);
      } else {
        return [...prev, streamId];
      }
    });
  };

  // Count of active recordings
  const activeRecordingCount = useMemo(() => {
    return selectedStreams.filter(streamId => isRecording(streamId)).length;
  }, [selectedStreams, isRecording]);

  return (
    <Widget
      id={id}
      title="Recorder"
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      initialPosition={initialPosition}
      initialSize={initialSize}
      minWidth={280}
      minHeight={350}
      onSettingsClick={() => setShowSettings(!showSettings)}
    >
      <div className="h-full flex flex-col p-2">
        {showSettings ? (
          /* Settings View */
          <div className="space-y-4">
            <div>
              <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-2">
                Select Streams to Record
              </h3>
              <div className="space-y-2">
                {AVAILABLE_STREAMS.map(stream => (
                  <label
                    key={stream.id}
                    className="flex items-center space-x-2 cursor-pointer hover:bg-gray-50 dark:hover:bg-botbot-darker p-2 rounded"
                  >
                    <input
                      type="checkbox"
                      checked={selectedStreams.includes(stream.id)}
                      onChange={() => toggleStreamSelection(stream.id)}
                      disabled={isAnyRecording}
                      className="w-4 h-4 text-primary dark:text-botbot-accent rounded
                                focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent"
                    />
                    <span className="text-sm text-gray-700 dark:text-gray-300">
                      {stream.name}
                    </span>
                    {isRecording(stream.id) && (
                      <Circle className="w-3 h-3 text-red-600 fill-current animate-pulse ml-auto" />
                    )}
                  </label>
                ))}
              </div>
            </div>

            <div>
              <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-2">
                File Format
              </h3>
              <div className="bg-gray-100 dark:bg-botbot-darker p-2 rounded">
                <p className="text-xs text-gray-600 dark:text-gray-400">
                  Format: <span className="font-medium">{videoFormat.toUpperCase()}</span>
                </p>
                <p className="text-xs text-gray-500 dark:text-gray-500 mt-1">
                  Change in Settings to update
                </p>
              </div>
            </div>
          </div>
        ) : (
          /* Main View */
          <div className="flex-1 flex flex-col">
            {/* Status Display */}
            <div className="flex-1 flex flex-col items-center justify-center">
              {!connection.online ? (
                <div className="text-center">
                  <Camera className="w-16 h-16 text-gray-400 mx-auto mb-2" />
                  <p className="text-gray-500 dark:text-gray-400">Robot Offline</p>
                </div>
              ) : isAnyRecording ? (
                <div className="text-center">
                  <div className="relative">
                    <Circle className="w-20 h-20 text-red-600 animate-pulse" />
                    <div className="absolute inset-0 flex items-center justify-center">
                      <span className="text-2xl font-bold text-red-600">
                        {activeRecordingCount}
                      </span>
                    </div>
                  </div>
                  <p className="text-red-600 font-medium mt-3">Recording</p>
                  <p className="text-sm text-gray-600 dark:text-gray-400 mt-1">
                    {activeRecordingCount} stream{activeRecordingCount !== 1 ? 's' : ''} active
                  </p>
                </div>
              ) : (
                <div className="text-center">
                  <Camera className="w-16 h-16 text-gray-400 mx-auto mb-2" />
                  <p className="text-gray-600 dark:text-gray-400">Ready to Record</p>
                  <p className="text-sm text-gray-500 dark:text-gray-500 mt-1">
                    {selectedStreams.length} stream{selectedStreams.length !== 1 ? 's' : ''} selected
                  </p>
                </div>
              )}

              {error && (
                <div className="mt-4 p-2 bg-red-100 dark:bg-red-900/20 rounded">
                  <p className="text-xs text-red-600 dark:text-red-400">{error}</p>
                </div>
              )}
            </div>

            {/* Stream List */}
            <div className="mb-4">
              <h4 className="text-xs font-medium text-gray-600 dark:text-gray-400 mb-2">
                Selected Streams:
              </h4>
              <div className="space-y-1 max-h-24 overflow-y-auto">
                {selectedStreams.length === 0 ? (
                  <p className="text-xs text-gray-500 dark:text-gray-500 italic">
                    No streams selected
                  </p>
                ) : (
                  selectedStreams.map(streamId => {
                    const stream = AVAILABLE_STREAMS.find(s => s.id === streamId);
                    return stream ? (
                      <div
                        key={streamId}
                        className="flex items-center justify-between text-xs
                                 bg-gray-50 dark:bg-botbot-darker px-2 py-1 rounded"
                      >
                        <span className="text-gray-700 dark:text-gray-300">
                          {stream.name}
                        </span>
                        {isRecording(streamId) && (
                          <Circle className="w-2 h-2 text-red-600 fill-current animate-pulse" />
                        )}
                      </div>
                    ) : null;
                  })
                )}
              </div>
            </div>

            {/* Control Buttons */}
            <div className="flex gap-2">
              <button
                onClick={isAnyRecording ? handleStopRecording : handleStartRecording}
                disabled={isProcessing || !connection.online || selectedStreams.length === 0}
                className={`flex-1 flex items-center justify-center gap-2 px-4 py-2 rounded-lg
                          font-medium transition-all duration-200 ${
                            isProcessing
                              ? 'bg-yellow-500 text-white cursor-wait'
                              : isAnyRecording
                              ? 'bg-red-600 hover:bg-red-700 text-white'
                              : 'bg-gray-200 dark:bg-botbot-darker hover:bg-gray-300 dark:hover:bg-botbot-dark text-gray-700 dark:text-white'
                          } ${
                            (!connection.online || selectedStreams.length === 0) && !isProcessing
                              ? 'opacity-50 cursor-not-allowed'
                              : ''
                          }`}
              >
                {isProcessing ? (
                  <>
                    <div className="w-4 h-4 border-2 border-white border-t-transparent rounded-full animate-spin" />
                    <span>Processing...</span>
                  </>
                ) : isAnyRecording ? (
                  <>
                    <Download className="w-4 h-4" />
                    <span>Stop & Save</span>
                  </>
                ) : (
                  <>
                    <Circle className="w-4 h-4" />
                    <span>Start Recording</span>
                  </>
                )}
              </button>
            </div>
          </div>
        )}
      </div>
    </Widget>
  );
}