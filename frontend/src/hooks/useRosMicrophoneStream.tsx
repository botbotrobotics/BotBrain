import { useEffect, useRef, useState, useCallback } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

/**
 * Custom hook for streaming microphone audio to ROS2
 * Captures audio from device microphone and publishes to /audio_streaming topic
 * Uses audio_common_msgs/AudioData message type (uint8[] data field)
 * Compatible with ROS2 Humble
 */
export default function useRosMicrophoneStream(
  topicName: string = '/audio_streaming'
) {
  const { connection } = useRobotConnection();
  const [isStreaming, setIsStreaming] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [hasPermission, setHasPermission] = useState<boolean | null>(null);

  // Refs for audio processing
  const streamRef = useRef<MediaStream | null>(null);
  const audioContextRef = useRef<AudioContext | null>(null);
  const processorRef = useRef<ScriptProcessorNode | null>(null);
  const topicRef = useRef<ROSLIB.Topic<unknown> | null>(null);
  const sourceRef = useRef<MediaStreamAudioSourceNode | null>(null);

  // Audio parameters
  const SAMPLE_RATE = 16000; // 16kHz sample rate for speech
  const BUFFER_SIZE = 2048; // Buffer size for script processor
  const BYTES_PER_SAMPLE = 2; // 16-bit samples

  /**
   * Convert Float32Array audio samples to Uint8Array (16-bit PCM)
   */
  const float32ToUint8 = useCallback((floatArray: Float32Array): Uint8Array => {
    const int16Array = new Int16Array(floatArray.length);

    // Convert float32 (-1 to 1) to int16 (-32768 to 32767)
    for (let i = 0; i < floatArray.length; i++) {
      const sample = Math.max(-1, Math.min(1, floatArray[i]));
      int16Array[i] = sample < 0 ? sample * 32768 : sample * 32767;
    }

    // Convert Int16Array to Uint8Array (little-endian)
    const uint8Array = new Uint8Array(int16Array.length * BYTES_PER_SAMPLE);
    for (let i = 0; i < int16Array.length; i++) {
      const value = int16Array[i];
      const byteIndex = i * BYTES_PER_SAMPLE;
      // Little-endian: low byte first, high byte second
      uint8Array[byteIndex] = value & 0xff;
      uint8Array[byteIndex + 1] = (value >> 8) & 0xff;
    }

    return uint8Array;
  }, []);

  /**
   * Request microphone permission
   */
  const requestMicrophonePermission = useCallback(async (): Promise<boolean> => {
    try {
      console.log('[Microphone] Requesting microphone permission...');

      // Check if getUserMedia is supported
      if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        setError('Microphone access is not supported in this browser');
        return false;
      }

      // Request microphone access
      const stream = await navigator.mediaDevices.getUserMedia({
        audio: {
          echoCancellation: true,
          noiseSuppression: true,
          autoGainControl: true,
          sampleRate: SAMPLE_RATE
        },
        video: false
      });

      // Store the stream
      streamRef.current = stream;
      setHasPermission(true);
      console.log('[Microphone] Permission granted');
      return true;

    } catch (err: any) {
      console.error('[Microphone] Permission denied or error:', err);

      if (err.name === 'NotAllowedError') {
        setError('Microphone permission was denied. Please allow microphone access and try again.');
      } else if (err.name === 'NotFoundError') {
        setError('No microphone found. Please connect a microphone and try again.');
      } else {
        setError(`Failed to access microphone: ${err.message}`);
      }

      setHasPermission(false);
      return false;
    }
  }, []);

  /**
   * Initialize audio context and processor
   */
  const initAudioProcessing = useCallback(async (stream: MediaStream) => {
    try {
      console.log('[Microphone] Initializing audio processing...');

      // Create audio context
      audioContextRef.current = new (window.AudioContext || (window as any).webkitAudioContext)({
        sampleRate: SAMPLE_RATE,
        latencyHint: 'interactive'
      });

      // Resume context if suspended
      if (audioContextRef.current.state === 'suspended') {
        await audioContextRef.current.resume();
      }

      // Create source from stream
      sourceRef.current = audioContextRef.current.createMediaStreamSource(stream);

      // Create script processor for audio processing
      // Note: ScriptProcessorNode is deprecated but still widely supported
      // AudioWorklet is the modern replacement but requires more setup
      processorRef.current = audioContextRef.current.createScriptProcessor(
        BUFFER_SIZE,
        1, // Input channels
        1  // Output channels
      );

      // Connect audio nodes
      sourceRef.current.connect(processorRef.current);
      processorRef.current.connect(audioContextRef.current.destination);

      console.log('[Microphone] Audio processing initialized');
      return true;

    } catch (err: any) {
      console.error('[Microphone] Failed to initialize audio processing:', err);
      setError(`Failed to initialize audio processing: ${err.message}`);
      return false;
    }
  }, []);

  /**
   * Start streaming microphone audio to ROS
   */
  const startStreaming = useCallback(async () => {
    if (!connection.ros || !connection.online) {
      setError('Robot is not connected');
      return;
    }

    if (isStreaming) {
      console.log('[Microphone] Already streaming');
      return;
    }

    setError(null);
    console.log('[Microphone] Starting microphone stream to topic:', topicName);

    try {
      // Request microphone permission if not already granted
      if (!streamRef.current) {
        const hasPermission = await requestMicrophonePermission();
        if (!hasPermission || !streamRef.current) {
          return;
        }
      }

      // Initialize audio processing
      const initialized = await initAudioProcessing(streamRef.current);
      if (!initialized || !processorRef.current) {
        return;
      }

      // Create ROS topic publisher
      // Using audio_common_msgs/AudioData for audio streaming
      topicRef.current = new ROSLIB.Topic({
        ros: connection.ros,
        name: topicName,
        messageType: 'audio_common_msgs/AudioData', // Use AudioData message type
        compression: 'none',
        throttle_rate: 0,
        queue_size: 10
      });

      let messageCount = 0;
      let lastLogTime = Date.now();

      // Set up audio processing callback
      processorRef.current.onaudioprocess = (audioEvent: AudioProcessingEvent) => {
        // Get input audio data
        const inputData = audioEvent.inputBuffer.getChannelData(0);

        // Convert to Uint8Array
        const uint8Data = float32ToUint8(inputData);

        // Create ROS message with AudioData structure (simple uint8[] data field)
        const message = {
          data: Array.from(uint8Data) // Convert Uint8Array to regular array for ROS
        };

        // Publish to ROS topic
        if (topicRef.current && connection.online) {
          topicRef.current.publish(message);
          messageCount++;

          // Log periodically
          const now = Date.now();
          if (now - lastLogTime > 5000) { // Log every 5 seconds
            console.log(`[Microphone] Sent ${messageCount} audio chunks, last size: ${uint8Data.length} bytes`);
            lastLogTime = now;
            messageCount = 0;
          }
        }
      };

      // Advertise the topic
      topicRef.current.advertise();

      setIsStreaming(true);
      console.log('[Microphone] Successfully started streaming to:', topicName);

    } catch (err: any) {
      console.error('[Microphone] Failed to start streaming:', err);
      setError(`Failed to start streaming: ${err.message}`);
      setIsStreaming(false);
    }
  }, [connection.ros, connection.online, isStreaming, topicName, requestMicrophonePermission, initAudioProcessing, float32ToUint8]);

  /**
   * Stop streaming microphone audio
   */
  const stopStreaming = useCallback(() => {
    console.log('[Microphone] Stopping microphone stream');

    // Stop audio processing
    if (processorRef.current) {
      processorRef.current.onaudioprocess = null;
      processorRef.current.disconnect();
      processorRef.current = null;
    }

    // Disconnect source
    if (sourceRef.current) {
      sourceRef.current.disconnect();
      sourceRef.current = null;
    }

    // Close audio context
    if (audioContextRef.current && audioContextRef.current.state !== 'closed') {
      audioContextRef.current.close().then(() => {
        console.log('[Microphone] AudioContext closed');
      });
      audioContextRef.current = null;
    }

    // Unadvertise and clean up ROS topic
    if (topicRef.current) {
      topicRef.current.unadvertise();
      topicRef.current = null;
    }

    // Stop media stream tracks
    if (streamRef.current) {
      streamRef.current.getTracks().forEach(track => {
        track.stop();
        console.log('[Microphone] Stopped track:', track.kind, track.label);
      });
      streamRef.current = null;
      setHasPermission(null); // Reset permission state
    }

    setIsStreaming(false);
  }, []);

  /**
   * Toggle microphone streaming
   */
  const toggleStreaming = useCallback(async () => {
    if (isStreaming) {
      stopStreaming();
    } else {
      await startStreaming();
    }
  }, [isStreaming, startStreaming, stopStreaming]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (isStreaming) {
        stopStreaming();
      }
    };
  }, [isStreaming, stopStreaming]);

  // Stop streaming if connection is lost
  useEffect(() => {
    if (!connection.online && isStreaming) {
      console.log('[Microphone] Connection lost, stopping stream');
      stopStreaming();
    }
  }, [connection.online, isStreaming, stopStreaming]);

  return {
    isStreaming,
    hasPermission,
    error,
    toggleStreaming,
    startStreaming,
    stopStreaming,
  };
}