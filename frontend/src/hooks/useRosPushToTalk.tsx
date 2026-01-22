import { useEffect, useRef, useState, useCallback } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

/**
 * Custom hook for push-to-talk microphone streaming to ROS2
 * Captures audio from device microphone and publishes to topic while button is held
 * Uses audio_common_msgs/AudioData message type (uint8[] data field)
 * Compatible with ROS2 Humble
 */
export default function useRosPushToTalk(
  topicName: string = '/audio_streaming'
) {
  const { connection } = useRobotConnection();
  const [isTransmitting, setIsTransmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [hasPermission, setHasPermission] = useState<boolean | null>(null);
  const [isInitialized, setIsInitialized] = useState(false);

  // Refs for audio processing
  const streamRef = useRef<MediaStream | null>(null);
  const audioContextRef = useRef<AudioContext | null>(null);
  const processorRef = useRef<ScriptProcessorNode | null>(null);
  const topicRef = useRef<ROSLIB.Topic<unknown> | null>(null);
  const sourceRef = useRef<MediaStreamAudioSourceNode | null>(null);
  const isTransmittingRef = useRef<boolean>(false); // For use in audio callback

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
      console.log('[PushToTalk] Requesting microphone permission...');

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
      console.log('[PushToTalk] Permission granted');
      return true;

    } catch (err: any) {
      console.error('[PushToTalk] Permission denied or error:', err);

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
   * Initialize audio context, processor, and ROS topic
   * This is done once when permission is granted, not on every press
   */
  const initialize = useCallback(async () => {
    if (isInitialized) return true;

    if (!connection.ros || !connection.online) {
      setError('Robot is not connected');
      return false;
    }

    setError(null);
    console.log('[PushToTalk] Initializing audio system...');

    try {
      // Request microphone permission if not already granted
      if (!streamRef.current) {
        const hasPermission = await requestMicrophonePermission();
        if (!hasPermission || !streamRef.current) {
          return false;
        }
      }

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
      sourceRef.current = audioContextRef.current.createMediaStreamSource(streamRef.current);

      // Create script processor for audio processing
      processorRef.current = audioContextRef.current.createScriptProcessor(
        BUFFER_SIZE,
        1, // Input channels
        1  // Output channels
      );

      // Connect audio nodes
      sourceRef.current.connect(processorRef.current);
      processorRef.current.connect(audioContextRef.current.destination);

      // Create ROS topic publisher
      topicRef.current = new ROSLIB.Topic({
        ros: connection.ros,
        name: topicName,
        messageType: 'audio_common_msgs/AudioData',
        compression: 'none',
        throttle_rate: 0,
        queue_size: 10
      });

      // Advertise the topic
      topicRef.current.advertise();

      // Set up audio processing callback
      let messageCount = 0;
      let lastLogTime = Date.now();

      processorRef.current.onaudioprocess = (audioEvent: AudioProcessingEvent) => {
        // Only process and send audio when transmitting
        if (!isTransmittingRef.current) return;

        // Get input audio data
        const inputData = audioEvent.inputBuffer.getChannelData(0);

        // Convert to Uint8Array
        const uint8Data = float32ToUint8(inputData);

        // Create and publish ROS message
        const message = {
          data: Array.from(uint8Data)
        };

        if (topicRef.current && connection.online) {
          topicRef.current.publish(message);
          messageCount++;

          // Log periodically
          const now = Date.now();
          if (now - lastLogTime > 5000) { // Log every 5 seconds
            console.log(`[PushToTalk] Sent ${messageCount} audio chunks while transmitting`);
            lastLogTime = now;
            messageCount = 0;
          }
        }
      };

      setIsInitialized(true);
      console.log('[PushToTalk] Audio system initialized and ready');
      return true;

    } catch (err: any) {
      console.error('[PushToTalk] Failed to initialize:', err);
      setError(`Failed to initialize: ${err.message}`);
      setIsInitialized(false);
      return false;
    }
  }, [connection.ros, connection.online, isInitialized, topicName, requestMicrophonePermission, float32ToUint8]);

  /**
   * Start transmitting audio (when button is pressed)
   */
  const startTransmitting = useCallback(async () => {
    // Initialize if not already done
    const initialized = await initialize();
    if (!initialized) {
      console.error('[PushToTalk] Cannot start transmitting - initialization failed');
      return;
    }

    if (isTransmitting) {
      console.log('[PushToTalk] Already transmitting');
      return;
    }

    console.log('[PushToTalk] Starting transmission...');
    isTransmittingRef.current = true;
    setIsTransmitting(true);
  }, [initialize, isTransmitting]);

  /**
   * Stop transmitting audio (when button is released)
   */
  const stopTransmitting = useCallback(() => {
    if (!isTransmitting) {
      return;
    }

    console.log('[PushToTalk] Stopping transmission...');
    isTransmittingRef.current = false;
    setIsTransmitting(false);
  }, [isTransmitting]);

  /**
   * Cleanup function - only called on unmount or when connection is lost
   */
  const cleanup = useCallback(() => {
    console.log('[PushToTalk] Cleaning up audio resources...');

    // Stop transmitting
    isTransmittingRef.current = false;
    setIsTransmitting(false);

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
        console.log('[PushToTalk] AudioContext closed');
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
        console.log('[PushToTalk] Stopped track:', track.kind, track.label);
      });
      streamRef.current = null;
      setHasPermission(null);
    }

    setIsInitialized(false);
  }, []);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      cleanup();
    };
  }, [cleanup]);

  // Cleanup if connection is lost
  useEffect(() => {
    if (!connection.online && isInitialized) {
      console.log('[PushToTalk] Connection lost, cleaning up...');
      cleanup();
    }
  }, [connection.online, isInitialized, cleanup]);

  return {
    isTransmitting,
    hasPermission,
    isInitialized,
    error,
    startTransmitting,
    stopTransmitting,
    initialize,
  };
}