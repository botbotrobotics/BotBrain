import { useEffect, useRef, useState, useCallback } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

interface AudioDataMessage {
  data: number[] | Uint8Array | string; // Can be base64 encoded string
}

/**
 * Custom hook for streaming audio from ROS topics
 * Designed to work with ReSpeaker Mic Array v2.0
 * Default to /audio/channel1 (raw data from mic1)
 * Uses audio_common_msgs/msg/AudioData message type (ROS2)
 */
export default function useRosAudioStream(
  topicName: string = '/audio/channel5',
  useDummy: boolean = false
) {
  const { connection } = useRobotConnection();
  const [isPlaying, setIsPlaying] = useState(false);
  const [error, setError] = useState<string | null>(null);
  
  // Audio context and related refs
  const audioContextRef = useRef<AudioContext | null>(null);
  const topicRef = useRef<ROSLIB.Topic<unknown> | null>(null);
  const nextTimeRef = useRef<number>(0);
  const isProcessingRef = useRef(false);
  const messageTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  
  // Audio processing parameters for ReSpeaker
  const SAMPLE_RATE = 16000; // ReSpeaker default sample rate
  const CHANNELS = 1; // Mono audio
  const BYTES_PER_SAMPLE = 2; // 16-bit samples
  
  /**
   * Initialize audio context
   */
  const initAudioContext = useCallback(async () => {
    if (!audioContextRef.current) {
      try {
        console.log('[Audio] Creating AudioContext with sample rate:', SAMPLE_RATE);
        audioContextRef.current = new (window.AudioContext || (window as any).webkitAudioContext)({
          sampleRate: SAMPLE_RATE,
          latencyHint: 'interactive'
        });
        
        // Resume context if suspended (required for Chrome)
        if (audioContextRef.current.state === 'suspended') {
          await audioContextRef.current.resume();
          console.log('[Audio] AudioContext resumed');
        }
      } catch (err) {
        setError('Failed to create audio context');
        console.error('[Audio] AudioContext error:', err);
      }
    }
  }, []);

  /**
   * List available topics for debugging
   */
  const listAvailableTopics = useCallback(() => {
    if (!connection.ros) return;
    
    console.log('[Audio] Requesting list of available topics...');
    
    connection.ros.getTopics((result: { topics: string[], types: string[] }) => {
      console.log('[Audio] Available topics:', result.topics.length, 'topics found');
      
      // Check if our audio topics exist
      const audioTopics = result.topics.filter(topic => topic.includes('audio'));
      if (audioTopics.length > 0) {
        console.log('[Audio] Found audio topics:', audioTopics);
      } else {
        console.warn('[Audio] No audio topics found!');
      }
      
      // Check for our specific topic
      const targetTopic = `${useDummy ? '/dummy' : ''}${topicName}`;
      if (result.topics.includes(targetTopic)) {
        console.log('[Audio] Target topic exists:', targetTopic);
      } else {
        console.warn('[Audio] Target topic NOT found:', targetTopic);
        console.log('[Audio] Did you mean one of these?', result.topics.filter(t => t.includes('channel')));
      }
    }, (error: any) => {
      console.error('[Audio] Error getting topics:', error);
    });
  }, [connection.ros, topicName, useDummy]);

  /**
   * Check topic info
   */
  const checkTopicInfo = useCallback((fullTopicName: string) => {
    if (!connection.ros) return;
    
    // Get topic type
    connection.ros.getTopicType(fullTopicName, (type: string) => {
      console.log(`[Audio] Topic ${fullTopicName} has type:`, type);
      
      // Also check if anyone is publishing
      if (connection.ros) {
        connection.ros.getTopicsForType('audio_common_msgs/msg/AudioData', (topics: string[]) => {
          console.log('[Audio] Topics publishing audio_common_msgs/msg/AudioData:', topics);
        });
      }
    }, (error: any) => {
      console.error(`[Audio] Error getting topic type for ${fullTopicName}:`, error);
    });
  }, [connection.ros]);

  /**
   * Convert base64 string to Uint8Array
   */
  const base64ToUint8Array = useCallback((base64: string): Uint8Array => {
    // Remove any whitespace or newlines
    const cleanBase64 = base64.replace(/\s/g, '');
    
    try {
      // Decode base64 to binary string
      const binaryString = atob(cleanBase64);
      
      // Convert binary string to Uint8Array
      const bytes = new Uint8Array(binaryString.length);
      for (let i = 0; i < binaryString.length; i++) {
        bytes[i] = binaryString.charCodeAt(i);
      }
      
      return bytes;
    } catch (err) {
      console.error('[Audio] Error decoding base64:', err);
      return new Uint8Array(0);
    }
  }, []);

  /**
   * Convert byte array to Int16 samples then to Float32
   */
  const convertBytesToFloat32 = useCallback((data: Uint8Array | number[]): Float32Array => {
    // Convert to Uint8Array if needed
    const bytes = data instanceof Uint8Array ? data : new Uint8Array(data);
    
    // Calculate number of samples (2 bytes per sample for 16-bit audio)
    const numSamples = Math.floor(bytes.length / BYTES_PER_SAMPLE);
    const floatArray = new Float32Array(numSamples);
    
    // Convert 16-bit little-endian samples to float32 (-1 to 1 range)
    for (let i = 0; i < numSamples; i++) {
      const byteIndex = i * BYTES_PER_SAMPLE;
      // Little-endian: low byte first, high byte second
      let int16Value = (bytes[byteIndex + 1] << 8) | bytes[byteIndex];
      
      // Handle signed 16-bit conversion (two's complement)
      if (int16Value >= 0x8000) {
        int16Value = int16Value - 0x10000;
      }
      
      // Convert signed 16-bit to float (-1 to 1)
      floatArray[i] = int16Value / 32768.0;
    }
    
    return floatArray;
  }, []);

  /**
   * Check which topics are actively publishing
   */
  const checkActivePublishers = useCallback(() => {
    if (!connection.ros) return;
    
    console.log('[Audio] Checking for actively publishing audio topics...');
    
    // List of potential audio topics to check
    const audioTopics = [
      '/audio',
      '/audio/channel0', 
      '/audio/channel1',
      '/audio/channel2',
      '/audio/channel3',
      '/audio/channel4',
      '/audio/channel5',
      '/audioreceiver',
      '/audiosender',
      '/speech_audio'
    ];
    
    audioTopics.forEach(topicName => {
      // First check the topic type
      connection.ros!.getTopicType(topicName, (type: string) => {
        if (!type) return;
        
        console.log(`[Audio] Checking ${topicName} with type: ${type}`);
        
        // Try to subscribe briefly to see if data is coming
        const testTopic = new ROSLIB.Topic({
          ros: connection.ros!,
          name: topicName,
          messageType: type // Use the actual message type
        });
        
        let receivedMessage = false;
        const timeoutId = setTimeout(() => {
          if (!receivedMessage) {
            console.log(`[Audio] ${topicName} - No data received`);
          }
          testTopic.unsubscribe();
        }, 2000);
        
        testTopic.subscribe((message: any) => {
          if (!receivedMessage) {
            receivedMessage = true;
            clearTimeout(timeoutId);
            console.log(`[Audio] ${topicName} - ACTIVE! Message structure:`, {
              hasData: !!message.data,
              dataType: typeof message.data,
              dataLength: message.data?.length,
              messageKeys: Object.keys(message),
              sampleData: message.data?.slice ? message.data.slice(0, 20) : 'no data'
            });
            testTopic.unsubscribe();
          }
        });
      });
    });
  }, [connection.ros]);

  /**
   * Play audio buffer using Web Audio API
   */
  const playAudioChunk = useCallback((audioData: Float32Array) => {
    if (!audioContextRef.current || audioContextRef.current.state === 'closed') {
      console.warn('[Audio] AudioContext not available');
      return;
    }
    
    try {
      // Create audio buffer
      const audioBuffer = audioContextRef.current.createBuffer(
        CHANNELS,
        audioData.length,
        SAMPLE_RATE
      );
      
      // Copy data to buffer
      audioBuffer.getChannelData(0).set(audioData);
      
      // Create buffer source
      const source = audioContextRef.current.createBufferSource();
      source.buffer = audioBuffer;
      source.connect(audioContextRef.current.destination);
      
      // Schedule playback
      const currentTime = audioContextRef.current.currentTime;
      const startTime = Math.max(currentTime, nextTimeRef.current);
      source.start(startTime);
      
      // Update next time for seamless playback
      nextTimeRef.current = startTime + audioBuffer.duration;
      
    } catch (err) {
      console.error('[Audio] Error playing audio chunk:', err);
    }
  }, []);

  /**
   * Start audio streaming
   */
  const startStreaming = useCallback(async () => {
    if (!connection.ros || !connection.online || isPlaying) {
      console.log('[Audio] Cannot start streaming:', { 
        hasRos: !!connection.ros, 
        online: connection.online, 
        isPlaying 
      });
      return;
    }
    
    setError(null);
    console.log('[Audio] Starting audio stream from topic:', topicName, 'dummy mode:', useDummy);
    
    // List available topics for debugging
    listAvailableTopics();
    
    // Check which topics are actively publishing
    checkActivePublishers();
    
    // Initialize audio context
    await initAudioContext();
    
    try {
      // Reset timing
      nextTimeRef.current = 0;
      
      // Create ROS topic subscription with proper naming
      const fullTopicName = `${useDummy ? '/dummy' : ''}${topicName.startsWith('/') ? topicName : `/${topicName}`}`;
      
      // Check topic info
      checkTopicInfo(fullTopicName);
      
      // Try different compression settings
      const compressionOptions = ['none', 'cbor', 'png'];
      console.log(`[Audio] Trying topic ${fullTopicName} with compression: none`);
      
      const topic = new ROSLIB.Topic({
        ros: connection.ros,
        name: fullTopicName,
        messageType: 'audio_common_msgs/msg/AudioData', // ROS2 message type format
        compression: 'none', // Try without compression first
        throttle_rate: 0,
        queue_size: 1,
      });
      
      let messageCount = 0;
      let lastMessageTime = Date.now();
      
      // Set up timeout to check if messages are received
      messageTimeoutRef.current = setTimeout(() => {
        if (messageCount === 0) {
          console.warn('[Audio] No messages received after 5 seconds!');
          console.log('[Audio] Troubleshooting tips:');
          console.log('  1. Check if respeaker_ros node is running on the robot');
          console.log('  2. Run on robot: ros2 node list | grep -i audio');
          console.log('  3. Run on robot: ros2 topic list | grep -i audio');
          console.log('  4. Run on robot: ros2 topic echo', fullTopicName);
          console.log('  5. Check microphone permissions and ALSA configuration');
          
          // Try alternative topic names
          const alternatives = ['/audio/channel0', '/audio/raw', '/audio/channel1', '/audio/channel4', '/audio'];
          console.log('[Audio] You might want to try these topics:', alternatives);
          
          // Check which audio topics have publishers
          console.log('[Audio] Checking for active publishers on audio topics...');
          alternatives.forEach(topicName => {
            connection.ros?.getTopicType(topicName, (type: string) => {
              if (type) {
                console.log(`[Audio] ${topicName} exists with type: ${type}`);
              }
            });
          });
        }
      }, 5000);
      
      // Subscribe to topic
      topic.subscribe((message: any) => {
        messageCount++;
        const now = Date.now();
        const timeSinceLastMessage = now - lastMessageTime;
        lastMessageTime = now;
        
        // Clear timeout since we're receiving messages
        if (messageTimeoutRef.current && messageCount === 1) {
          clearTimeout(messageTimeoutRef.current);
          messageTimeoutRef.current = null;
        }
        
        // Log first few messages for debugging
        if (messageCount <= 5) {
          console.log(`[Audio] Message ${messageCount}:`, {
            dataType: typeof message.data,
            dataLength: message.data?.length,
            isArray: Array.isArray(message.data),
            isUint8Array: message.data instanceof Uint8Array,
            isString: typeof message.data === 'string',
            firstChars: typeof message.data === 'string' ? message.data.substring(0, 50) : undefined,
            firstBytes: message.data?.slice ? message.data.slice(0, 10) : 'no slice method',
            timeSinceLastMessage: messageCount > 1 ? `${timeSinceLastMessage}ms` : 'first message',
            fullMessage: messageCount === 1 ? message : 'see first message for full structure'
          });
        }
        
        try {
          if (message.data && (Array.isArray(message.data) || message.data instanceof Uint8Array)) {
            // Convert byte data to float samples
            const floatData = convertBytesToFloat32(message.data);
            
            // Play the audio chunk
            playAudioChunk(floatData);
            
            // Log periodically
            if (messageCount % 100 === 0) {
              console.log(`[Audio] Processed ${messageCount} messages, last message size: ${message.data.length} bytes`);
            }
          } else if (message.data && typeof message.data === 'string') {
            // Handle base64 encoded audio data
            const audioBytes = base64ToUint8Array(message.data);
            
            if (audioBytes.length > 0) {
              // Convert byte data to float samples
              const floatData = convertBytesToFloat32(audioBytes);
              
              // Play the audio chunk
              playAudioChunk(floatData);
              
              // Log periodically
              if (messageCount % 100 === 0) {
                console.log(`[Audio] Processed ${messageCount} base64 messages, decoded size: ${audioBytes.length} bytes`);
              }
            } else {
              console.warn('[Audio] Failed to decode base64 data');
            }
          } else {
            console.warn('[Audio] Invalid message data:', message);
          }
        } catch (err) {
          console.error('[Audio] Error processing message:', err);
        }
      });
      
      topicRef.current = topic;
      setIsPlaying(true);
      console.log('[Audio] Successfully subscribed to topic:', fullTopicName);
      
      // Note: In roslib 2.x, Topic only emits 'message', 'unsubscribe', 'warning', and 'unadvertise' events
      // 'error' event is handled at the Ros connection level, not Topic level
      topic.on('warning', (warning: unknown) => {
        console.warn('[Audio] Topic warning:', warning);
      });
      
    } catch (err) {
      setError('Failed to start audio streaming');
      console.error('[Audio] Streaming error:', err);
      setIsPlaying(false);
    }
      }, [connection.ros, connection.online, isPlaying, topicName, useDummy, initAudioContext, convertBytesToFloat32, playAudioChunk, listAvailableTopics, checkTopicInfo, checkActivePublishers, base64ToUint8Array]);

  /**
   * Stop audio streaming
   */
  const stopStreaming = useCallback(() => {
    console.log('[Audio] Stopping audio stream');
    
    // Clear any timeouts
    if (messageTimeoutRef.current) {
      clearTimeout(messageTimeoutRef.current);
      messageTimeoutRef.current = null;
    }
    
    // Unsubscribe from topic
    if (topicRef.current) {
      topicRef.current.unsubscribe();
      topicRef.current = null;
    }
    
    // Reset timing
    nextTimeRef.current = 0;
    isProcessingRef.current = false;
    
    // Close audio context
    if (audioContextRef.current && audioContextRef.current.state !== 'closed') {
      audioContextRef.current.close().then(() => {
        console.log('[Audio] AudioContext closed');
      });
      audioContextRef.current = null;
    }
    
    setIsPlaying(false);
  }, []);

  /**
   * Toggle audio streaming
   */
  const toggleStreaming = useCallback(() => {
    if (isPlaying) {
      stopStreaming();
    } else {
      startStreaming();
    }
      }, [isPlaying, startStreaming, stopStreaming]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (isPlaying) {
        stopStreaming();
      }
    };
  }, [isPlaying, stopStreaming]);

  // Stop streaming if connection is lost
  useEffect(() => {
    if (!connection.online && isPlaying) {
      console.log('[Audio] Connection lost, stopping stream');
      stopStreaming();
    }
  }, [connection.online, isPlaying, stopStreaming]);

  return {
    isPlaying,
    error,
    toggleStreaming,
    startStreaming,
    stopStreaming,
  };
} 