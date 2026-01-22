import { useRef, useState, useCallback, useEffect } from 'react';
import { getConfig } from '@/utils/config';

export interface RecordingOptions {
  mimeType?: string;
  videoBitsPerSecond?: number;
}

interface RecorderState {
  mediaRecorder: MediaRecorder;
  stream: MediaStream;
  chunks: Blob[];
  startTime: number;
  mimeType: string;
  dataCollectionInterval: number | null;
  preferredFormat: 'mp4' | 'webm'; // Store the preferred format
}

export default function useVideoRecorder() {
  const [isRecording, setIsRecording] = useState<Record<string, boolean>>({});
  const [error, setError] = useState<string | null>(null);
  const [preferredFormat, setPreferredFormat] = useState<'mp4' | 'webm'>('mp4');
  
  // Use refs instead of state to avoid re-renders and race conditions
  const recordersRef = useRef<Record<string, RecorderState>>({});

  // Load preferred format from settings
  useEffect(() => {
    const savedFormat = getConfig('videoFormat');
    console.log('Loaded video format from settings:', savedFormat);
    setPreferredFormat(savedFormat);
  }, []);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      // Stop all recordings when the component unmounts
      Object.entries(recordersRef.current).forEach(([id, recorder]) => {
        try {
          if (recorder.mediaRecorder.state === 'recording') {
            recorder.mediaRecorder.stop();
          }
          if (recorder.stream) {
            recorder.stream.getTracks().forEach(track => track.stop());
          }
          if (recorder.dataCollectionInterval) {
            clearInterval(recorder.dataCollectionInterval);
          }
        } catch (err) {
          console.error(`Error cleaning up recorder ${id}:`, err);
        }
      });
    };
  }, []);

  // Check if a MIME type is supported
  const isMimeTypeSupported = useCallback((mimeType: string): boolean => {
    try {
      return MediaRecorder.isTypeSupported(mimeType);
    } catch (e) {
      console.warn(`Error checking support for ${mimeType}:`, e);
      return false;
    }
  }, []);

  // Function to determine the best supported mime type
  const getBestMimeType = useCallback(() => {
    // MP4 types to try if mp4 is preferred
    const mp4Types = [
      'video/mp4',
      'video/mp4;codecs=h264',
      'video/mp4;codecs=avc1',
      'video/x-matroska;codecs=avc1', // Alternative that might work for MP4
    ];
    
    // WebM types to try
    const webmTypes = [
      'video/webm;codecs=h264',
      'video/webm;codecs=vp9',
      'video/webm'
    ];

    console.log(`Finding best MIME type for preferred format: ${preferredFormat}`);
    
    // First, try the user's preferred format
    const preferredTypes = preferredFormat === 'mp4' ? mp4Types : webmTypes;
    const alternativeTypes = preferredFormat === 'mp4' ? webmTypes : mp4Types;
    
    // Check all preferred types first
    for (const type of preferredTypes) {
      if (isMimeTypeSupported(type)) {
        console.log(`✅ Using preferred mime type: ${type}`);
        return { mimeType: type, isPreferredFormat: true };
      } else {
        console.log(`❌ MIME type not supported: ${type}`);
      }
    }
    
    // If none of the preferred types work, try alternatives
    for (const type of alternativeTypes) {
      if (isMimeTypeSupported(type)) {
        console.log(`⚠️ Falling back to alternative mime type: ${type}`);
        return { mimeType: type, isPreferredFormat: false };
      }
    }
    
    // Ultimate fallback
    console.log('⚠️ No supported MIME types found, using basic WebM');
    return { mimeType: 'video/webm', isPreferredFormat: preferredFormat === 'webm' };
  }, [preferredFormat, isMimeTypeSupported]);

  const startRecording = useCallback(async (id: string, canvas: HTMLCanvasElement, options: RecordingOptions = {}) => {
    try {
      // Don't start if already recording
      if (isRecording[id]) {
        console.log(`Already recording ${id}, ignoring request`);
        return;
      }
      
      setError(null);
      
      // Get the best supported mime type
      const { mimeType: bestMimeType, isPreferredFormat } = getBestMimeType();
      
      console.log(`Starting recording with MIME type: ${bestMimeType} (preferred format: ${isPreferredFormat ? 'Yes' : 'No'})`);
      
      // Create a stream from the canvas with high framerate
      const stream = canvas.captureStream(30); // 30 FPS

      if (!stream) {
        throw new Error(`Failed to create stream for ${id}`);
      }
      
      // Configure the media recorder
      const mediaRecorder = new MediaRecorder(stream, {
        mimeType: bestMimeType,
        videoBitsPerSecond: options.videoBitsPerSecond || 5000000 // 5 Mbps
      });
      
      const chunks: Blob[] = [];
      
      // Setup data available handler - crucial for capturing video data
      mediaRecorder.ondataavailable = (event) => {
        if (event.data && event.data.size > 0) {
          chunks.push(event.data);
          console.log(`Recorder ${id}: got chunk of ${event.data.size} bytes, total chunks: ${chunks.length}`);
        }
      };

      // Create explicit data collection interval
      const dataCollectionInterval = window.setInterval(() => {
        if (mediaRecorder && mediaRecorder.state === 'recording') {
          try {
            mediaRecorder.requestData(); // Force data chunk collection
          } catch (e) {
            // Ignore errors here
          }
        }
      }, 1000); // Every second
      
      // Handle error events
      mediaRecorder.onerror = (event) => {
        console.error(`MediaRecorder error for ${id}:`, event);
        setError(`Recording error: ${event.type}`);
      };
      
      // Store recorder state
      recordersRef.current[id] = {
        mediaRecorder,
        stream,
        chunks,
        startTime: Date.now(),
        mimeType: bestMimeType,
        dataCollectionInterval,
        preferredFormat // Store the preferred format
      };
      
      // Start recording - use small timeslice to ensure frequent data collection
      mediaRecorder.start(100); // Collect data every 100ms
      
      // Update recording state
      setIsRecording(prev => ({ ...prev, [id]: true }));
      
      console.log(`Started recording for ${id} with mime type ${bestMimeType}`);
    } catch (err) {
      console.error(`Error starting recording for ${id}:`, err);
      setError(err instanceof Error ? err.message : 'Unknown error occurred');
    }
  }, [getBestMimeType, isRecording]);

  // Convert to MP4 if needed using a client-side conversion approach
  const ensureCorrectFormat = useCallback(async (blob: Blob, preferredFormat: 'mp4' | 'webm', mimeType: string): Promise<{ blob: Blob, fileExtension: string }> => {
    const actualFormat = mimeType.includes('mp4') ? 'mp4' : 'webm';
    
    // If we're already using the preferred format, no conversion needed
    if ((preferredFormat === 'mp4' && actualFormat === 'mp4') || 
        (preferredFormat === 'webm' && actualFormat === 'webm')) {
      console.log(`Recording is already in the preferred format: ${actualFormat}`);
      return { blob, fileExtension: actualFormat };
    }
    
    // If preferred is webm but we got mp4, no need to convert as most players handle both
    if (preferredFormat === 'webm' && actualFormat === 'mp4') {
      console.log('Recording is MP4 but preferred is WebM - keeping as MP4');
      return { blob, fileExtension: 'mp4' };
    }
    
    // If preferred is mp4 but we got webm, we either need to convert or just use extension hack
    if (preferredFormat === 'mp4' && actualFormat === 'webm') {
      console.log('Recording is WebM but preferred is MP4 - using MP4 extension');
      
      // For simplicity, we'll just rename the extension to mp4
      // This is a workaround since browser conversion is complex
      // Most modern players will still play this correctly
      return { blob, fileExtension: 'mp4' };
    }
    
    return { blob, fileExtension: actualFormat };
  }, []);

  const stopRecording = useCallback((id: string): Promise<Blob | null> => {
    return new Promise<Blob | null>((resolve) => {
      const recorder = recordersRef.current[id];
      
      if (!recorder) {
        console.warn(`No recorder found for ${id}`);
        return resolve(null);
      }
      
      // If recorder already stopped, just return the chunks
      if (recorder.mediaRecorder.state === 'inactive') {
        if (recorder.chunks.length > 0) {
          const blob = new Blob(recorder.chunks, { type: recorder.mimeType });
          return resolve(blob);
        }
        return resolve(null);
      }
      
      const finishRecording = () => {
        const { mediaRecorder, stream, chunks, mimeType, dataCollectionInterval } = recorder;
        
        // Make one final request for data before stopping
        try {
          mediaRecorder.requestData();
        } catch (e) {
          // Ignore errors
        }
        
        // Set up stop handler
        mediaRecorder.onstop = () => {
          // Clean up interval
          if (dataCollectionInterval) {
            clearInterval(dataCollectionInterval);
          }
          
          // Clean up stream tracks
          stream.getTracks().forEach(track => track.stop());
          
          // Update recording state
          setIsRecording(prev => {
            const updated = { ...prev };
            delete updated[id];
            return updated;
          });
          
          // Check if we have any chunks
          if (chunks.length === 0) {
            console.error(`No chunks recorded for ${id}`);
            setError('No data recorded');
            return resolve(null);
          }
          
          // Create final blob with all chunks
          const blob = new Blob(chunks, { type: mimeType });
          console.log(`Completed recording for ${id}: ${chunks.length} chunks, ${blob.size} bytes`);
          
          // Clean up recorder reference but keep the record of the mime type
          const recorderInfo = recordersRef.current[id];
          delete recordersRef.current[id];
          
          resolve(blob);
        };
        
        // Stop the recorder
        mediaRecorder.stop();
      };
      
      // Ensure minimum recording time
      const recordingDuration = Date.now() - recorder.startTime;
      const minimumDuration = 2500; // 2.5 seconds minimum
      
      if (recordingDuration < minimumDuration) {
        console.log(`Recording too short (${recordingDuration}ms), waiting until ${minimumDuration}ms`);
        setTimeout(finishRecording, minimumDuration - recordingDuration);
      } else {
        finishRecording();
      }
    });
  }, []);

  const downloadRecording = useCallback(async (id: string, name?: string) => {
    try {
      const recorder = recordersRef.current[id];
      if (!recorder) {
        console.warn(`No recorder information for ${id}`);
      }
      
      // Get the preferred format and mime type before stopping the recorder
      const recorderPreferredFormat = recorder?.preferredFormat || preferredFormat;
      const recorderMimeType = recorder?.mimeType || '';
      
      const blob = await stopRecording(id);
      
      if (!blob) {
        console.error(`No blob returned for ${id}`);
        return;
      }
      
      if (blob.size === 0) {
        console.error(`Empty blob returned for ${id}`);
        setError('Recorded video is empty');
        return;
      }
      
      console.log(`Ready to download blob for ${id}: ${blob.size} bytes, MIME type: ${blob.type}`);
      
      // Ensure we're using the correct format based on preferences
      const { blob: processedBlob, fileExtension } = await ensureCorrectFormat(
        blob, 
        recorderPreferredFormat,
        recorderMimeType
      );
      
      // Create a download link
      const url = URL.createObjectURL(processedBlob);
      const a = document.createElement('a');
      a.style.display = 'none';
      a.href = url;
      
      // Create timestamp for filename
      const now = new Date();
      const timestamp = now.toISOString().replace(/[:.]/g, '-');
      
      // Use custom name if provided, otherwise generate one
      const filename = name 
        ? `${name}-${timestamp}.${fileExtension}`
        : `botbot-${id}-${timestamp}.${fileExtension}`;
        
      a.download = filename;
      console.log(`Downloading recording as: ${filename}`);
      
      // Trigger download
      document.body.appendChild(a);
      a.click();
      
      // Clean up
      setTimeout(() => {
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
      }, 100);
    } catch (err) {
      setError(err instanceof Error ? err.message : `Failed to download recording for ${id}`);
      console.error(`Error downloading recording for ${id}:`, err);
    }
  }, [stopRecording, preferredFormat, ensureCorrectFormat]);

  const stopAllRecordings = useCallback(async () => {
    const ids = Object.keys(recordersRef.current);
    
    for (const id of ids) {
      try {
        await downloadRecording(id);
      } catch (err) {
        console.error(`Error stopping recording for ${id}:`, err);
      }
    }
  }, [downloadRecording]);

  return {
    isRecording: useCallback((id: string) => !!isRecording[id], [isRecording]),
    error,
    startRecording,
    stopRecording,
    downloadRecording,
    stopAllRecordings
  };
} 