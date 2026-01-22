'use client';

import Container from '@/ui/container';
import { Robot3DViewer } from './robot-3d-viewer';
import { useEffect, useRef, useState, useCallback } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useLanguage } from '@/contexts/LanguageContext';
import RosCameraImg from './ros-camera-img';
import { Crosshair, Video, Circle, Grid, CornerUpLeft, Volume2, VolumeX, Map, Box, Mic, MicOff, Radio, MapPin, Route } from 'lucide-react';
import { CameraType, OverlayType } from '@/types/CameraType';
import useVideoRecorder from '@/hooks/useVideoRecorder';
import useRosAudioStream from '@/hooks/useRosAudioStream';
import useRosMicrophoneStream from '@/hooks/useRosMicrophoneStream';
import useRosPushToTalk from '@/hooks/useRosPushToTalk';
import { auditLogger } from '@/utils/audit-logger';
import MapViewNav2 from './map-view-nav2';
import { useThermalCamera, THERMAL_PALETTES, ThermalPaletteName } from '@/hooks/ros/useThermalCamera';
import { useNavigationTargets } from '@/contexts/NavigationTargetsContext';
import useSmoothedPose from '@/hooks/ros/useSmoothedPose';
import useRosNavPlan from '@/hooks/ros/useRosNavPlan';
import { DEFAULT_CAMERA_CONFIGS } from '@/utils/waypointProjection';

const MAIN_CAMERA_ID = 'main-camera';
const SIDE_CAMERA_ID = 'side-camera';

interface RobotCamsProps {
  isMobile?: boolean;
}

export default function RobotCams({ isMobile = false }: RobotCamsProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const [sideCam, setSideCam] = useState<CameraType>('back');
  const [thermalMode, setThermalMode] = useState<ThermalPaletteName>('WHITEHOT');
  const [containerHeight, setContainerHeight] = useState(0);
  const camDivRef = useRef<HTMLDivElement>(null);
  const { connection } = useRobotConnection();
  const { t } = useLanguage();

  // Thermal camera controls
  const {
    setPalette,
    triggerFlatFieldCorrection,
    isLoading: isThermalLoading,
    error: thermalError,
  } = useThermalCamera();
  
  // Video overlay state
  const [videoOverlay, setVideoOverlay] = useState<OverlayType>('none');

  // Waypoint AR overlay state
  const [waypointArEnabled, setWaypointArEnabled] = useState(false);
  const { targets: navigationTargets } = useNavigationTargets();

  // Path AR overlay state
  const [pathArEnabled, setPathArEnabled] = useState(false);
  const { navPlan } = useRosNavPlan('/plan');

  // Smoothed pose for AR overlays (enabled when either AR feature is active)
  const smoothedPose = useSmoothedPose({ enabled: waypointArEnabled || pathArEnabled });

  // Recording state
  const [isProcessingRecord, setIsProcessingRecord] = useState(false);
  const { isRecording, error, startRecording, stopRecording, downloadRecording } = useVideoRecorder();
  
  // Audio streaming state
  // Using channel1 (raw data from mic1)
  const { isPlaying: isAudioPlaying, error: audioError, toggleStreaming } = useRosAudioStream('/audio/channel0');

  // Microphone streaming state
  const {
    isStreaming: isMicrophoneOn,
    error: microphoneError,
    hasPermission,
    toggleStreaming: toggleMicrophone
  } = useRosMicrophoneStream('/audio_streaming');

  // Push-to-talk state (uses same /audio_streaming topic)
  const {
    isTransmitting: isPushToTalkActive,
    error: pushToTalkError,
    hasPermission: pushToTalkPermission,
    startTransmitting,
    stopTransmitting,
    initialize: initializePushToTalk
  } = useRosPushToTalk('/audio_streaming');

  // Report any recording errors
  useEffect(() => {
    if (error) {
      console.error('Recording error:', error);
    }
  }, [error]);

  // Report any audio streaming errors
  useEffect(() => {
    if (audioError) {
      console.error('Audio streaming error:', audioError);
    }
  }, [audioError]);

  // Report any microphone streaming errors
  useEffect(() => {
    if (microphoneError) {
      console.error('Microphone streaming error:', microphoneError);
      // Could show a notification to the user here
    }
  }, [microphoneError]);

  // Report any push-to-talk errors
  useEffect(() => {
    if (pushToTalkError) {
      console.error('Push-to-talk error:', pushToTalkError);
      // Could show a notification to the user here
    }
  }, [pushToTalkError]);

  useEffect(() => {
    if (!containerRef.current) return;

    const updateHeight = () => {
      if (!containerRef.current) return;
      // Calculate dynamic height based on container's current size
      const newHeight = containerRef.current.clientHeight * 0.6; // Use 60% of container height for 3D viewer
      setContainerHeight(newHeight);
    };

    const resizeObserver = new ResizeObserver(() => {
      updateHeight();
    });

    resizeObserver.observe(containerRef.current);
    updateHeight(); // Initial calculation

    return () => {
      resizeObserver.disconnect();
    };
  }, []);

  // Get canvas elements by ID - more reliable than querySelector
  const getCanvasElement = (cameraType: string): HTMLCanvasElement | null => {
    const element = document.getElementById(`camera-canvas-${cameraType}`);
    return element instanceof HTMLCanvasElement ? element : null;
  };

  // Handle downloading recordings with appropriate filenames
  const downloadAllRecordings = async () => {
    try {
      setIsProcessingRecord(true);
      
      // Download main camera recording if active
      if (isRecording(MAIN_CAMERA_ID)) {
        console.log('Downloading main camera recording');
        await downloadRecording(MAIN_CAMERA_ID, `botbot-main-camera`);
        
        // Log recording stop
        auditLogger.logRecordingStopped(
          'main',
          0, // Duration could be tracked if needed
          connection.connectedRobot?.id,
          connection.connectedRobot?.name
        );
      }
      
      // Download side camera recording if active
      if (isRecording(SIDE_CAMERA_ID)) {
        console.log(`Downloading ${sideCam} camera recording`);
        await downloadRecording(SIDE_CAMERA_ID, `botbot-${sideCam}-camera`);
        
        // Log recording stop
        auditLogger.logRecordingStopped(
          sideCam,
          0, // Duration could be tracked if needed
          connection.connectedRobot?.id,
          connection.connectedRobot?.name
        );
      }
    } catch (err) {
      console.error('Error downloading recordings:', err);
    } finally {
      setIsProcessingRecord(false);
    }
  };

  const handleRecordToggle = async () => {
    if (isProcessingRecord) {
      return; // Prevent multiple simultaneous operations
    }
    
    const anyRecording = isRecording(MAIN_CAMERA_ID) || isRecording(SIDE_CAMERA_ID);
    
    if (anyRecording) {
      // Stop and download all recordings
      await downloadAllRecordings();
      return;
    }
    
    // Start new recordings
    setIsProcessingRecord(true);
    
    try {
      // Get the canvas elements to record
      const mainCanvas = getCanvasElement('camera');
      const sideCanvas = getCanvasElement(sideCam);
      
      // Log what we found
      console.log('Found canvases:', { 
        mainCanvas: !!mainCanvas, 
        sideCanvas: !!sideCanvas
      });
      
      // Start recording main camera if available
      if (mainCanvas) {
        console.log('Starting recording of main camera');
        await startRecording(MAIN_CAMERA_ID, mainCanvas);
        
        // Log recording start
        auditLogger.logRecordingStarted(
          'main',
          connection.connectedRobot?.id,
          connection.connectedRobot?.name
        );
      } else {
        console.warn('Main camera canvas not found');
      }
      
      // Start recording side camera if available
      if (sideCanvas) {
        console.log(`Starting recording of ${sideCam} camera`);
        await startRecording(SIDE_CAMERA_ID, sideCanvas);
        
        // Log recording start
        auditLogger.logRecordingStarted(
          sideCam,
          connection.connectedRobot?.id,
          connection.connectedRobot?.name
        );
      } else {
        console.warn('Side camera canvas not found');
      }
    } catch (err) {
      console.error('Error toggling recording:', err);
    } finally {
      setIsProcessingRecord(false);
    }
  };

  // Check if any recording is active
  const isAnyRecording = isRecording(MAIN_CAMERA_ID) || isRecording(SIDE_CAMERA_ID);

  // Handle audio streaming toggle with logging
  const handleAudioToggle = useCallback(() => {
    // Log audio event
    auditLogger.log({
      event_type: 'audio',
      event_action: isAudioPlaying ? 'audio_stopped' : 'audio_played',
      robot_id: connection.connectedRobot?.id,
      robot_name: connection.connectedRobot?.name,
      event_details: {
        source: 'audio_stream',
        topic: '/audio/channel1'
      }
    });

    toggleStreaming();
  }, [isAudioPlaying, toggleStreaming, connection.connectedRobot]);

  // Handle microphone toggle with logging
  const handleMicrophoneToggle = useCallback(async () => {
    // Log microphone event
    auditLogger.log({
      event_type: 'audio',
      event_action: isMicrophoneOn ? 'audio_stopped' : 'audio_played',
      robot_id: connection.connectedRobot?.id,
      robot_name: connection.connectedRobot?.name,
      event_details: {
        source: 'microphone_input',
        type: 'microphone',
        action: isMicrophoneOn ? 'muted' : 'unmuted',
        topic: '/audio_streaming',
        permission_status: hasPermission
      }
    });

    // Toggle microphone streaming
    await toggleMicrophone();
  }, [isMicrophoneOn, connection.connectedRobot, toggleMicrophone, hasPermission]);

  // Handle push-to-talk press (mouse down or touch start)
  const handlePushToTalkPress = useCallback(async () => {
    // Log push-to-talk start event
    auditLogger.log({
      event_type: 'audio',
      event_action: 'audio_played',
      robot_id: connection.connectedRobot?.id,
      robot_name: connection.connectedRobot?.name,
      event_details: {
        source: 'push_to_talk',
        type: 'push_to_talk',
        action: 'started',
        topic: '/audio_streaming'
      }
    });

    // Start transmitting
    await startTransmitting();
  }, [connection.connectedRobot, startTransmitting]);

  // Handle push-to-talk release (mouse up or touch end)
  const handlePushToTalkRelease = useCallback(() => {
    // Log push-to-talk stop event
    auditLogger.log({
      event_type: 'audio',
      event_action: 'audio_stopped',
      robot_id: connection.connectedRobot?.id,
      robot_name: connection.connectedRobot?.name,
      event_details: {
        source: 'push_to_talk',
        type: 'push_to_talk',
        action: 'stopped',
        topic: '/audio_streaming'
      }
    });

    // Stop transmitting
    stopTransmitting();
  }, [connection.connectedRobot, stopTransmitting]);

  // Handle thermal palette change
  const handleThermalPaletteChange = useCallback(async (paletteName: ThermalPaletteName) => {
    console.log('[RobotCams] handleThermalPaletteChange called with:', paletteName);
    setThermalMode(paletteName);
    const paletteValue = THERMAL_PALETTES[paletteName];
    console.log('[RobotCams] Calling setPalette with value:', paletteValue);
    const result = await setPalette(paletteValue);
    console.log('[RobotCams] setPalette result:', result);
  }, [setPalette]);

  // Handle flat-field correction
  const handleFlatFieldCorrection = useCallback(async () => {
    console.log('[RobotCams] handleFlatFieldCorrection called');
    const result = await triggerFlatFieldCorrection();
    console.log('[RobotCams] triggerFlatFieldCorrection result:', result);
  }, [triggerFlatFieldCorrection]);

  const _NotPluggedInPlaceholder = () => (
    <div className="w-full h-full flex flex-col items-center justify-center text-center p-4 bg-gray-100 dark:bg-botbot-dark rounded-lg">
      <div className="flex flex-col items-center gap-1">
        <svg
          width="25"
          height="25"
          viewBox="0 0 24 24"
          fill="none"
          xmlns="http://www.w3.org/2000/svg"
        >
          <path
            d="M21 15a1 1 0 0 0-1 1v3a1 1 0 0 1-1 1H5a1 1 0 0 1-1-1V5a1 1 0 0 1 1-1h3a1 1 0 0 0 0-2H5a3 3 0 0 0-3 3v14a3 3 0 0 0 3 3h14a3 3 0 0 0 3-3v-3a1 1 0 0 0-1-1ZM15 3a1 1 0 0 0 0 2h2.59l-5.3 5.29a1 1 0 0 0 0 1.42a1 1 0 0 0 1.42 0L19 6.41V9a1 1 0 0 0 2 0V4a1 1 0 0 0-1-1Z"
            fill="currentColor"
          />
        </svg>
        <span className="text-sm">{t('robotCams', 'nothingConnected')}</span>
      </div>
    </div>
  );

  // Mobile layout
  if (isMobile) {
    return (
      <div id="container-ref" className="w-full" ref={containerRef}>
        {/* Video Streams Section */}
        <Container
          className="w-full mb-4"
          customContentClasses="w-full"
        >
          <div className="flex justify-between items-center mb-4">
            <h3 className="heading-text flex items-center">
              <Video className="mr-2 w-5 h-5" />
              {t('robotCams', 'live')}
              {connection.online && (
                <span className="ml-2 w-2 h-2 rounded-full bg-red-600 animate-pulse"></span>
              )}
            </h3>
            
            {connection.online && (
              <div className="flex items-center gap-2">
                {/* Waypoint AR toggle button */}
                <button
                  onClick={() => setWaypointArEnabled(!waypointArEnabled)}
                  className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple transition-colors ${
                    waypointArEnabled
                      ? "bg-cyan-500 text-white border-cyan-600"
                      : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                  }`}
                  aria-label={t('robotCams', 'waypointAr')}
                  title={waypointArEnabled ? t('robotCams', 'waypointArEnabled') : t('robotCams', 'waypointArDisabled')}
                >
                  <MapPin className="w-5 h-5" />
                </button>

                {/* Path AR toggle button */}
                <button
                  onClick={() => setPathArEnabled(!pathArEnabled)}
                  className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple transition-colors ${
                    pathArEnabled
                      ? "bg-green-500 text-white border-green-600"
                      : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                  }`}
                  aria-label={t('robotCams', 'pathAr')}
                  title={pathArEnabled ? t('robotCams', 'pathArEnabled') : t('robotCams', 'pathArDisabled')}
                >
                  <Route className="w-5 h-5" />
                </button>

                {/* Video overlay dropdown */}
                <select
                  className="bg-white dark:bg-botbot-dark text-gray-700 dark:text-white text-sm rounded-lg px-2 py-1 border border-gray-300 dark:border-gray-700 focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple"
                  onChange={(e) => setVideoOverlay(e.target.value as OverlayType)}
                  value={videoOverlay}
                  aria-label={t('robotCams', 'overlay')}
                  title={t('robotCams', 'overlay')}
                >
                  <option value="none">{t('robotCams', 'noOverlay')}</option>
                  <option value="crosshair">{t('robotCams', 'crosshair')}</option>
                  <option value="grid">{t('robotCams', 'grid')}</option>
                  <option value="corners">{t('robotCams', 'corners')}</option>
                </select>

                <select
                  className="bg-white dark:bg-botbot-dark text-gray-700 dark:text-white text-sm rounded-lg px-2 py-1 border border-gray-300 dark:border-gray-700 focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple"
                  onChange={(e) => setSideCam(e.target.value as CameraType)}
                  value={sideCam}
                >
                  <option value="back">{t('robotCams', 'backCamera')}</option>
                  <option value="thermal">{t('robotCams', 'thermal')}</option>
                  <option value="rgb">{t('robotCams', 'rgb')}</option>
                </select>

                {sideCam === 'thermal' && (
                  <>
                    <button
                      className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple ${
                        isThermalLoading
                          ? "bg-yellow-500 text-white border-yellow-600"
                          : thermalError
                            ? "bg-red-600 text-white border-red-700"
                            : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                      }`}
                      onClick={handleFlatFieldCorrection}
                      disabled={isThermalLoading}
                      aria-label="Flat-field correction"
                      title={thermalError || "Flat-field correction (calibrate thermal camera)"}
                    >
                      {isThermalLoading ? (
                        <div className="w-5 h-5 border-2 border-white border-t-transparent rounded-full animate-spin"></div>
                      ) : (
                        <Crosshair className="w-5 h-5" />
                      )}
                    </button>
                    <select
                      className="bg-white dark:bg-botbot-dark text-gray-700 dark:text-white text-sm rounded-lg px-2 py-1 border border-gray-300 dark:border-gray-700 focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple"
                      onChange={(e) => handleThermalPaletteChange(e.target.value as ThermalPaletteName)}
                      value={thermalMode}
                      disabled={isThermalLoading}
                    >
                      <option value="WHITEHOT">White Hot</option>
                      <option value="BLACKHOT">Black Hot</option>
                      <option value="FUSION1">Fusion 1</option>
                      <option value="RAINBOW">Rainbow</option>
                      <option value="FUSION2">Fusion 2</option>
                      <option value="IRONBOW1">Ironbow 1</option>
                      <option value="IRONBOW2">Ironbow 2</option>
                      <option value="SEPIA">Sepia</option>
                      <option value="COLOR1">Color 1</option>
                      <option value="COLOR2">Color 2</option>
                      <option value="ICEFIRE">Ice Fire</option>
                      <option value="RAIN">Rain</option>
                      <option value="REDHOT">Red Hot</option>
                      <option value="GREENHOT">Green Hot</option>
                      <option value="DEEPBLUE">Deep Blue</option>
                      <option value="WINTER">Winter</option>
                      <option value="SUMMER">Summer</option>
                    </select>
                  </>
                )}

                <button
                  className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple ${
                    isProcessingRecord
                      ? "bg-yellow-500 text-white border-yellow-600"
                      : isAnyRecording
                        ? "bg-red-600 text-white border-red-700"
                        : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                  }`}
                  onClick={handleRecordToggle}
                  disabled={isProcessingRecord}
                  aria-label={isProcessingRecord ? "Processing..." : isAnyRecording ? "Stop recording" : "Start recording"}
                  title={isProcessingRecord ? "Processing..." : isAnyRecording ? "Stop recording" : "Start recording"}
                >
                  {isProcessingRecord ? (
                    <div className="w-5 h-5 border-2 border-white border-t-transparent rounded-full animate-spin"></div>
                  ) : (
                    <Circle className="w-5 h-5" />
                  )}
                </button>

                <button
                  className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple ${
                    isAudioPlaying
                      ? "bg-green-600 text-white border-green-700"
                      : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                  }`}
                  onClick={handleAudioToggle}
                  aria-label={isAudioPlaying ? "Stop audio streaming" : "Start audio streaming"}
                  title={isAudioPlaying ? "Stop audio streaming" : "Start audio streaming"}
                >
                  {isAudioPlaying ? (
                    <Volume2 className="w-5 h-5 animate-pulse" />
                  ) : (
                    <VolumeX className="w-5 h-5" />
                  )}
                </button>

                <button
                  className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple ${
                    isMicrophoneOn
                      ? "bg-blue-600 text-white border-blue-700"
                      : microphoneError
                        ? "bg-red-600 text-white border-red-700"
                        : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                  }`}
                  onClick={handleMicrophoneToggle}
                  aria-label={isMicrophoneOn ? "Stop microphone streaming" : "Start microphone streaming"}
                  title={
                    microphoneError
                      ? `Error: ${microphoneError}`
                      : isMicrophoneOn
                        ? "Stop microphone streaming to robot"
                        : "Start microphone streaming to robot"
                  }
                >
                  {isMicrophoneOn ? (
                    <Mic className="w-5 h-5 animate-pulse" />
                  ) : (
                    <MicOff className="w-5 h-5" />
                  )}
                </button>

                <button
                  className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple transition-colors ${
                    isPushToTalkActive
                      ? "bg-orange-600 text-white border-orange-700"
                      : pushToTalkError
                        ? "bg-red-600 text-white border-red-700"
                        : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700 hover:bg-gray-100 dark:hover:bg-gray-700"
                  }`}
                  onMouseDown={handlePushToTalkPress}
                  onMouseUp={handlePushToTalkRelease}
                  onMouseLeave={handlePushToTalkRelease}
                  onTouchStart={handlePushToTalkPress}
                  onTouchEnd={handlePushToTalkRelease}
                  onTouchCancel={handlePushToTalkRelease}
                  aria-label="Push to talk"
                  title={
                    pushToTalkError
                      ? `Error: ${pushToTalkError}`
                      : isPushToTalkActive
                        ? "Release to stop talking"
                        : "Hold to talk"
                  }
                >
                  <Radio className={`w-5 h-5 ${isPushToTalkActive ? 'animate-pulse' : ''}`} />
                </button>
              </div>
            )}
          </div>
          
          {/* Camera displays - side by side */}
          <div
            className="w-full flex flex-row gap-3 h-[200px]"
            ref={camDivRef}
          >
            <div className="w-1/2 h-full ros-camera-main relative">
              <RosCameraImg
                cameraType="camera"
                overlay={videoOverlay}
                waypointArEnabled={waypointArEnabled}
                navigationTargets={navigationTargets}
                robotPose={smoothedPose}
                cameraConfig={DEFAULT_CAMERA_CONFIGS['camera']}
                pathArEnabled={pathArEnabled}
                navPlan={navPlan}
              />
              {isRecording(MAIN_CAMERA_ID) && (
                <div className="absolute top-3 right-3 flex items-center gap-1.5 bg-red-600/70 text-white px-2 py-1 rounded-md">
                  <Circle className="w-3 h-3 fill-current animate-pulse" />
                  <span className="text-xs font-medium">REC</span>
                </div>
              )}
            </div>
            <div className="w-1/2 h-full relative">
              <RosCameraImg
                key={sideCam}
                cameraType={sideCam}
                offlineMsg={t('robotOffline', 'noPayload')}
                overlay={videoOverlay}
                waypointArEnabled={waypointArEnabled}
                navigationTargets={navigationTargets}
                robotPose={smoothedPose}
                cameraConfig={DEFAULT_CAMERA_CONFIGS[sideCam]}
                pathArEnabled={pathArEnabled}
                navPlan={navPlan}
              />
              {isRecording(SIDE_CAMERA_ID) && (
                <div className="absolute top-3 right-3 flex items-center gap-1.5 bg-red-600/70 text-white px-2 py-1 rounded-md">
                  <Circle className="w-3 h-3 fill-current animate-pulse" />
                  <span className="text-xs font-medium">REC</span>
                </div>
              )}
            </div>
          </div>
        </Container>

        {/* 3D Viewer Section */}
        <Container
          className="w-full mb-4"
          customContentClasses="w-full"
        >
          <div className="flex justify-between items-center mb-4">
            <h3 className="heading-text flex items-center">
              <Box className="mr-2 w-5 h-5" />
              3D Viewer
            </h3>
          </div>
          <div className="w-full h-[300px] dark:bg-botbot-dark rounded-lg overflow-hidden">
            <Robot3DViewer canvasInitialHeight={300} />
          </div>
        </Container>

        {/* Map View Section */}
        <Container
          className="w-full"
          customContentClasses="w-full"
        >
          <div className="flex justify-between items-center mb-4">
            <h3 className="heading-text flex items-center">
              <Map className="mr-2 w-5 h-5" />
              Map View
            </h3>
          </div>
          <div className="w-full h-[300px]">
            <MapViewNav2 className="w-full h-full" />
          </div>
        </Container>
      </div>
    );
  }

  // Desktop layout (original)
  return (
    <div id="container-ref" className="w-full h-full" ref={containerRef}>
      <Container
        className="w-full h-full"
        customContentClasses="w-full h-full flex flex-col"
      >
        <div className="flex flex-col gap-3 h-full overflow-hidden">
          {/* Cameras Section - Now on top */}
          <div className="w-full min-h-[220px] md:min-h-[280px] flex-[0.5]">
            <div className="flex justify-between items-center mb-4">
              <h3 className="heading-text flex items-center">
                <Video className="mr-2 w-5 h-5" />
                {t('robotCams', 'live')}
                {connection.online && (
                  <span className="ml-2 w-2 h-2 rounded-full bg-red-600 animate-pulse"></span>
                )}
              </h3>
              
              {connection.online && (
                <div className="flex items-center gap-2">
                  {/* Waypoint AR toggle button */}
                  <button
                    onClick={() => setWaypointArEnabled(!waypointArEnabled)}
                    className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple transition-colors ${
                      waypointArEnabled
                        ? "bg-cyan-500 text-white border-cyan-600"
                        : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                    }`}
                    aria-label={t('robotCams', 'waypointAr')}
                    title={waypointArEnabled ? t('robotCams', 'waypointArEnabled') : t('robotCams', 'waypointArDisabled')}
                  >
                    <MapPin className="w-5 h-5" />
                  </button>

                  {/* Path AR toggle button */}
                  <button
                    onClick={() => setPathArEnabled(!pathArEnabled)}
                    className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple transition-colors ${
                      pathArEnabled
                        ? "bg-green-500 text-white border-green-600"
                        : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                    }`}
                    aria-label={t('robotCams', 'pathAr')}
                    title={pathArEnabled ? t('robotCams', 'pathArEnabled') : t('robotCams', 'pathArDisabled')}
                  >
                    <Route className="w-5 h-5" />
                  </button>

                  {/* Video overlay dropdown */}
                  <select
                    className="bg-white dark:bg-botbot-dark text-gray-700 dark:text-white text-sm rounded-lg px-2 py-1 border border-gray-300 dark:border-gray-700 focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple"
                    onChange={(e) => setVideoOverlay(e.target.value as OverlayType)}
                    value={videoOverlay}
                    aria-label={t('robotCams', 'overlay')}
                    title={t('robotCams', 'overlay')}
                  >
                    <option value="none">{t('robotCams', 'noOverlay')}</option>
                    <option value="crosshair">{t('robotCams', 'crosshair')}</option>
                    <option value="grid">{t('robotCams', 'grid')}</option>
                    <option value="corners">{t('robotCams', 'corners')}</option>
                  </select>

                  <select
                    className="bg-white dark:bg-botbot-dark text-gray-700 dark:text-white text-sm rounded-lg px-2 py-1 border border-gray-300 dark:border-gray-700 focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple"
                    onChange={(e) => setSideCam(e.target.value as CameraType)}
                    value={sideCam}
                  >
                    <option value="back">{t('robotCams', 'backCamera')}</option>
                    <option value="thermal">{t('robotCams', 'thermal')}</option>
                    <option value="rgb">{t('robotCams', 'rgb')}</option>
                  </select>

                  {sideCam === 'thermal' && (
                    <>
                      <button
                        className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple ${
                          isThermalLoading
                            ? "bg-yellow-500 text-white border-yellow-600"
                            : thermalError
                              ? "bg-red-600 text-white border-red-700"
                              : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                        }`}
                        onClick={handleFlatFieldCorrection}
                        disabled={isThermalLoading}
                        aria-label="Flat-field correction"
                        title={thermalError || "Flat-field correction (calibrate thermal camera)"}
                      >
                        {isThermalLoading ? (
                          <div className="w-5 h-5 border-2 border-white border-t-transparent rounded-full animate-spin"></div>
                        ) : (
                          <Crosshair className="w-5 h-5" />
                        )}
                      </button>
                      <select
                        className="bg-white dark:bg-botbot-dark text-gray-700 dark:text-white text-sm rounded-lg px-2 py-1 border border-gray-300 dark:border-gray-700 focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple"
                        onChange={(e) => handleThermalPaletteChange(e.target.value as ThermalPaletteName)}
                        value={thermalMode}
                        disabled={isThermalLoading}
                      >
                        <option value="WHITEHOT">White Hot</option>
                        <option value="BLACKHOT">Black Hot</option>
                        <option value="FUSION1">Fusion 1</option>
                        <option value="RAINBOW">Rainbow</option>
                        <option value="FUSION2">Fusion 2</option>
                        <option value="IRONBOW1">Ironbow 1</option>
                        <option value="IRONBOW2">Ironbow 2</option>
                        <option value="SEPIA">Sepia</option>
                        <option value="COLOR1">Color 1</option>
                        <option value="COLOR2">Color 2</option>
                        <option value="ICEFIRE">Ice Fire</option>
                        <option value="RAIN">Rain</option>
                        <option value="REDHOT">Red Hot</option>
                        <option value="GREENHOT">Green Hot</option>
                        <option value="DEEPBLUE">Deep Blue</option>
                        <option value="WINTER">Winter</option>
                        <option value="SUMMER">Summer</option>
                      </select>
                    </>
                  )}

                  <button
                    className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple ${
                      isProcessingRecord 
                        ? "bg-yellow-500 text-white border-yellow-600"
                        : isAnyRecording
                          ? "bg-red-600 text-white border-red-700"
                          : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                    }`}
                    onClick={handleRecordToggle}
                    disabled={isProcessingRecord}
                    aria-label={isProcessingRecord ? "Processing..." : isAnyRecording ? "Stop recording" : "Start recording"}
                    title={isProcessingRecord ? "Processing..." : isAnyRecording ? "Stop recording" : "Start recording"}
                  >
                    {isProcessingRecord ? (
                      <div className="w-5 h-5 border-2 border-white border-t-transparent rounded-full animate-spin"></div>
                    ) : (
                      <Circle className="w-5 h-5" />
                    )}
                  </button>

                  <button
                    className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple ${
                      isAudioPlaying
                        ? "bg-green-600 text-white border-green-700"
                        : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                    }`}
                    onClick={handleAudioToggle}
                    aria-label={isAudioPlaying ? "Stop audio streaming" : "Start audio streaming"}
                    title={isAudioPlaying ? "Stop audio streaming" : "Start audio streaming"}
                  >
                    {isAudioPlaying ? (
                      <Volume2 className="w-5 h-5 animate-pulse" />
                    ) : (
                      <VolumeX className="w-5 h-5" />
                    )}
                  </button>

                  <button
                    className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple ${
                      isMicrophoneOn
                        ? "bg-blue-600 text-white border-blue-700"
                        : microphoneError
                          ? "bg-red-600 text-white border-red-700"
                          : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                    }`}
                    onClick={handleMicrophoneToggle}
                    aria-label={isMicrophoneOn ? "Stop microphone streaming" : "Start microphone streaming"}
                    title={
                      microphoneError
                        ? `Error: ${microphoneError}`
                        : isMicrophoneOn
                          ? "Stop microphone streaming to robot"
                          : "Start microphone streaming to robot"
                    }
                  >
                    {isMicrophoneOn ? (
                      <Mic className="w-5 h-5 animate-pulse" />
                    ) : (
                      <MicOff className="w-5 h-5" />
                    )}
                  </button>

                  <button
                    className={`rounded-lg p-1 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple transition-colors ${
                      isPushToTalkActive
                        ? "bg-orange-600 text-white border-orange-700"
                        : pushToTalkError
                          ? "bg-red-600 text-white border-red-700"
                          : "bg-white dark:bg-botbot-dark text-gray-700 dark:text-white border-gray-300 dark:border-gray-700 hover:bg-gray-100 dark:hover:bg-gray-700"
                    }`}
                    onMouseDown={handlePushToTalkPress}
                    onMouseUp={handlePushToTalkRelease}
                    onMouseLeave={handlePushToTalkRelease}
                    onTouchStart={handlePushToTalkPress}
                    onTouchEnd={handlePushToTalkRelease}
                    onTouchCancel={handlePushToTalkRelease}
                    aria-label="Push to talk"
                    title={
                      pushToTalkError
                        ? `Error: ${pushToTalkError}`
                        : isPushToTalkActive
                          ? "Release to stop talking"
                          : "Hold to talk"
                    }
                  >
                    <Radio className={`w-5 h-5 ${isPushToTalkActive ? 'animate-pulse' : ''}`} />
                  </button>
                </div>
              )}
            </div>
            
            {/* Camera displays */}
            <div
              className="w-full flex flex-col sm:flex-row gap-3 h-[calc(100%-40px)]"
              ref={camDivRef}
            >
              <div className="w-full sm:w-1/2 min-h-[200px] h-full ros-camera-main relative">
                <RosCameraImg
                  cameraType="camera"
                  overlay={videoOverlay}
                  waypointArEnabled={waypointArEnabled}
                  navigationTargets={navigationTargets}
                  robotPose={smoothedPose}
                  cameraConfig={DEFAULT_CAMERA_CONFIGS['camera']}
                  pathArEnabled={pathArEnabled}
                  navPlan={navPlan}
                />
                {isRecording(MAIN_CAMERA_ID) && (
                  <div className="absolute top-3 right-3 flex items-center gap-1.5 bg-red-600/70 text-white px-2 py-1 rounded-md">
                    <Circle className="w-3 h-3 fill-current animate-pulse" />
                    <span className="text-xs font-medium">REC</span>
                  </div>
                )}
              </div>
              <div className="w-full sm:w-1/2 min-h-[200px] h-full relative">
                <RosCameraImg
                  key={sideCam}
                  cameraType={sideCam}
                  offlineMsg={t('robotOffline', 'noPayload')}
                  overlay={videoOverlay}
                  waypointArEnabled={waypointArEnabled}
                  navigationTargets={navigationTargets}
                  robotPose={smoothedPose}
                  cameraConfig={DEFAULT_CAMERA_CONFIGS[sideCam]}
                  pathArEnabled={pathArEnabled}
                  navPlan={navPlan}
                />
                {isRecording(SIDE_CAMERA_ID) && (
                  <div className="absolute top-3 right-3 flex items-center gap-1.5 bg-red-600/70 text-white px-2 py-1 rounded-md">
                    <Circle className="w-3 h-3 fill-current animate-pulse" />
                    <span className="text-xs font-medium">REC</span>
                  </div>
                )}
              </div>
            </div>
          </div>

          {/* 3D Viewer and Map Section - Now at the bottom */}
          <div className="w-full flex-[0.5] min-h-[300px] flex gap-3">
            {/* 3D Viewer - Left half */}
            <div className="w-1/2 h-full dark:bg-botbot-dark rounded-lg overflow-hidden">
              <div className="w-full h-full relative">
                <div className="absolute top-3 left-3 z-10 bg-white dark:bg-gray-800 text-gray-700 dark:text-white text-xs rounded-lg px-3 py-1.5 border border-gray-300 dark:border-gray-700 shadow-sm">
                  <Box className="w-3 h-3 inline mr-1" />
                  3D View
                </div>
                <Robot3DViewer canvasInitialHeight={containerHeight || 300} />
              </div>
            </div>
            
            {/* Map View - Right half */}
            <div className="w-1/2 h-full">
              <div className="w-full h-full relative">
                <div className="absolute top-3 left-3 z-10 bg-white dark:bg-gray-800 text-gray-700 dark:text-white text-xs rounded-lg px-3 py-1.5 border border-gray-300 dark:border-gray-700 shadow-sm">
                  <Map className="w-3 h-3 inline mr-1" />
                  Map View
                </div>
                <MapViewNav2 className="w-full h-full" />
              </div>
            </div>
          </div>
        </div>
      </Container>
    </div>
  );
}
