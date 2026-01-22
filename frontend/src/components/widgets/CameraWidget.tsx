'use client';

import { useState } from 'react';
import { Widget } from './Widget';
import { Play, Pause, Video, Camera, Save, MapPin, Route } from 'lucide-react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import RosCameraImg from '../ros-camera-img';
import { useLanguage } from '@/contexts/LanguageContext';
import { useDashboard } from '@/contexts/DashboardContext';
import {
  CameraType,
  CameraDataSource,
  BotBrainCamera,
  CamCamCamera,
} from '@/types/CameraType';
import { useNavigationTargets } from '@/contexts/NavigationTargetsContext';
import useSmoothedPose from '@/hooks/ros/useSmoothedPose';
import useRosNavPlan from '@/hooks/ros/useRosNavPlan';
import { DEFAULT_CAMERA_CONFIGS } from '@/utils/waypointProjection';

interface CameraWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  initialTopic?: string;
  props?: {
    dataSource?: CameraDataSource;
    botbrainCamera?: BotBrainCamera;
    camcamCamera?: CamCamCamera;
    topic?: string;
  };
}

export function CameraWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 400, height: 350 },
  initialTopic = 'compressed_camera',
  props,
}: CameraWidgetProps) {
  const [isPlaying, setIsPlaying] = useState(true);
  const [showSettings, setShowSettings] = useState(false);
  const [topic, setTopic] = useState(props?.topic ?? initialTopic);
  const [isEditingTopic, setIsEditingTopic] = useState(false);
  const [dataSource, setDataSource] = useState<CameraDataSource>(
    props?.dataSource ?? 'botbrain'
  );
  const [botbrainCamera, setBotbrainCamera] = useState<BotBrainCamera>(
    props?.botbrainCamera ?? 'frontal'
  );
  const [camcamCamera, setCamcamCamera] = useState<CamCamCamera>(
    props?.camcamCamera ?? 'thermal'
  );

  // AR overlay states
  const [waypointArEnabled, setWaypointArEnabled] = useState(false);
  const [pathArEnabled, setPathArEnabled] = useState(false);

  const [tempTopic, setTempTopic] = useState(topic);
  const { connection } = useRobotConnection();
  const { t } = useLanguage();
  const { updateWidgetProps } = useDashboard();

  // Navigation data for AR overlays
  const { targets: navigationTargets } = useNavigationTargets();
  const { navPlan } = useRosNavPlan('/plan');
  const smoothedPose = useSmoothedPose({ enabled: waypointArEnabled || pathArEnabled });

  // Function to handle topic changes
  const handleTopicChange = () => {
    setTopic(tempTopic);
    setIsEditingTopic(false);
  };

  // Function to save camera settings
  const handleSaveSettings = () => {
    const propsToSave: Record<string, any> = {
      dataSource,
      botbrainCamera,
      camcamCamera,
    };

    // Only save topic if in topic mode
    if (dataSource === 'topic') {
      propsToSave.topic = topic;
    }

    updateWidgetProps(id, propsToSave);
    setShowSettings(false);
  };

  // Get the camera type to display based on current settings
  const getCameraType = (): CameraType => {
    if (dataSource === 'botbrain') {
      return botbrainCamera === 'frontal' ? 'camera' : 'back';
    } else if (dataSource === 'camcam') {
      return camcamCamera; // 'thermal' or 'rgb'
    }
    return 'camera'; // fallback for topic mode
  };

  return (
    <Widget
      id={id}
      title="Camera Feed"
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      initialPosition={initialPosition}
      initialSize={initialSize}
      minWidth={320}
      minHeight={300}
      onSettingsClick={() => setShowSettings(!showSettings)}
    >
      <div className="h-full flex flex-col">
        {showSettings ? (
          <>
            {/* Data Source Selection */}
            <div className="mb-4 flex space-x-4 text-sm">
              <label className="flex items-center space-x-1">
                {t('myUI', 'cameraSource')}
              </label>
              <select
                className="bg-white dark:bg-botbot-dark text-gray-700 dark:text-white text-sm rounded-lg px-2 py-1 border border-gray-300 dark:border-gray-700 focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple"
                onChange={(e) => {
                  setDataSource(e.target.value as CameraDataSource);
                }}
                value={dataSource}
              >
                <option value="botbrain">{t('myUI', 'botbrain')}</option>
                <option value="camcam">{t('myUI', 'camcam')}</option>
                <option value="topic">{t('myUI', 'byTopic')}</option>
              </select>
            </div>

            {/* Camera/Topic Selection based on Data Source */}
            <div className="mb-2 flex items-center text-sm">
              <div className="w-full flex flex-col items-start">
                {dataSource === 'topic' ? (
                  // Custom Topic Input
                  <div className="w-full flex flex-row items-center gap-2">
                    <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                      {t('myUI', 'topic')}:
                    </label>
                    <div className="w-full flex flex-row items-center">
                      <Camera className="w-4 h-4 mr-1 text-gray-500" />
                      {isEditingTopic ? (
                        <div className="flex-1 flex">
                          <input
                            type="text"
                            value={tempTopic}
                            onChange={(e) => setTempTopic(e.target.value)}
                            className="flex-1 bg-gray-100 dark:bg-botbot-darker px-2 py-1 rounded text-sm border border-gray-300 dark:border-botbot-dark"
                            onKeyDown={(e) => {
                              if (e.key === 'Enter') handleTopicChange();
                              if (e.key === 'Escape') {
                                setTempTopic(topic);
                                setIsEditingTopic(false);
                              }
                            }}
                            autoFocus
                          />
                          <button
                            onClick={handleTopicChange}
                            className="ml-1 px-2 py-1 bg-gray-200 dark:bg-botbot-dark rounded text-xs"
                          >
                            {t('myUI', 'save')}
                          </button>
                        </div>
                      ) : (
                        <div
                          className="flex-1 cursor-pointer px-2 py-1 rounded bg-gray-100 dark:bg-botbot-darker hover:bg-gray-200 dark:hover:bg-botbot-dark text-sm truncate"
                          onClick={() => {
                            setIsEditingTopic(true);
                            setTempTopic(topic);
                          }}
                          title={`ROS Topic: ${topic}`}
                        >
                          {topic}
                        </div>
                      )}
                    </div>
                  </div>
                ) : dataSource === 'botbrain' ? (
                  // BotBrain Camera Selection
                  <div className="flex flex-row items-center gap-2">
                    <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                      {t('myUI', 'camera')}:
                    </label>
                    <select
                      className="bg-white dark:bg-botbot-dark text-gray-700 dark:text-white text-sm rounded-lg px-2 py-1 border border-gray-300 dark:border-gray-700 focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple"
                      onChange={(e) => {
                        setBotbrainCamera(e.target.value as BotBrainCamera);
                      }}
                      value={botbrainCamera}
                    >
                      <option value="frontal">{t('myUI', 'frontalCamera')}</option>
                      <option value="back">{t('myUI', 'backCamera')}</option>
                    </select>
                  </div>
                ) : (
                  // CamCam Camera Selection
                  <div className="flex flex-row items-center gap-2">
                    <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                      {t('myUI', 'camera')}:
                    </label>
                    <select
                      className="bg-white dark:bg-botbot-dark text-gray-700 dark:text-white text-sm rounded-lg px-2 py-1 border border-gray-300 dark:border-gray-700 focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple"
                      onChange={(e) => {
                        setCamcamCamera(e.target.value as CamCamCamera);
                      }}
                      value={camcamCamera}
                    >
                      <option value="thermal">{t('robotCams', 'thermal')}</option>
                      <option value="rgb">{t('myUI', 'rgbInfrared')}</option>
                    </select>
                  </div>
                )}
              </div>
            </div>

            {/* Save button */}
            <div className="mt-4 flex justify-end">
              <button
                onClick={handleSaveSettings}
                className="flex items-center gap-2 px-4 py-2 bg-primary hover:bg-primary-dark text-white rounded-lg transition-colors"
              >
                <Save className="w-4 h-4" />
                {t('myUI', 'save')}
              </button>
            </div>
          </>
        ) : (
          <>
            {/* Camera feed area */}
            <div className="flex-1 bg-black rounded-md overflow-hidden">
              {connection.online && isPlaying ? (
                <div className="w-full h-full">
                  {dataSource === 'topic' ? (
                    // For topic mode, pass the custom topic with default camera config
                    <RosCameraImg
                      topicName={topic}
                      cameraType="camera"
                      waypointArEnabled={waypointArEnabled}
                      navigationTargets={navigationTargets}
                      robotPose={smoothedPose}
                      cameraConfig={DEFAULT_CAMERA_CONFIGS['camera']}
                      pathArEnabled={pathArEnabled}
                      navPlan={navPlan}
                    />
                  ) : (
                    // For BotBrain or CamCam, use the appropriate camera type
                    <RosCameraImg
                      cameraType={getCameraType()}
                      waypointArEnabled={waypointArEnabled}
                      navigationTargets={navigationTargets}
                      robotPose={smoothedPose}
                      cameraConfig={DEFAULT_CAMERA_CONFIGS[getCameraType()]}
                      pathArEnabled={pathArEnabled}
                      navPlan={navPlan}
                    />
                  )}
                </div>
              ) : (
                <div className="w-full h-full flex items-center justify-center text-gray-400">
                  <div className="flex flex-col items-center justify-center">
                    <Video className="w-16 h-16 mb-2" />
                    <p>
                      {!connection.online ? 'Robot offline' : 'Stream paused'}
                    </p>
                  </div>
                </div>
              )}
            </div>

            {/* Controls */}
            <div className="mt-2 flex justify-center items-center gap-2">
              <button
                onClick={() => setIsPlaying(!isPlaying)}
                className="p-2 rounded-full bg-gray-200 dark:bg-botbot-darker hover:bg-gray-300 dark:hover:bg-botbot-dark transition"
              >
                {isPlaying ? (
                  <Pause className="w-5 h-5 text-gray-700 dark:text-white" />
                ) : (
                  <Play className="w-5 h-5 text-gray-700 dark:text-white" />
                )}
              </button>

              {/* AR toggle buttons - only show when robot is online */}
              {connection.online && (
                <>
                  {/* Waypoint AR toggle button */}
                  <button
                    onClick={() => setWaypointArEnabled(!waypointArEnabled)}
                    className={`rounded-lg p-1.5 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple transition-colors ${
                      waypointArEnabled
                        ? "bg-cyan-500 text-white border-cyan-600"
                        : "bg-gray-200 dark:bg-botbot-darker text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                    }`}
                    aria-label={t('robotCams', 'waypointAr')}
                    title={waypointArEnabled ? t('robotCams', 'waypointArEnabled') : t('robotCams', 'waypointArDisabled')}
                  >
                    <MapPin className="w-4 h-4" />
                  </button>

                  {/* Path AR toggle button */}
                  <button
                    onClick={() => setPathArEnabled(!pathArEnabled)}
                    className={`rounded-lg p-1.5 border focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple transition-colors ${
                      pathArEnabled
                        ? "bg-green-500 text-white border-green-600"
                        : "bg-gray-200 dark:bg-botbot-darker text-gray-700 dark:text-white border-gray-300 dark:border-gray-700"
                    }`}
                    aria-label={t('robotCams', 'pathAr')}
                    title={pathArEnabled ? t('robotCams', 'pathArEnabled') : t('robotCams', 'pathArDisabled')}
                  >
                    <Route className="w-4 h-4" />
                  </button>
                </>
              )}
            </div>
          </>
        )}
      </div>
    </Widget>
  );
}
