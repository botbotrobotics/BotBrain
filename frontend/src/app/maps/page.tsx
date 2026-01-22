'use client';

import { useEffect, useState, useRef } from 'react';
import SimpleMapView from '@/components/simple-map-view';
import { MapPin, Play, Square, Clock, Edit, RefreshCw, Trash2, Download, CheckCircle, ChevronRight, ChevronLeft, Gamepad2, ArrowUp, ArrowDown, Lock, Unlock, AlertTriangle } from 'lucide-react';
import { useRosMappingServices } from '@/hooks/ros/useRosMappingServices';
import { useNotifications } from '@/contexts/NotificationsContext';
import RosCameraImg from '@/components/ros-camera-img';
import { JoysticksWrapper } from '@/components/ui/JoysticksWrapper';
import useRobotActions from '@/hooks/ros/useRobotActions';
import { useHeader } from '@/contexts/HeaderContext';
import { useWakeLock } from '@/hooks/useWakeLock';

interface MapLocation {
  id: string;
  name: string;
  path: string;
  lastUpdated: string;
  size: string;
}

export default function MapsPage() {
  const [selectedLocation, setSelectedLocation] = useState<string | null>(null);
  const [isMapping, setIsMapping] = useState(false);
  const [showMapNameInput, setShowMapNameInput] = useState(false);
  const [mapName, setMapName] = useState('');
  const [mappingTime, setMappingTime] = useState(0);
  const [locations, setLocations] = useState<MapLocation[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [loadingMapId, setLoadingMapId] = useState<string | null>(null);
  const [leftPanelCollapsed, setLeftPanelCollapsed] = useState(false);
  const [showJoysticks, setShowJoysticks] = useState(false);
  const [deletingMapId, setDeletingMapId] = useState<string | null>(null);
  const [showDeleteConfirm, setShowDeleteConfirm] = useState<string | null>(null);
  const [isSettingHome, setIsSettingHome] = useState(false);
  const intervalRef = useRef<NodeJS.Timeout | null>(null);
  
  const { dispatch } = useNotifications();
  const { joystickEnabled, setJoystickEnabled } = useHeader();
  const robotActions = useRobotActions();
  
  const {
    isConnected,
    listDatabaseFiles,
    loadDatabase,
    setMappingMode,
    setLocalizationMode,
    saveDatabase,
    deleteDatabase,
    getCurrentDatabase,
    formatDatabaseInfo,
  } = useRosMappingServices();

  // Keep the screen awake while on this page
  useWakeLock();

  useEffect(() => {
    document.title = 'Maps - BotBot';
  }, []);

  useEffect(() => {
    if (isConnected) {
      fetchMaps();
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [isConnected]);

  useEffect(() => {
    if (isMapping) {
      intervalRef.current = setInterval(() => {
        setMappingTime(prev => prev + 1);
      }, 1000);
    } else {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
      setMappingTime(0);
    }

    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
    };
  }, [isMapping]);

  const formatTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  const fetchMaps = async () => {
    if (!isConnected) return;

    setIsLoading(true);
    try {
      // Fetch both map list and current database in parallel
      const [dbFiles, currentDb] = await Promise.all([
        listDatabaseFiles(),
        getCurrentDatabase()
      ]);

      const formattedMaps = formatDatabaseInfo(dbFiles);
      const mapsWithIds = formattedMaps.map((map, index) => ({
        id: index.toString(),
        name: map.name,
        path: map.path,
        lastUpdated: map.lastModified || 'Unknown',
        size: map.size || 'Unknown',
      }));
      setLocations(mapsWithIds);

      // Match current database to a map in the list
      if (currentDb) {
        const activeMap = mapsWithIds.find(
          m => m.path === currentDb || m.name === currentDb || m.path.endsWith(currentDb)
        );
        if (activeMap) {
          setSelectedLocation(activeMap.id);
        }
      }
    } catch (error) {
      console.error('Failed to fetch maps:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handleStartMapping = async () => {
    if (!showMapNameInput) {
      setShowMapNameInput(true);
    } else if (mapName.trim()) {
      try {
        const dbFileName = `${mapName.trim().replace(/\s+/g, '_')}.db`;
        await setMappingMode(dbFileName, true);
        setIsMapping(true);
        setShowMapNameInput(false);
      } catch (error) {
        console.error('Failed to start mapping:', error);
        setIsMapping(false);
      }
    }
  };

  const handleStopMapping = async () => {
    try {
      await saveDatabase();
      await setLocalizationMode();
      setIsMapping(false);
      setMapName('');
      
      setTimeout(() => {
        fetchMaps();
      }, 1000);
    } catch (error) {
      console.error('Failed to stop mapping:', error);
    }
  };

  const handleLoadMap = async (mapId: string, mapPath: string, mapName: string) => {
    try {
      setLoadingMapId(mapId);
      // First load the database
      await loadDatabase(mapPath, false);
      // Then switch to localization mode
      await setLocalizationMode();
      
      // Mark as selected
      setSelectedLocation(mapId);
      
      // Show success notification
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'success',
          title: 'Map Loaded',
          message: `Successfully loaded and activated map: ${mapName}`,
        },
      });
    } catch (error) {
      console.error('Failed to load map:', error);
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'error',
          title: 'Failed to Load Map',
          message: `Could not load map: ${mapName}`,
        },
      });
    } finally {
      setLoadingMapId(null);
    }
  };

  const handleDeleteMap = async (mapId: string, mapPath: string, mapName: string) => {
    try {
      setDeletingMapId(mapId);
      await deleteDatabase(mapPath);
      
      // If the deleted map was selected, clear selection
      if (selectedLocation === mapId) {
        setSelectedLocation(null);
      }
      
      // Refresh the maps list
      setTimeout(() => {
        fetchMaps();
      }, 500);
    } catch (error) {
      console.error('Failed to delete map:', error);
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'error',
          title: 'Failed to Delete Map',
          message: `Could not delete map: ${mapName}`,
        },
      });
    } finally {
      setDeletingMapId(null);
      setShowDeleteConfirm(null);
    }
  };

  return (
    <div className="w-full h-full flex flex-col">
      {/* Header */}
      <div className="px-4 pt-4 pb-2 flex-shrink-0">
        <h1 className="text-2xl font-bold text-gray-800 dark:text-white">
          Maps Management
        </h1>
        <p className="text-sm text-gray-600 dark:text-gray-400">
          View and manage robot navigation maps
        </p>
      </div>

      {/* Main Content Area - Dynamic Layout */}
      <div className="flex-1 px-4 pb-4 min-h-0">
        <div className="h-full bg-white/5 dark:bg-botbot-darker/50 backdrop-blur-sm rounded-lg p-4">
          <div className={`h-full grid grid-cols-1 gap-4 transition-all duration-300 ${
            leftPanelCollapsed 
              ? 'lg:grid-cols-[70%_30%]'
              : 'lg:grid-cols-[20%_50%_30%] xl:grid-cols-[20%_50%_30%]'
          }`}>
            
            {/* New Left Column - Camera & Controls (20%) */}
            <div className={`relative flex-col min-h-0 h-full transition-all duration-300 ${
              leftPanelCollapsed ? 'hidden' : 'flex'
            }`}>
              {/* Collapse button */}
              <button
                onClick={() => setLeftPanelCollapsed(true)}
                className="absolute -right-4 top-1/2 transform -translate-y-1/2 z-10 bg-gray-100 dark:bg-botbot-darkest hover:bg-gray-200 dark:hover:bg-botbot-darker rounded-full p-2 shadow-lg transition-all duration-200"
                title="Hide panel"
              >
                <ChevronLeft className="w-6 h-6 text-gray-600 dark:text-gray-400" />
              </button>

              {/* Panel Content */}
              <div className="flex flex-col h-full space-y-4">
                {/* Camera Feed */}
                <div className="flex-shrink-0">
                  <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-2">Live Camera</h3>
                  <div className="bg-gray-50 dark:bg-botbot-darkest/50 rounded-lg overflow-hidden aspect-video">
                    <RosCameraImg 
                      cameraType="camera"
                      width={320}
                      height={180}
                    />
                  </div>
                </div>

                {/* Joystick Toggle */}
                <div className="flex-shrink-0">
                  <button
                    onClick={() => setShowJoysticks(!showJoysticks)}
                    className={`w-full flex items-center justify-center gap-2 px-3 py-2 rounded-lg transition-all duration-200 ${
                      showJoysticks
                        ? 'bg-blue-500 text-white hover:bg-blue-600'
                        : 'bg-gray-100 dark:bg-botbot-darkest text-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-botbot-darker'
                    }`}
                  >
                    <Gamepad2 className="w-4 h-4" />
                    <span className="text-sm font-medium">
                      {showJoysticks ? 'Hide' : 'Show'} Joysticks
                    </span>
                  </button>
                </div>

                {/* Robot Control Buttons */}
                <div className="flex-1 flex flex-col space-y-2">
                  <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">Robot Controls</h3>
                  
                  {/* Stand Up/Down */}
                  <div className="grid grid-cols-2 gap-2">
                    <button
                      onClick={() => robotActions.getUp.action()}
                      className="flex items-center justify-center gap-1 px-2 py-2 bg-green-500 hover:bg-green-600 text-white rounded-lg transition-all duration-200 text-sm font-medium"
                      title="Stand Up"
                    >
                      <ArrowUp className="w-4 h-4" />
                      <span className="hidden xl:inline">Get Up</span>
                    </button>
                    <button
                      onClick={() => robotActions.getDown.action()}
                      className="flex items-center justify-center gap-1 px-2 py-2 bg-orange-500 hover:bg-orange-600 text-white rounded-lg transition-all duration-200 text-sm font-medium"
                      title="Stand Down"
                    >
                      <ArrowDown className="w-4 h-4" />
                      <span className="hidden xl:inline">Get Down</span>
                    </button>
                  </div>

                  {/* Lock/Unlock */}
                  <div className="grid grid-cols-2 gap-2">
                    <button
                      onClick={() => robotActions.jointLock.action()}
                      className="flex items-center justify-center gap-1 px-2 py-2 bg-red-500 hover:bg-red-600 text-white rounded-lg transition-all duration-200 text-sm font-medium"
                      title="Lock Joints"
                    >
                      <Lock className="w-4 h-4" />
                      <span className="hidden xl:inline">Lock</span>
                    </button>
                    <button
                      onClick={() => robotActions.balanceStand.action()}
                      className="flex items-center justify-center gap-1 px-2 py-2 bg-blue-500 hover:bg-blue-600 text-white rounded-lg transition-all duration-200 text-sm font-medium"
                      title="Unlock Joints"
                    >
                      <Unlock className="w-4 h-4" />
                      <span className="hidden xl:inline">Unlock</span>
                    </button>
                  </div>

                </div>
              </div>
            </div>

            {/* Center Column - Map Visualizer (50%) */}
            <div className="relative flex flex-col min-h-0 h-full">
              {/* Expand button - only shows when panel is collapsed */}
              {leftPanelCollapsed && (
                <button
                  onClick={() => setLeftPanelCollapsed(false)}
                  className="absolute -left-4 top-1/2 transform -translate-y-1/2 z-10 bg-gray-100 dark:bg-botbot-darkest hover:bg-gray-200 dark:hover:bg-botbot-darker rounded-full p-2 shadow-lg transition-all duration-200"
                  title="Show control panel"
                >
                  <ChevronRight className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                </button>
              )}
              <div className="flex items-center justify-between mb-3 flex-shrink-0">
                <h2 className="text-lg font-semibold text-gray-800 dark:text-white flex items-center gap-2">
                  <MapPin className="w-5 h-5" />
                  Map Viewer
                </h2>
                <div className="text-sm text-gray-600 dark:text-gray-400">
                  {selectedLocation 
                    ? locations.find(l => l.id === selectedLocation)?.name 
                    : 'Current Map'}
                </div>
              </div>
              <div className="flex-1 min-h-0 bg-gray-50 dark:bg-botbot-darkest/50 rounded-lg overflow-hidden">
                <SimpleMapView
                  className="w-full h-full"
                  isMapping={isMapping}
                  isSettingHome={isSettingHome}
                  onHomeSet={(position) => {
                    // Toggle home setting mode when button is clicked
                    if (position.x === 0 && position.y === 0 && position.theta === 0) {
                      // Button clicked or cancelled
                      if (!isSettingHome) {
                        setIsSettingHome(true);
                        dispatch({
                          type: 'ADD_NOTIFICATION',
                          payload: {
                            type: 'info',
                            title: 'Set Home Position',
                            message: 'Click on the map to set home position, then drag to set orientation',
                          },
                        });
                      } else {
                        setIsSettingHome(false);
                      }
                    } else {
                      // Home position was set
                      setIsSettingHome(false);
                    }
                  }}
                />
              </div>
            </div>

            {/* Right Column - Controls and Location List (30%) */}
            <div className="flex flex-col min-h-0 h-full">
              
              {/* Mapping Control */}
              <div className="mb-4 flex-shrink-0">
                <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-3">
                  Mapping Control
                </h3>
                <div className="space-y-3">
                  {/* Map Name Input - Shows when Start Mapping is clicked */}
                  {showMapNameInput && !isMapping && (
                    <div className="animate-in fade-in slide-in-from-top-2 duration-300">
                      <input
                        type="text"
                        value={mapName}
                        onChange={(e) => setMapName(e.target.value)}
                        onKeyDown={(e) => {
                          if (e.key === 'Enter' && mapName.trim()) {
                            handleStartMapping();
                          }
                        }}
                        placeholder="Enter map name..."
                        className="w-full px-3 py-2 text-sm bg-white dark:bg-botbot-darker border border-gray-300 dark:border-gray-600 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500 dark:focus:ring-blue-400 text-gray-900 dark:text-white placeholder-gray-500 dark:placeholder-gray-400"
                        autoFocus
                      />
                    </div>
                  )}

                  {/* Mapping Timer - Shows when mapping is active */}
                  {isMapping && (
                    <div className="animate-in fade-in slide-in-from-top-2 duration-300 bg-blue-50 dark:bg-blue-900/20 border border-blue-200 dark:border-blue-800 rounded-lg px-3 py-2">
                      <div className="flex items-center justify-between">
                        <div className="flex items-center gap-2">
                          <Clock className="w-4 h-4 text-blue-600 dark:text-blue-400" />
                          <span className="text-sm font-medium text-blue-900 dark:text-blue-100">
                            Mapping: {mapName}
                          </span>
                        </div>
                        <span className="text-sm font-mono text-blue-700 dark:text-blue-300">
                          {formatTime(mappingTime)}
                        </span>
                      </div>
                    </div>
                  )}

                  {/* Main Mapping Button */}
                  <button
                    onClick={isMapping ? handleStopMapping : handleStartMapping}
                    className={`w-full flex items-center justify-center gap-2 px-4 py-2.5 rounded-lg transition-all duration-300 font-medium text-sm ${
                      isMapping
                        ? 'bg-red-500 hover:bg-red-600 text-white shadow-lg hover:shadow-xl transform hover:scale-[1.02]'
                        : showMapNameInput && mapName.trim()
                        ? 'bg-green-500 hover:bg-green-600 text-white shadow-lg hover:shadow-xl transform hover:scale-[1.02]'
                        : 'bg-blue-500 hover:bg-blue-600 text-white shadow-md hover:shadow-lg'
                    }`}
                  >
                    {isMapping ? (
                      <>
                        <Square className="w-4 h-4" />
                        <span>Stop Mapping</span>
                      </>
                    ) : showMapNameInput && mapName.trim() ? (
                      <>
                        <Play className="w-4 h-4" />
                        <span>Start Mapping</span>
                      </>
                    ) : (
                      <>
                        <Play className="w-4 h-4" />
                        <span>Start New Map</span>
                      </>
                    )}
                  </button>

                  {/* Cancel button - Shows when name input is visible */}
                  {showMapNameInput && !isMapping && (
                    <button
                      onClick={() => {
                        setShowMapNameInput(false);
                        setMapName('');
                      }}
                      className="w-full px-3 py-1.5 text-xs font-medium text-gray-600 dark:text-gray-400 hover:text-gray-800 dark:hover:text-gray-200 transition-colors duration-200"
                    >
                      Cancel
                    </button>
                  )}
                </div>
              </div>

              {/* Locations Table */}
              <div className="flex-1 min-h-0 flex flex-col">
                <div className="flex items-center justify-between mb-3">
                  <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300">
                    Available Maps
                  </h3>
                  <button
                    onClick={fetchMaps}
                    disabled={!isConnected || isLoading}
                    className="p-1.5 text-gray-600 dark:text-gray-400 hover:text-gray-800 dark:hover:text-gray-200 hover:bg-gray-100 dark:hover:bg-botbot-darkest/80 rounded transition-all duration-200 disabled:opacity-50 disabled:cursor-not-allowed"
                    title="Refresh maps list"
                  >
                    <RefreshCw className={`w-4 h-4 ${isLoading ? 'animate-spin' : ''}`} />
                  </button>
                </div>
                <div className="flex-1 bg-gray-50 dark:bg-botbot-darkest/50 rounded-lg p-3 overflow-hidden">
                  {!isConnected ? (
                    <div className="h-full flex items-center justify-center">
                      <p className="text-sm text-gray-500 dark:text-gray-400">
                        Connect to robot to view maps
                      </p>
                    </div>
                  ) : isLoading && locations.length === 0 ? (
                    <div className="h-full flex items-center justify-center">
                      <RefreshCw className="w-5 h-5 animate-spin text-gray-500 dark:text-gray-400" />
                    </div>
                  ) : locations.length === 0 ? (
                    <div className="h-full flex items-center justify-center">
                      <p className="text-sm text-gray-500 dark:text-gray-400">
                        No maps available
                      </p>
                    </div>
                  ) : (
                    <div className="h-full overflow-y-auto custom-scrollbar">
                      <div className="space-y-2">
                        {locations.map((location) => (
                          <div
                            key={location.id}
                            className={`p-3 rounded-lg border transition-all duration-200 ${
                              loadingMapId === location.id
                                ? 'border-yellow-500 bg-yellow-50 dark:bg-yellow-900/20 cursor-wait'
                                : selectedLocation === location.id
                                ? 'border-blue-500 bg-blue-50 dark:bg-blue-900/20 cursor-pointer'
                                : 'border-gray-200 dark:border-gray-700 bg-white dark:bg-botbot-darker/50 hover:border-gray-300 dark:hover:border-gray-600 cursor-pointer'
                            }`}
                            onClick={() => {
                              if (loadingMapId || selectedLocation === location.id) return;
                              handleLoadMap(location.id, location.path, location.name);
                            }}
                          >
                            <div className="flex items-center justify-between gap-2">
                              <div className="flex-1 min-w-0">
                                <h4 className="text-sm font-medium text-gray-900 dark:text-white truncate">
                                  {location.name}
                                </h4>
                                <p className="text-xs text-gray-500 dark:text-gray-400 mt-1">
                                  {location.path}
                                </p>
                              </div>
                              <div className="flex items-center gap-1 flex-shrink-0">
                                {deletingMapId === location.id ? (
                                  <div className="flex items-center gap-2 px-3 py-1.5">
                                    <RefreshCw className="w-3 h-3 animate-spin text-red-600 dark:text-red-400" />
                                    <span className="text-xs font-medium text-red-700 dark:text-red-300">
                                      Deleting...
                                    </span>
                                  </div>
                                ) : loadingMapId === location.id ? (
                                  <div className="flex items-center gap-2 px-3 py-1.5">
                                    <RefreshCw className="w-3 h-3 animate-spin text-yellow-600 dark:text-yellow-400" />
                                    <span className="text-xs font-medium text-yellow-700 dark:text-yellow-300">
                                      Loading...
                                    </span>
                                  </div>
                                ) : selectedLocation === location.id ? (
                                  <>
                                    <div className="flex items-center gap-1 px-3 py-1.5 bg-blue-500 text-white rounded-md">
                                      <CheckCircle className="w-3 h-3" />
                                      <span className="text-xs font-medium">Active</span>
                                    </div>
                                    <button
                                      className="p-1.5 text-gray-500 dark:text-gray-400 hover:text-red-600 dark:hover:text-red-400 hover:bg-red-50 dark:hover:bg-red-900/20 rounded transition-colors duration-200"
                                      onClick={(e) => {
                                        e.stopPropagation();
                                        setShowDeleteConfirm(location.id);
                                      }}
                                      title="Delete map"
                                    >
                                      <Trash2 className="w-3.5 h-3.5" />
                                    </button>
                                  </>
                                ) : (
                                  <>
                                    <button
                                      className="px-3 py-1.5 text-xs font-medium rounded-md bg-gray-100 dark:bg-botbot-darkest text-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-botbot-darkest/80 transition-colors duration-200"
                                      onClick={(e) => {
                                        e.stopPropagation();
                                        handleLoadMap(location.id, location.path, location.name);
                                      }}
                                    >
                                      Load Map
                                    </button>
                                    <button
                                      className="p-1.5 text-gray-500 dark:text-gray-400 hover:text-red-600 dark:hover:text-red-400 hover:bg-red-50 dark:hover:bg-red-900/20 rounded transition-colors duration-200"
                                      onClick={(e) => {
                                        e.stopPropagation();
                                        setShowDeleteConfirm(location.id);
                                      }}
                                      title="Delete map"
                                    >
                                      <Trash2 className="w-3.5 h-3.5" />
                                    </button>
                                  </>
                                )}
                              </div>
                            </div>

                            {/* Delete Confirmation Dialog */}
                            {showDeleteConfirm === location.id && (
                              <div className="mt-3 p-3 bg-red-50 dark:bg-red-900/20 border border-red-200 dark:border-red-800 rounded-lg">
                                <div className="flex items-start gap-2 mb-3">
                                  <AlertTriangle className="w-4 h-4 text-red-600 dark:text-red-400 mt-0.5" />
                                  <div className="flex-1">
                                    <p className="text-sm font-medium text-red-900 dark:text-red-100">
                                      Delete "{location.name}"?
                                    </p>
                                    <p className="text-xs text-red-700 dark:text-red-300 mt-1">
                                      This action cannot be undone.
                                    </p>
                                  </div>
                                </div>
                                <div className="flex gap-2">
                                  <button
                                    onClick={(e) => {
                                      e.stopPropagation();
                                      handleDeleteMap(location.id, location.path, location.name);
                                    }}
                                    className="flex-1 px-3 py-1.5 text-xs font-medium bg-red-600 hover:bg-red-700 text-white rounded-md transition-colors duration-200"
                                  >
                                    Delete
                                  </button>
                                  <button
                                    onClick={(e) => {
                                      e.stopPropagation();
                                      setShowDeleteConfirm(null);
                                    }}
                                    className="flex-1 px-3 py-1.5 text-xs font-medium bg-gray-200 dark:bg-botbot-darkest hover:bg-gray-300 dark:hover:bg-botbot-darker text-gray-700 dark:text-gray-300 rounded-md transition-colors duration-200"
                                  >
                                    Cancel
                                  </button>
                                </div>
                              </div>
                            )}
                          </div>
                        ))}
                      </div>
                    </div>
                  )}
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
      
      {/* Joysticks Overlay */}
      {showJoysticks && (
        <JoysticksWrapper enabled={showJoysticks} />
      )}
    </div>
  );
}