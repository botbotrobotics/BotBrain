'use client';

import { useState, useEffect, useRef } from 'react';
import { Widget } from './Widget';
import SimpleMapView from '../simple-map-view';
import { 
  MapPin, 
  Play, 
  Square, 
  Clock, 
  RefreshCw, 
  CheckCircle,
  ChevronDown,
  ChevronUp,
  Loader2
} from 'lucide-react';
import { useRosMappingServices } from '@/hooks/ros/useRosMappingServices';
import { useNotifications } from '@/contexts/NotificationsContext';

interface MapLocation {
  id: string;
  name: string;
  path: string;
  lastUpdated: string;
  size: string;
}

interface MapsManagementWidgetProps {
  id: string;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  props?: Record<string, any>;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
}

export function MapsManagementWidget({
  id,
  initialPosition,
  initialSize = { width: 450, height: 500 },
  props,
  onRemove,
  onStartDrag,
  onEndDrag,
}: MapsManagementWidgetProps) {
  const [selectedLocation, setSelectedLocation] = useState<string | null>(null);
  const [isMapping, setIsMapping] = useState(false);
  const [showMapNameInput, setShowMapNameInput] = useState(false);
  const [mapName, setMapName] = useState('');
  const [mappingTime, setMappingTime] = useState(0);
  const [locations, setLocations] = useState<MapLocation[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [loadingMapId, setLoadingMapId] = useState<string | null>(null);
  const [showMapView, setShowMapView] = useState(true);
  const [showMapsList, setShowMapsList] = useState(true);
  const intervalRef = useRef<NodeJS.Timeout | null>(null);
  
  const { dispatch } = useNotifications();
  
  const {
    isConnected,
    listDatabaseFiles,
    loadDatabase,
    setMappingMode,
    setLocalizationMode,
    saveDatabase,
    formatDatabaseInfo,
  } = useRosMappingServices();

  useEffect(() => {
    if (isConnected) {
      fetchMaps();
    }
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
      const dbFiles = await listDatabaseFiles();
      const formattedMaps = formatDatabaseInfo(dbFiles);
      const mapsWithIds = formattedMaps.map((map, index) => ({
        id: index.toString(),
        name: map.name,
        path: map.path,
        lastUpdated: map.lastModified || 'Unknown',
        size: map.size || 'Unknown',
      }));
      setLocations(mapsWithIds);
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
      await loadDatabase(mapPath, false);
      await setLocalizationMode();
      setSelectedLocation(mapId);
      
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'success',
          title: 'Map Loaded',
          message: `Successfully loaded: ${mapName}`,
        },
      });
    } catch (error) {
      console.error('Failed to load map:', error);
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'error',
          title: 'Failed to Load Map',
          message: `Could not load: ${mapName}`,
        },
      });
    } finally {
      setLoadingMapId(null);
    }
  };

  return (
    <Widget
      id={id}
      title={
        <div className="flex items-center gap-2">
          <MapPin className="w-4 h-4" />
          <span>Maps Management</span>
        </div>
      }
      initialPosition={initialPosition}
      initialSize={initialSize}
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      minWidth={350}
      minHeight={400}
    >
      <div className="h-full flex flex-col -m-4">
        {/* Map View Section */}
        {showMapView && (
          <div className="flex-1 min-h-0">
            <div className="px-4 pt-2 pb-1 bg-gray-50 dark:bg-botbot-darker/50 border-b border-gray-200 dark:border-gray-700">
              <button
                onClick={() => setShowMapView(!showMapView)}
                className="flex items-center justify-between w-full text-left"
              >
                <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300">
                  Map View {selectedLocation && 
                    <span className="text-xs text-gray-500 dark:text-gray-400 ml-1">
                      ({locations.find(l => l.id === selectedLocation)?.name})
                    </span>
                  }
                </h3>
                <ChevronDown className="w-4 h-4 text-gray-500 dark:text-gray-400" />
              </button>
            </div>
            <div className="h-[200px] bg-gray-50 dark:bg-botbot-darkest/30">
              <SimpleMapView className="w-full h-full" isMapping={isMapping} />
            </div>
          </div>
        )}

        {/* Mapping Control Section */}
        <div className="px-4 py-3 border-b border-gray-200 dark:border-gray-700">
          <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-2">
            Mapping Control
          </h3>
          
          <div className="space-y-2">
            {/* Map Name Input */}
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
                  className="w-full px-3 py-1.5 text-sm bg-white dark:bg-botbot-darker border border-gray-300 dark:border-gray-600 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500 dark:focus:ring-blue-400 text-gray-900 dark:text-white placeholder-gray-500 dark:placeholder-gray-400"
                  autoFocus
                />
              </div>
            )}

            {/* Mapping Timer */}
            {isMapping && (
              <div className="animate-in fade-in slide-in-from-top-2 duration-300 bg-blue-50 dark:bg-blue-900/20 border border-blue-200 dark:border-blue-800 rounded-lg px-2 py-1.5">
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-2">
                    <Clock className="w-3 h-3 text-blue-600 dark:text-blue-400" />
                    <span className="text-xs font-medium text-blue-900 dark:text-blue-100">
                      {mapName}
                    </span>
                  </div>
                  <span className="text-xs font-mono text-blue-700 dark:text-blue-300">
                    {formatTime(mappingTime)}
                  </span>
                </div>
              </div>
            )}

            {/* Mapping Button */}
            <button
              onClick={isMapping ? handleStopMapping : handleStartMapping}
              className={`w-full flex items-center justify-center gap-2 px-3 py-2 rounded-lg transition-all duration-300 font-medium text-sm ${
                isMapping
                  ? 'bg-red-500 hover:bg-red-600 text-white'
                  : showMapNameInput && mapName.trim()
                  ? 'bg-green-500 hover:bg-green-600 text-white'
                  : 'bg-blue-500 hover:bg-blue-600 text-white'
              }`}
            >
              {isMapping ? (
                <>
                  <Square className="w-3 h-3" />
                  <span>Stop Mapping</span>
                </>
              ) : showMapNameInput && mapName.trim() ? (
                <>
                  <Play className="w-3 h-3" />
                  <span>Start Mapping</span>
                </>
              ) : (
                <>
                  <Play className="w-3 h-3" />
                  <span>Start New Map</span>
                </>
              )}
            </button>

            {/* Cancel button */}
            {showMapNameInput && !isMapping && (
              <button
                onClick={() => {
                  setShowMapNameInput(false);
                  setMapName('');
                }}
                className="w-full px-2 py-1 text-xs font-medium text-gray-600 dark:text-gray-400 hover:text-gray-800 dark:hover:text-gray-200 transition-colors duration-200"
              >
                Cancel
              </button>
            )}
          </div>
        </div>

        {/* Available Maps Section */}
        <div className="flex-1 min-h-0 flex flex-col px-4 pb-4">
          <div className="flex items-center justify-between py-2">
            <button
              onClick={() => setShowMapsList(!showMapsList)}
              className="flex items-center gap-1 text-sm font-medium text-gray-700 dark:text-gray-300"
            >
              Available Maps
              {showMapsList ? (
                <ChevronUp className="w-4 h-4 text-gray-500 dark:text-gray-400" />
              ) : (
                <ChevronDown className="w-4 h-4 text-gray-500 dark:text-gray-400" />
              )}
            </button>
            <button
              onClick={fetchMaps}
              disabled={!isConnected || isLoading}
              className="p-1 text-gray-600 dark:text-gray-400 hover:text-gray-800 dark:hover:text-gray-200 hover:bg-gray-100 dark:hover:bg-botbot-darkest/80 rounded transition-all duration-200 disabled:opacity-50 disabled:cursor-not-allowed"
              title="Refresh maps list"
            >
              <RefreshCw className={`w-3 h-3 ${isLoading ? 'animate-spin' : ''}`} />
            </button>
          </div>

          {showMapsList && (
            <div className="flex-1 bg-gray-50 dark:bg-botbot-darkest/30 rounded-lg p-2 overflow-hidden">
              {!isConnected ? (
                <div className="h-full flex items-center justify-center">
                  <p className="text-xs text-gray-500 dark:text-gray-400">
                    Connect to robot to view maps
                  </p>
                </div>
              ) : isLoading && locations.length === 0 ? (
                <div className="h-full flex items-center justify-center">
                  <RefreshCw className="w-4 h-4 animate-spin text-gray-500 dark:text-gray-400" />
                </div>
              ) : locations.length === 0 ? (
                <div className="h-full flex items-center justify-center">
                  <p className="text-xs text-gray-500 dark:text-gray-400">
                    No maps available
                  </p>
                </div>
              ) : (
                <div className="h-full overflow-y-auto custom-scrollbar">
                  <div className="space-y-1.5">
                    {locations.map((location) => (
                      <div
                        key={location.id}
                        className={`p-2 rounded-lg border transition-all duration-200 cursor-pointer ${
                          loadingMapId === location.id
                            ? 'border-yellow-500 bg-yellow-50 dark:bg-yellow-900/20'
                            : selectedLocation === location.id
                            ? 'border-blue-500 bg-blue-50 dark:bg-blue-900/20'
                            : 'border-gray-200 dark:border-gray-700 bg-white dark:bg-botbot-darker/50 hover:border-gray-300 dark:hover:border-gray-600'
                        }`}
                        onClick={() => {
                          if (loadingMapId || selectedLocation === location.id) return;
                          handleLoadMap(location.id, location.path, location.name);
                        }}
                      >
                        <div className="flex items-center justify-between gap-2">
                          <div className="flex-1 min-w-0">
                            <h4 className="text-xs font-medium text-gray-900 dark:text-white truncate">
                              {location.name}
                            </h4>
                            <p className="text-xs text-gray-500 dark:text-gray-400 mt-0.5 truncate">
                              {location.size}
                            </p>
                          </div>
                          <div className="flex-shrink-0">
                            {loadingMapId === location.id ? (
                              <Loader2 className="w-3 h-3 animate-spin text-yellow-600 dark:text-yellow-400" />
                            ) : selectedLocation === location.id ? (
                              <CheckCircle className="w-3 h-3 text-blue-500" />
                            ) : (
                              <button
                                className="px-2 py-0.5 text-xs font-medium rounded bg-gray-100 dark:bg-botbot-darkest text-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-botbot-darkest/80 transition-colors duration-200"
                                onClick={(e) => {
                                  e.stopPropagation();
                                  handleLoadMap(location.id, location.path, location.name);
                                }}
                              >
                                Load
                              </button>
                            )}
                          </div>
                        </div>
                      </div>
                    ))}
                  </div>
                </div>
              )}
            </div>
          )}
        </div>
      </div>
    </Widget>
  );
}