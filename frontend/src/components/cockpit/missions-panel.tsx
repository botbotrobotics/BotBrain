'use client';

import { useState, useEffect, useCallback } from 'react';
import Container from '@/components/ui/container';
import { Route, Play, Square, RefreshCw, Clock, MapPin } from 'lucide-react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useRosMappingServices } from '@/hooks/ros/useRosMappingServices';
import useFollowWaypoints from '@/hooks/ros/useFollowWaypoints';
import { missionsService, MissionWithWaypoints } from '@/services/missions';
import { cn } from '@/utils/cn';
import { useActiveMission } from '@/contexts/ActiveMissionContext';

// Helper to normalize map names for comparison
function normalizeMapName(name: string): string {
  const fileName = name.split('/').pop() || name;
  return fileName.replace(/\.db$/i, '').toLowerCase().trim();
}

// Format elapsed time as mm:ss
function formatElapsedTime(seconds: number): string {
  const mins = Math.floor(seconds / 60);
  const secs = seconds % 60;
  return `${mins}:${secs.toString().padStart(2, '0')}`;
}

interface MissionItemProps {
  mission: MissionWithWaypoints;
  isSelected: boolean;
  isRunning: boolean;
  progress: { currentWaypointIndex: number; totalWaypoints: number; elapsedTime: number } | null;
  onSelect: () => void;
  disabled?: boolean;
}

function MissionItem({ mission, isSelected, isRunning, progress, onSelect, disabled }: MissionItemProps) {
  const waypointCount = mission.waypoints?.length || 0;

  return (
    <button
      onClick={onSelect}
      disabled={disabled}
      className={cn(
        "w-full text-left p-2 rounded-lg transition-all duration-200",
        "border border-transparent",
        "hover:bg-gray-50 dark:hover:bg-botbot-darker",
        isSelected && "bg-primary/10 border-primary dark:bg-primary/20",
        isRunning && "bg-green-50 border-green-500 dark:bg-green-900/20 dark:border-green-500",
        disabled && "opacity-50 cursor-not-allowed"
      )}
    >
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2 min-w-0 flex-1">
          <div className={cn(
            "w-3 h-3 rounded-full border-2 flex-shrink-0",
            isSelected || isRunning
              ? "border-primary bg-primary dark:border-botbot-accent dark:bg-botbot-accent"
              : "border-gray-300 dark:border-gray-600"
          )} />
          <span className="font-medium truncate text-sm">{mission.name}</span>
          {isRunning && (
            <span className="text-xs px-1.5 py-0.5 rounded bg-green-500 text-white flex-shrink-0">
              Running
            </span>
          )}
        </div>
        <span className="text-xs text-gray-500 dark:text-gray-400 flex-shrink-0 ml-2">
          {waypointCount} {waypointCount === 1 ? 'waypoint' : 'waypoints'}
        </span>
      </div>

      {/* Progress display when running */}
      {isRunning && progress && (
        <div className="mt-1.5 pl-5 flex items-center gap-3 text-xs text-gray-600 dark:text-gray-400">
          <span className="flex items-center gap-1">
            <MapPin className="w-3 h-3" />
            {progress.currentWaypointIndex + 1}/{progress.totalWaypoints}
          </span>
          <span className="flex items-center gap-1">
            <Clock className="w-3 h-3" />
            {formatElapsedTime(progress.elapsedTime)}
          </span>
        </div>
      )}
    </button>
  );
}

export default function MissionsPanel() {
  const { connectionStatus } = useRobotConnection();
  const { getCurrentDatabase, isConnected: rosConnected } = useRosMappingServices();
  const {
    startNavigation,
    cancelNavigation,
    status: navStatus,
    progress: navProgress,
    isConnected: navConnected
  } = useFollowWaypoints();
  const {
    setMissionWaypoints,
    setCurrentWaypointIndex,
    setIsMissionActive,
    clearMission
  } = useActiveMission();

  const [missions, setMissions] = useState<MissionWithWaypoints[]>([]);
  const [currentMapName, setCurrentMapName] = useState<string | null>(null);
  const [selectedMissionId, setSelectedMissionId] = useState<string | null>(null);
  const [activeMissionId, setActiveMissionId] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [isRefreshing, setIsRefreshing] = useState(false);

  const isRobotConnected = connectionStatus === 'connected';
  const isNavigating = navStatus === 'navigating';

  // Fetch missions from database
  const fetchMissions = useCallback(async () => {
    try {
      const data = await missionsService.getMissions();
      setMissions(data);

      // Check if any mission is currently active
      const active = data.find(m => m.is_active);
      if (active) {
        setActiveMissionId(active.id);
        setSelectedMissionId(active.id);
      }
    } catch (error) {
      console.error('Failed to fetch missions:', error);
    }
  }, []);

  // Fetch current map name
  const fetchCurrentMap = useCallback(async () => {
    if (!rosConnected) {
      setCurrentMapName(null);
      return;
    }
    try {
      const mapName = await getCurrentDatabase();
      setCurrentMapName(mapName);
    } catch (error) {
      console.error('Failed to get current map:', error);
      setCurrentMapName(null);
    }
  }, [rosConnected, getCurrentDatabase]);

  // Initial load
  useEffect(() => {
    const load = async () => {
      setIsLoading(true);
      await Promise.all([fetchMissions(), fetchCurrentMap()]);
      setIsLoading(false);
    };
    load();
  }, [fetchMissions, fetchCurrentMap]);

  // Subscribe to mission changes
  useEffect(() => {
    const channel = missionsService.subscribeToMissionChanges(() => {
      fetchMissions();
    });
    return () => {
      channel.unsubscribe();
    };
  }, [fetchMissions]);

  // Refresh current map when connection changes
  useEffect(() => {
    if (rosConnected) {
      fetchCurrentMap();
    }
  }, [rosConnected, fetchCurrentMap]);

  // Sync current waypoint index to context during navigation
  useEffect(() => {
    if (navStatus === 'navigating') {
      setCurrentWaypointIndex(navProgress.currentWaypointIndex);
    }
  }, [navProgress.currentWaypointIndex, navStatus, setCurrentWaypointIndex]);

  // Handle navigation completion
  useEffect(() => {
    if (navStatus === 'completed' || navStatus === 'cancelled' || navStatus === 'failed') {
      // Clear mission waypoints from map
      clearMission();
      // Deactivate mission in database
      if (activeMissionId) {
        missionsService.setActiveMission(null).catch(console.error);
        setActiveMissionId(null);
      }
    }
  }, [navStatus, activeMissionId, clearMission]);

  // Filter missions by current map
  const filteredMissions = missions.filter(mission => {
    if (!currentMapName || !mission.map_name) return false;
    return normalizeMapName(mission.map_name) === normalizeMapName(currentMapName);
  });

  // Get selected mission
  const selectedMission = filteredMissions.find(m => m.id === selectedMissionId);

  // Handle refresh
  const handleRefresh = async () => {
    setIsRefreshing(true);
    await Promise.all([fetchMissions(), fetchCurrentMap()]);
    setIsRefreshing(false);
  };

  // Handle start/stop
  const handleStartStop = async () => {
    if (isNavigating && activeMissionId) {
      // Stop navigation
      cancelNavigation();
      clearMission();
      await missionsService.setActiveMission(null);
      setActiveMissionId(null);
    } else if (selectedMission && selectedMission.waypoints.length > 0) {
      // Start navigation
      await missionsService.resetWaypointsStatus(selectedMission.id);
      await missionsService.setActiveMission(selectedMission.id);
      setActiveMissionId(selectedMission.id);

      const waypoints = selectedMission.waypoints.map(wp => ({
        x: wp.x,
        y: wp.y,
        theta: wp.theta,
        id: wp.id,
      }));

      // Publish mission waypoints to context for map display
      setMissionWaypoints(waypoints);
      setIsMissionActive(true);
      setCurrentWaypointIndex(0);

      startNavigation(waypoints);
    }
  };

  // Determine button state
  const canStart = selectedMission &&
    selectedMission.waypoints.length > 0 &&
    isRobotConnected &&
    navConnected &&
    !isNavigating;
  const canStop = isNavigating && activeMissionId;

  return (
    <Container
      title={
        <div className="flex items-center">
          <Route className="mr-2 w-5 h-5" />
          <span>Missions</span>
        </div>
      }
      className="h-full !flex !flex-col"
      customContentClasses="!p-2 flex-1 min-h-0 flex flex-col overflow-hidden"
      customActions={
        <button
          onClick={handleRefresh}
          disabled={isRefreshing}
          className={cn(
            "p-1.5 rounded-md transition-all duration-200",
            "hover:bg-gray-100 dark:hover:bg-botbot-darker",
            isRefreshing && "animate-spin"
          )}
          title="Refresh missions"
        >
          <RefreshCw className="w-4 h-4" />
        </button>
      }
    >
      {/* Mission list - scrolls only when needed */}
      <div className="flex-1 overflow-y-auto min-h-0 space-y-1">
          {isLoading ? (
            <div className="flex items-center justify-center py-4 text-sm text-gray-500 dark:text-gray-400">
              Loading...
            </div>
          ) : !isRobotConnected ? (
            <div className="flex flex-col items-center justify-center py-4 text-sm text-gray-500 dark:text-gray-400 text-center px-2">
              <Route className="w-6 h-6 mb-1 opacity-50" />
              <span>Connect to robot to view missions</span>
            </div>
          ) : !currentMapName ? (
            <div className="flex flex-col items-center justify-center py-4 text-sm text-gray-500 dark:text-gray-400 text-center px-2">
              <Route className="w-6 h-6 mb-1 opacity-50" />
              <span>Load a map to see missions</span>
            </div>
          ) : filteredMissions.length === 0 ? (
            <div className="flex flex-col items-center justify-center py-4 text-sm text-gray-500 dark:text-gray-400 text-center px-2">
              <Route className="w-6 h-6 mb-1 opacity-50" />
              <span>No missions for current map</span>
            </div>
          ) : (
            filteredMissions.map(mission => (
              <MissionItem
                key={mission.id}
                mission={mission}
                isSelected={selectedMissionId === mission.id}
                isRunning={activeMissionId === mission.id && isNavigating}
                progress={activeMissionId === mission.id ? {
                  currentWaypointIndex: navProgress.currentWaypointIndex,
                  totalWaypoints: navProgress.totalWaypoints,
                  elapsedTime: navProgress.elapsedTime,
                } : null}
                onSelect={() => !isNavigating && setSelectedMissionId(mission.id)}
                disabled={isNavigating && activeMissionId !== mission.id}
              />
            ))
          )}
        </div>

      {/* Start/Stop button - always visible at bottom */}
      <div className="flex-shrink-0 pt-2 border-t border-gray-100 dark:border-gray-800">
        <button
          onClick={handleStartStop}
          disabled={!canStart && !canStop}
          className={cn(
            "w-full py-2 px-4 rounded-lg font-medium text-sm transition-all duration-200",
            "flex items-center justify-center gap-2",
            canStop
              ? "bg-red-500 hover:bg-red-600 text-white"
              : canStart
                ? "bg-primary hover:bg-primary/90 text-white dark:bg-botbot-accent dark:hover:bg-botbot-accent/90"
                : "bg-gray-200 text-gray-400 cursor-not-allowed dark:bg-gray-700 dark:text-gray-500"
          )}
        >
          {canStop ? (
            <>
              <Square className="w-4 h-4" />
              Stop Mission
            </>
          ) : (
            <>
              <Play className="w-4 h-4" />
              Start Mission
            </>
          )}
        </button>
      </div>
    </Container>
  );
}
