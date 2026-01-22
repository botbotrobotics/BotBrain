'use client';

import { useState, useEffect, useCallback } from 'react';
import { useRosMappingServices } from '@/hooks/ros/useRosMappingServices';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';

interface MapMissionCompatibility {
  currentMapName: string | null;
  missionMapName: string | null;
  isCompatible: boolean;
  isLoading: boolean;
  isRobotConnected: boolean;
  switchToMissionMap: () => Promise<boolean>;
  refreshCurrentMap: () => Promise<void>;
}

// Helper to normalize map names for comparison
function normalizeMapName(name: string): string {
  // Remove path prefix if present, remove .db extension
  const fileName = name.split('/').pop() || name;
  return fileName
    .replace(/\.db$/i, '')
    .toLowerCase()
    .trim();
}

export function useMapMissionCompatibility(missionMapName: string | null): MapMissionCompatibility {
  const { connectionStatus } = useRobotConnection();
  const { getCurrentDatabase, loadDatabase, isConnected } = useRosMappingServices();
  const [currentMapName, setCurrentMapName] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  const isRobotConnected = connectionStatus === 'connected';

  const refreshCurrentMap = useCallback(async () => {
    if (!isConnected) {
      setCurrentMapName(null);
      setIsLoading(false);
      return;
    }

    try {
      setIsLoading(true);
      const mapName = await getCurrentDatabase();
      setCurrentMapName(mapName);
    } catch (error) {
      console.error('Failed to get current map:', error);
      setCurrentMapName(null);
    } finally {
      setIsLoading(false);
    }
  }, [isConnected, getCurrentDatabase]);

  // Fetch current map on mount and when connection changes
  useEffect(() => {
    refreshCurrentMap();
  }, [refreshCurrentMap]);

  // Check compatibility - compatible if:
  // - No mission map specified (legacy mission)
  // - No current map loaded (can't verify)
  // - Robot not connected (can't verify)
  // - Map names match (normalized comparison)
  const isCompatible =
    !missionMapName ||
    !currentMapName ||
    !isRobotConnected ||
    normalizeMapName(missionMapName) === normalizeMapName(currentMapName);

  const switchToMissionMap = useCallback(async (): Promise<boolean> => {
    if (!missionMapName || !isConnected) return false;

    try {
      await loadDatabase(missionMapName, false);
      await refreshCurrentMap();
      return true;
    } catch (error) {
      console.error('Failed to switch map:', error);
      return false;
    }
  }, [missionMapName, isConnected, loadDatabase, refreshCurrentMap]);

  return {
    currentMapName,
    missionMapName,
    isCompatible,
    isLoading,
    isRobotConnected,
    switchToMissionMap,
    refreshCurrentMap,
  };
}
