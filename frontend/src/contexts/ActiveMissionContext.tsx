'use client';

import { createContext, useContext, useState, ReactNode, useCallback } from 'react';

interface MissionWaypoint {
  id: string;
  x: number;
  y: number;
  theta: number;
}

interface ActiveMissionContextType {
  missionWaypoints: MissionWaypoint[];
  currentWaypointIndex: number;
  isMissionActive: boolean;
  setMissionWaypoints: (waypoints: MissionWaypoint[]) => void;
  setCurrentWaypointIndex: (index: number) => void;
  setIsMissionActive: (active: boolean) => void;
  clearMission: () => void;
}

const ActiveMissionContext = createContext<ActiveMissionContextType | null>(null);

// Default values when no provider is present (for components used outside cockpit)
const defaultValue: ActiveMissionContextType = {
  missionWaypoints: [],
  currentWaypointIndex: 0,
  isMissionActive: false,
  setMissionWaypoints: () => {},
  setCurrentWaypointIndex: () => {},
  setIsMissionActive: () => {},
  clearMission: () => {},
};

export function ActiveMissionProvider({ children }: { children: ReactNode }) {
  const [missionWaypoints, setMissionWaypoints] = useState<MissionWaypoint[]>([]);
  const [currentWaypointIndex, setCurrentWaypointIndex] = useState(0);
  const [isMissionActive, setIsMissionActive] = useState(false);

  const clearMission = useCallback(() => {
    setMissionWaypoints([]);
    setCurrentWaypointIndex(0);
    setIsMissionActive(false);
  }, []);

  return (
    <ActiveMissionContext.Provider value={{
      missionWaypoints,
      currentWaypointIndex,
      isMissionActive,
      setMissionWaypoints,
      setCurrentWaypointIndex,
      setIsMissionActive,
      clearMission,
    }}>
      {children}
    </ActiveMissionContext.Provider>
  );
}

export function useActiveMission() {
  const context = useContext(ActiveMissionContext);
  // Return default values if no provider, allowing component to work everywhere
  return context ?? defaultValue;
}
