'use client';

import { createContext, useContext, useState, useEffect, ReactNode, useCallback, useMemo } from 'react';
import { useActiveMission } from './ActiveMissionContext';

export interface NavigationTarget {
  id: string;
  x: number;
  y: number;
  theta: number;
  type: 'mission' | 'single_goal';
  index?: number;        // For mission waypoints (0-based)
  isCurrent: boolean;    // Is this the current active target?
  isReached: boolean;    // Has this target been reached?
}

export interface SingleGoal {
  x: number;
  y: number;
  theta: number;
}

interface NavigationTargetsContextType {
  targets: NavigationTarget[];
  currentTarget: NavigationTarget | null;
  isNavigating: boolean;
  singleGoal: SingleGoal | null;
  setSingleGoal: (goal: SingleGoal | null) => void;
}

const NavigationTargetsContext = createContext<NavigationTargetsContextType | null>(null);

// Default values when no provider is present
const defaultValue: NavigationTargetsContextType = {
  targets: [],
  currentTarget: null,
  isNavigating: false,
  singleGoal: null,
  setSingleGoal: () => {},
};

export function NavigationTargetsProvider({ children }: { children: ReactNode }) {
  // Get mission waypoints from ActiveMissionContext
  const {
    missionWaypoints,
    currentWaypointIndex,
    isMissionActive,
  } = useActiveMission();

  // Track single "Go To" goals
  const [singleGoal, setSingleGoal] = useState<SingleGoal | null>(null);

  // Clear single goal when a mission becomes active
  useEffect(() => {
    if (isMissionActive) {
      setSingleGoal(null);
    }
  }, [isMissionActive]);

  // Compute unified targets list
  const targets = useMemo<NavigationTarget[]>(() => {
    const result: NavigationTarget[] = [];

    // Add mission waypoints if mission is active
    if (isMissionActive && missionWaypoints.length > 0) {
      missionWaypoints.forEach((wp, index) => {
        result.push({
          id: wp.id,
          x: wp.x,
          y: wp.y,
          theta: wp.theta,
          type: 'mission',
          index,
          isCurrent: index === currentWaypointIndex,
          isReached: index < currentWaypointIndex,
        });
      });
    }

    // Add single goal if set and no active mission
    if (singleGoal && !isMissionActive) {
      result.push({
        id: 'single-goal',
        x: singleGoal.x,
        y: singleGoal.y,
        theta: singleGoal.theta,
        type: 'single_goal',
        isCurrent: true,
        isReached: false,
      });
    }

    return result;
  }, [missionWaypoints, currentWaypointIndex, isMissionActive, singleGoal]);

  // Get the current target (the one being navigated to)
  const currentTarget = useMemo<NavigationTarget | null>(() => {
    return targets.find(t => t.isCurrent) || null;
  }, [targets]);

  // Determine if navigation is active
  const isNavigating = isMissionActive || singleGoal !== null;

  // Memoize the setSingleGoal callback
  const setSingleGoalCallback = useCallback((goal: SingleGoal | null) => {
    setSingleGoal(goal);
  }, []);

  const value = useMemo<NavigationTargetsContextType>(() => ({
    targets,
    currentTarget,
    isNavigating,
    singleGoal,
    setSingleGoal: setSingleGoalCallback,
  }), [targets, currentTarget, isNavigating, singleGoal, setSingleGoalCallback]);

  return (
    <NavigationTargetsContext.Provider value={value}>
      {children}
    </NavigationTargetsContext.Provider>
  );
}

export function useNavigationTargets() {
  const context = useContext(NavigationTargetsContext);
  // Return default values if no provider, allowing component to work everywhere
  return context ?? defaultValue;
}
