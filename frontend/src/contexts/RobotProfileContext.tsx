'use client';

import React, { createContext, useContext, useEffect, useState } from 'react';
import { RobotProfile, getRobotProfile } from '@/config/robot-profiles';
import { useRobotConnection } from './RobotConnectionContext';

interface RobotProfileContextType {
  currentProfile: RobotProfile | null;
  isLoading: boolean;
  error: string | null;
  refreshProfile: () => void;
}

const RobotProfileContext = createContext<RobotProfileContextType | undefined>(undefined);

export function RobotProfileProvider({ children }: { children: React.ReactNode }) {
  const { connection } = useRobotConnection();
  const [currentProfile, setCurrentProfile] = useState<RobotProfile | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const loadProfile = () => {
    if (!connection.connectedRobot) {
      setCurrentProfile(null);
      setError(null);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const robotType = connection.connectedRobot.type;
      const profile = getRobotProfile(robotType);
      
      if (profile) {
        setCurrentProfile(profile);
      } else {
        // Default to Go2-R1 if profile not found
        const defaultProfile = getRobotProfile('Go2-R1');
        setCurrentProfile(defaultProfile || null);
        setError(`Profile not found for robot type: ${robotType}. Using default profile.`);
      }
    } catch (err) {
      setError(`Failed to load robot profile: ${err}`);
      setCurrentProfile(null);
    } finally {
      setIsLoading(false);
    }
  };

  useEffect(() => {
    loadProfile();
  }, [connection.connectedRobot]);

  const refreshProfile = () => {
    loadProfile();
  };

  return (
    <RobotProfileContext.Provider 
      value={{ 
        currentProfile, 
        isLoading, 
        error,
        refreshProfile 
      }}
    >
      {children}
    </RobotProfileContext.Provider>
  );
}

export function useRobotProfile() {
  const context = useContext(RobotProfileContext);
  if (context === undefined) {
    throw new Error('useRobotProfile must be used within a RobotProfileProvider');
  }
  return context;
}