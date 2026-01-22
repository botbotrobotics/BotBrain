'use client';

import React, { createContext, useContext, useState, useEffect, useCallback, ReactNode } from 'react';
import { useSupabase } from './SupabaseProvider';
import { SpeedMode, SPEED_MODE_MULTIPLIERS, DEFAULT_SPEED_MODE } from '@/types/speed-mode';

interface SpeedModeContextType {
  speedMode: SpeedMode;
  speedMultiplier: number;
  setSpeedMode: (mode: SpeedMode) => void;
  isLoading: boolean;
  reloadSpeedMode: () => Promise<void>;
}

const SpeedModeContext = createContext<SpeedModeContextType | undefined>(undefined);

export function SpeedModeProvider({ children }: { children: ReactNode }) {
  const { user, supabase } = useSupabase();
  const [speedMode, setSpeedModeState] = useState<SpeedMode>(DEFAULT_SPEED_MODE);
  const [isLoading, setIsLoading] = useState(true);

  const loadSpeedMode = useCallback(async () => {
    if (!supabase || !user) {
      setSpeedModeState(DEFAULT_SPEED_MODE);
      setIsLoading(false);
      return;
    }

    try {
      const { data: profile, error } = await supabase
        .from('user_profiles')
        .select('speed_mode')
        .eq('user_id', user.id)
        .single();

      if (!error && profile?.speed_mode) {
        // Validate the value is a valid SpeedMode
        if (['beginner', 'normal', 'insane'].includes(profile.speed_mode)) {
          setSpeedModeState(profile.speed_mode as SpeedMode);
        }
      }
    } catch (error) {
      console.error('Error loading speed mode:', error);
    } finally {
      setIsLoading(false);
    }
  }, [supabase, user]);

  // Load speed mode from user profile
  useEffect(() => {
    if (user && supabase) {
      loadSpeedMode();
    } else {
      // Reset to default when user logs out
      setSpeedModeState(DEFAULT_SPEED_MODE);
      setIsLoading(false);
    }
  }, [user, supabase, loadSpeedMode]);

  const setSpeedMode = (mode: SpeedMode) => {
    setSpeedModeState(mode);
    // Note: Actual saving happens in the profile page's save handler
    // This just updates local state for immediate UI feedback
  };

  const speedMultiplier = SPEED_MODE_MULTIPLIERS[speedMode];

  return (
    <SpeedModeContext.Provider value={{
      speedMode,
      speedMultiplier,
      setSpeedMode,
      isLoading,
      reloadSpeedMode: loadSpeedMode
    }}>
      {children}
    </SpeedModeContext.Provider>
  );
}

export const useSpeedMode = () => {
  const context = useContext(SpeedModeContext);
  if (context === undefined) {
    throw new Error('useSpeedMode must be used within a SpeedModeProvider');
  }
  return context;
};
