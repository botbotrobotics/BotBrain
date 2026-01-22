'use client';

import { useState, useCallback } from 'react';
import * as ROSLIB from 'roslib';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';

// Channel constants for HikvisionParameters service
export const CAMERA_CHANNELS = {
  RGB: 1,
  IR: 2, // Thermal/infrared camera
} as const;

// Command constants for HikvisionParameters service
export const THERMAL_COMMANDS = {
  SET_MANUAL_CORRECTION: 0,
  GET_PALETTE: 1,
  SET_PALETTE: 2,
} as const;

// Thermal palette values
export const THERMAL_PALETTES = {
  WHITEHOT: 0,
  BLACKHOT: 1,
  FUSION1: 2,
  RAINBOW: 3,
  FUSION2: 4,
  IRONBOW1: 5,
  IRONBOW2: 6,
  SEPIA: 7,
  COLOR1: 8,
  COLOR2: 9,
  ICEFIRE: 10,
  RAIN: 11,
  REDHOT: 12,
  GREENHOT: 13,
  DEEPBLUE: 14,
  WINTER: 15,
  SUMMER: 16,
} as const;

export type ThermalPaletteName = keyof typeof THERMAL_PALETTES;

interface HikvisionParametersResponse {
  result: number;
  success: boolean;
}

interface UseThermalCameraReturn {
  setPalette: (paletteValue: number) => Promise<boolean>;
  triggerFlatFieldCorrection: () => Promise<boolean>;
  isLoading: boolean;
  error: string | null;
  clearError: () => void;
}

export function useThermalCamera(): UseThermalCameraReturn {
  const { connection } = useRobotConnection();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const callHikvisionService = useCallback(
    async (command: number, value: number): Promise<boolean> => {
      console.log('[ThermalCamera] callHikvisionService called with command:', command, 'value:', value);
      console.log('[ThermalCamera] connection.ros:', connection.ros);
      console.log('[ThermalCamera] connection.online:', connection.online);

      if (!connection.ros || !connection.online) {
        console.error('[ThermalCamera] Not connected to robot');
        setError('Not connected to robot');
        return false;
      }

      setIsLoading(true);
      setError(null);

      return new Promise((resolve) => {
        try {
          const service = new ROSLIB.Service({
            ros: connection.ros!,
            name: '/camcam_camera/parameters',
            serviceType: 'bot_custom_interfaces/srv/HikvisionParameters',
          });

          const request = {
            channel: CAMERA_CHANNELS.IR, // IR/Thermal camera is channel 2
            command: command,
            value: value,
          };

          console.log('[ThermalCamera] Calling service with request:', { channel: CAMERA_CHANNELS.IR, command, value });

          const timeoutId = setTimeout(() => {
            console.error('[ThermalCamera] Service timeout');
            setError('Service timeout');
            setIsLoading(false);
            resolve(false);
          }, 10000);

          service.callService(
            request,
            (response: any) => {
              clearTimeout(timeoutId);
              setIsLoading(false);
              console.log('[ThermalCamera] Service response (full):', JSON.stringify(response));

              // Check success: either explicit success field, or result === 0 means success
              const isSuccess = response.success === true || response.result === 0;

              if (isSuccess) {
                console.log('[ThermalCamera] Command succeeded');
                resolve(true);
              } else {
                console.error('[ThermalCamera] Command failed with result:', response.result);
                setError(`Camera command failed with result: ${response.result}`);
                resolve(false);
              }
            },
            (err: string) => {
              clearTimeout(timeoutId);
              setIsLoading(false);
              console.error('[ThermalCamera] Service call error:', err);
              setError(`Service call failed: ${err}`);
              resolve(false);
            }
          );
        } catch (err) {
          console.error('[ThermalCamera] Exception:', err);
          setIsLoading(false);
          setError('Failed to create service');
          resolve(false);
        }
      });
    },
    [connection]
  );

  const setPalette = useCallback(
    async (paletteValue: number): Promise<boolean> => {
      return callHikvisionService(THERMAL_COMMANDS.SET_PALETTE, paletteValue);
    },
    [callHikvisionService]
  );

  const triggerFlatFieldCorrection = useCallback(async (): Promise<boolean> => {
    return callHikvisionService(THERMAL_COMMANDS.SET_MANUAL_CORRECTION, 0);
  }, [callHikvisionService]);

  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    setPalette,
    triggerFlatFieldCorrection,
    isLoading,
    error,
    clearError,
  };
}
