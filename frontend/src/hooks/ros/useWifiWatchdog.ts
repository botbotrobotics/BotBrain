'use client';

import { useState, useCallback, useEffect } from 'react';
import * as ROSLIB from 'roslib';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { WifiWatchdogParams } from '@/types/WifiControl';

export function useWifiWatchdog() {
  const { connection } = useRobotConnection();
  const [watchdogParams, setWatchdogParams] = useState<WifiWatchdogParams>({
    watchdog_enabled: false,
    check_interval: 5.0,
    wifi_ssid: '',
    wifi_pass: ''
  });
  const [isUpdating, setIsUpdating] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Get current watchdog parameters
  const getWatchdogParams = useCallback(async (): Promise<WifiWatchdogParams> => {
    if (!connection.ros || !connection.online) {
      return watchdogParams;
    }

    const paramNames = [
      'watchdog_enabled',
      'check_interval',
      'wifi_ssid',
      'wifi_pass'
    ];

    try {
      const params: Partial<WifiWatchdogParams> = {};

      for (const paramName of paramNames) {
        const param = new ROSLIB.Param({
          ros: connection.ros,
          name: `/wifi_services/${paramName}`
        });

        await new Promise((resolve) => {
          // Add timeout for parameter get
          const timeoutId = setTimeout(() => {
            console.log(`Parameter ${paramName} timeout - not available`);
            resolve(null);
          }, 2000); // 2 second timeout per parameter

          param.get((value: any) => {
            clearTimeout(timeoutId);

            // Don't try to parse the value, just use it directly
            // ROSLIB already handles JSON parsing internally
            if (value !== null && value !== undefined && value !== '') {
              params[paramName as keyof WifiWatchdogParams] = value;
            }
            resolve(value);
          });
        });
      }

      const newParams = {
        ...watchdogParams,
        ...params
      };

      setWatchdogParams(newParams);
      return newParams;

    } catch (err) {
      console.error('Failed to get watchdog parameters:', err);
      return watchdogParams;
    }
  }, [connection, watchdogParams]);

  // Set watchdog enabled/disabled
  const setWatchdogEnabled = useCallback(async (enabled: boolean): Promise<boolean> => {
    if (!connection.ros || !connection.online) {
      setError('Not connected to ROS');
      return false;
    }

    setIsUpdating(true);
    setError(null);

    try {
      const param = new ROSLIB.Param({
        ros: connection.ros,
        name: '/wifi_services/watchdog_enabled'
      });

      await new Promise((resolve, reject) => {
        // Add timeout for parameter set
        const timeoutId = setTimeout(() => {
          console.log('Watchdog enabled parameter set timeout');
          reject(new Error('Timeout setting parameter'));
        }, 5000); // 5 second timeout

        try {
          param.set(enabled, () => {
            clearTimeout(timeoutId);
            resolve(true); // Just resolve true, don't try to process result
          });
        } catch (err) {
          clearTimeout(timeoutId);
          reject(err);
        }
      });

      setWatchdogParams(prev => ({ ...prev, watchdog_enabled: enabled }));
      setIsUpdating(false);
      return true;

    } catch (err) {
      const errorMsg = `Failed to set watchdog enabled: ${err}`;
      setError(errorMsg);
      setIsUpdating(false);
      return false;
    }
  }, [connection]);

  // Set check interval
  const setCheckInterval = useCallback(async (interval: number): Promise<boolean> => {
    if (!connection.ros || !connection.online) {
      setError('Not connected to ROS');
      return false;
    }

    if (interval < 1 || interval > 60) {
      setError('Check interval must be between 1 and 60 seconds');
      return false;
    }

    setIsUpdating(true);
    setError(null);

    try {
      const param = new ROSLIB.Param({
        ros: connection.ros,
        name: '/wifi_services/check_interval'
      });

      await new Promise((resolve, reject) => {
        // Add timeout for parameter set
        const timeoutId = setTimeout(() => {
          console.log('Check interval parameter set timeout');
          reject(new Error('Timeout setting parameter'));
        }, 5000); // 5 second timeout

        try {
          param.set(interval, () => {
            clearTimeout(timeoutId);
            resolve(true); // Just resolve true, don't try to process result
          });
        } catch (err) {
          clearTimeout(timeoutId);
          reject(err);
        }
      });

      setWatchdogParams(prev => ({ ...prev, check_interval: interval }));
      setIsUpdating(false);
      return true;

    } catch (err) {
      const errorMsg = `Failed to set check interval: ${err}`;
      setError(errorMsg);
      setIsUpdating(false);
      return false;
    }
  }, [connection]);

  // Set WiFi credentials for auto-reconnect
  const setWifiCredentials = useCallback(async (ssid: string, password: string): Promise<boolean> => {
    if (!connection.ros || !connection.online) {
      setError('Not connected to ROS');
      return false;
    }

    setIsUpdating(true);
    setError(null);

    try {
      // Set SSID
      const ssidParam = new ROSLIB.Param({
        ros: connection.ros,
        name: '/wifi_services/wifi_ssid'
      });

      await new Promise((resolve, reject) => {
        // Add timeout for parameter set
        const timeoutId = setTimeout(() => {
          console.log('SSID parameter set timeout');
          reject(new Error('Timeout setting SSID'));
        }, 5000); // 5 second timeout

        try {
          ssidParam.set(ssid, () => {
            clearTimeout(timeoutId);
            resolve(true); // Just resolve true, don't try to process result
          });
        } catch (err) {
          clearTimeout(timeoutId);
          reject(err);
        }
      });

      // Set Password
      const passParam = new ROSLIB.Param({
        ros: connection.ros,
        name: '/wifi_services/wifi_pass'
      });

      await new Promise((resolve, reject) => {
        // Add timeout for parameter set
        const timeoutId = setTimeout(() => {
          console.log('Password parameter set timeout');
          reject(new Error('Timeout setting password'));
        }, 5000); // 5 second timeout

        try {
          passParam.set(password, () => {
            clearTimeout(timeoutId);
            resolve(true); // Just resolve true, don't try to process result
          });
        } catch (err) {
          clearTimeout(timeoutId);
          reject(err);
        }
      });

      setWatchdogParams(prev => ({
        ...prev,
        wifi_ssid: ssid,
        wifi_pass: password
      }));
      setIsUpdating(false);
      return true;

    } catch (err) {
      const errorMsg = `Failed to set WiFi credentials: ${err}`;
      setError(errorMsg);
      setIsUpdating(false);
      return false;
    }
  }, [connection]);

  // Load initial parameters when connected
  useEffect(() => {
    if (connection.online) {
      getWatchdogParams();
    }
  }, [connection.online]); // eslint-disable-line react-hooks/exhaustive-deps

  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    watchdogParams,
    isUpdating,
    error,
    getWatchdogParams,
    setWatchdogEnabled,
    setCheckInterval,
    setWifiCredentials,
    clearError
  };
}