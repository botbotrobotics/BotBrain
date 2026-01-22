'use client';

import { useState, useCallback } from 'react';
import * as ROSLIB from 'roslib';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';

export interface UploaderConfig {
  minConfidence: number;
  objectFilter: string;
}

export function useYoloUploaderConfig() {
  const { connection } = useRobotConnection();
  const [isLoading, setIsLoading] = useState(false);
  const [isFetching, setIsFetching] = useState(false);
  const [lastError, setLastError] = useState<string | null>(null);

  const getParameters = useCallback(
    (): Promise<UploaderConfig> => {
      return new Promise((resolve, reject) => {
        if (!connection.ros || !connection.online) {
          reject(new Error('Robot not connected'));
          return;
        }

        setIsFetching(true);

        const service = new ROSLIB.Service({
          ros: connection.ros,
          name: '/uploader_node/get_parameters',
          serviceType: 'rcl_interfaces/srv/GetParameters'
        });

        const request = {
          names: ['min_confidence', 'object_filter']
        };

        const timeoutId = setTimeout(() => {
          setIsFetching(false);
          reject(new Error('Service timeout'));
        }, 10000);

        service.callService(
          request,
          (response: any) => {
            clearTimeout(timeoutId);
            setIsFetching(false);

            try {
              const values = response.values || [];
              let minConfidence = 0.5;
              let objectFilter = '';

              // Parse the response - values are in same order as requested names
              if (values[0]) {
                // min_confidence is a double (type 3)
                minConfidence = values[0].double_value ?? 0.5;
              }
              if (values[1]) {
                // object_filter is a string (type 4)
                objectFilter = values[1].string_value ?? '';
              }

              resolve({ minConfidence, objectFilter });
            } catch (e) {
              reject(new Error('Failed to parse parameters'));
            }
          },
          (error: any) => {
            clearTimeout(timeoutId);
            setIsFetching(false);
            const errorMsg = typeof error === 'string' ? error : 'Failed to get parameters';
            reject(new Error(errorMsg));
          }
        );
      });
    },
    [connection]
  );

  const setMinConfidence = useCallback(
    (value: number): Promise<boolean> => {
      return new Promise((resolve, reject) => {
        if (!connection.ros || !connection.online) {
          const error = 'Robot not connected';
          setLastError(error);
          reject(new Error(error));
          return;
        }

        setIsLoading(true);
        setLastError(null);

        const service = new ROSLIB.Service({
          ros: connection.ros,
          name: '/uploader_node/set_parameters',
          serviceType: 'rcl_interfaces/srv/SetParameters'
        });

        const request = {
          parameters: [
            {
              name: 'min_confidence',
              value: {
                type: 3, // DOUBLE type
                double_value: value
              }
            }
          ]
        };

        const timeoutId = setTimeout(() => {
          setLastError('Service timeout');
          setIsLoading(false);
          reject(new Error('Service timeout'));
        }, 10000);

        service.callService(
          request,
          (response: any) => {
            clearTimeout(timeoutId);
            setIsLoading(false);
            if (response.results?.[0]?.successful) {
              resolve(true);
            } else {
              const errorMsg = response.results?.[0]?.reason || 'Failed to set min_confidence';
              setLastError(errorMsg);
              reject(new Error(errorMsg));
            }
          },
          (error: any) => {
            clearTimeout(timeoutId);
            setIsLoading(false);
            const errorMsg = typeof error === 'string' ? error : 'Service call failed';
            setLastError(errorMsg);
            reject(new Error(errorMsg));
          }
        );
      });
    },
    [connection]
  );

  const setObjectFilter = useCallback(
    (filter: string): Promise<boolean> => {
      return new Promise((resolve, reject) => {
        if (!connection.ros || !connection.online) {
          const error = 'Robot not connected';
          setLastError(error);
          reject(new Error(error));
          return;
        }

        setIsLoading(true);
        setLastError(null);

        const service = new ROSLIB.Service({
          ros: connection.ros,
          name: '/uploader_node/set_parameters',
          serviceType: 'rcl_interfaces/srv/SetParameters'
        });

        const request = {
          parameters: [
            {
              name: 'object_filter',
              value: {
                type: 4, // STRING type
                string_value: filter
              }
            }
          ]
        };

        const timeoutId = setTimeout(() => {
          setLastError('Service timeout');
          setIsLoading(false);
          reject(new Error('Service timeout'));
        }, 10000);

        service.callService(
          request,
          (response: any) => {
            clearTimeout(timeoutId);
            setIsLoading(false);
            if (response.results?.[0]?.successful) {
              resolve(true);
            } else {
              const errorMsg = response.results?.[0]?.reason || 'Failed to set object_filter';
              setLastError(errorMsg);
              reject(new Error(errorMsg));
            }
          },
          (error: any) => {
            clearTimeout(timeoutId);
            setIsLoading(false);
            const errorMsg = typeof error === 'string' ? error : 'Service call failed';
            setLastError(errorMsg);
            reject(new Error(errorMsg));
          }
        );
      });
    },
    [connection]
  );

  const applySettings = useCallback(
    async (minConfidence: number, objectFilter: string): Promise<boolean> => {
      try {
        setIsLoading(true);
        setLastError(null);

        // Apply both settings
        await setMinConfidence(minConfidence);
        await setObjectFilter(objectFilter);

        setIsLoading(false);
        return true;
      } catch (error) {
        setIsLoading(false);
        throw error;
      }
    },
    [setMinConfidence, setObjectFilter]
  );

  const clearError = useCallback(() => {
    setLastError(null);
  }, []);

  return {
    getParameters,
    setMinConfidence,
    setObjectFilter,
    applySettings,
    isLoading,
    isFetching,
    lastError,
    clearError
  };
}
