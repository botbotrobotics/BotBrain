import { useState, useCallback, useRef } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';

interface TriggerResponse {
  success: boolean;
  message: string;
}

export function useSystemReboot() {
  const { connection } = useRobotConnection();
  const [isRebooting, setIsRebooting] = useState(false);
  const [lastError, setLastError] = useState<string | null>(null);
  const serviceClientRef = useRef<ROSLIB.Service<unknown, unknown> | null>(null);

  // Initialize service client
  const getServiceClient = useCallback(() => {
    if (!connection.ros || !connection.online) {
      return null;
    }

    if (!serviceClientRef.current) {
      serviceClientRef.current = new ROSLIB.Service({
        ros: connection.ros,
        name: '/jtop/reboot',
        serviceType: 'std_srvs/srv/Trigger',
      });
    }

    return serviceClientRef.current;
  }, [connection.ros, connection.online]);

  // Execute reboot
  const rebootSystem = useCallback(
    (): Promise<TriggerResponse> => {
      return new Promise((resolve, reject) => {
        const serviceClient = getServiceClient();

        if (!serviceClient) {
          const error = 'ROS not connected';
          setLastError(error);
          reject(new Error(error));
          return;
        }

        setIsRebooting(true);
        setLastError(null);

        // Trigger service doesn't require any request parameters
        const request = {};

        serviceClient.callService(
          request,
          (response: any) => {
            setIsRebooting(false);
            const result = response as TriggerResponse;

            if (result.success) {
              resolve(result);
            } else {
              const errorMsg = result.message || 'Reboot failed';
              setLastError(errorMsg);
              reject(new Error(errorMsg));
            }
          },
          (error: string) => {
            setIsRebooting(false);
            const errorMsg = `Service call failed: ${error}`;
            setLastError(errorMsg);
            reject(new Error(errorMsg));
          }
        );
      });
    },
    [getServiceClient]
  );

  return {
    rebootSystem,
    isRebooting,
    lastError,
    isConnected: connection.online,
  };
}