'use client';

import { useState, useCallback } from 'react';
import * as ROSLIB from 'roslib';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';

export function useWifiRadio() {
  const { connection } = useRobotConnection();
  const [isToggling, setIsToggling] = useState(false);
  const [savedNetworks, setSavedNetworks] = useState<string[]>([]);
  const [fourGStatus, setFourGStatus] = useState<string>('');
  const [error, setError] = useState<string | null>(null);

  // Toggle WiFi radio on/off
  const toggleWifiRadio = useCallback(async (enable: boolean): Promise<boolean> => {
    setIsToggling(true);
    setError(null);

    return new Promise((resolve, reject) => {
      if (!connection.ros || !connection.online) {
        const errorMsg = 'Not connected to robot';
        setError(errorMsg);
        setIsToggling(false);
        reject(new Error(errorMsg));
        return;
      }

      const service = new ROSLIB.Service({
        ros: connection.ros,
        name: '/wifi_radio',
        serviceType: 'std_srvs/srv/SetBool'
      });

      if (!service) {
        const errorMsg = 'Failed to connect to WiFi radio service';
        setError(errorMsg);
        setIsToggling(false);
        reject(new Error(errorMsg));
        return;
      }

      const request = { data: enable };

      // Add timeout for service call
      const timeoutId = setTimeout(() => {
        console.log('WiFi radio toggle timeout - service not responding');
        setError('Timeout - service not responding');
        setIsToggling(false);
        reject(new Error('Service timeout'));
      }, 10000); // 10 second timeout

      service.callService(
        request,
        (response: any) => {
          clearTimeout(timeoutId);
          setIsToggling(false);

          // Handle string responses
          let parsedResponse = response;
          if (typeof response === 'string') {
            try {
              parsedResponse = JSON.parse(response);
            } catch (e) {
              parsedResponse = { success: false, message: response };
            }
          }

          if (parsedResponse && parsedResponse.success) {
            resolve(true);
          } else {
            const errorMsg = parsedResponse?.message || 'Failed to toggle WiFi radio';
            setError(errorMsg);
            reject(new Error(errorMsg));
          }
        },
        (error: string) => {
          clearTimeout(timeoutId);
          const errorMsg = `Failed to toggle WiFi: ${error}`;
          setError(errorMsg);
          setIsToggling(false);
          reject(new Error(errorMsg));
        }
      );
    });
  }, [connection]);

  // Get saved networks
  const getSavedNetworks = useCallback(async (): Promise<string[]> => {
    setError(null);

    return new Promise((resolve, reject) => {
      if (!connection.ros || !connection.online) {
        const errorMsg = 'Not connected to robot';
        setError(errorMsg);
        reject(new Error(errorMsg));
        return;
      }

      const service = new ROSLIB.Service({
        ros: connection.ros,
        name: '/saved_networks',
        serviceType: 'std_srvs/srv/Trigger'
      });

      if (!service) {
        const errorMsg = 'Failed to connect to saved networks service';
        setError(errorMsg);
        reject(new Error(errorMsg));
        return;
      }

      const request = {};

      // Add timeout for service call
      const timeoutId = setTimeout(() => {
        console.log('Saved networks timeout - service not responding');
        setSavedNetworks([]);
        resolve([]);
      }, 10000); // 10 second timeout

      service.callService(
        request,
        (response: any) => {
          clearTimeout(timeoutId);

          // Handle string responses
          let parsedResponse = response;
          if (typeof response === 'string') {
            try {
              parsedResponse = JSON.parse(response);
            } catch (e) {
              // Not JSON, treat as raw output
              parsedResponse = { success: true, message: response };
            }
          }

          console.log('Saved networks response:', parsedResponse);

          if (parsedResponse && parsedResponse.success) {
            // The message field contains the network names separated by newlines
            const dataField = parsedResponse.message || '';

            // Check if it's actual network names or just a success message
            if (typeof dataField === 'string' &&
                dataField.trim() &&
                dataField.toLowerCase() !== 'ok' &&
                dataField.toLowerCase() !== 'success' &&
                dataField.toLowerCase() !== 'true') {

              // Parse saved networks - simple newline separated list
              const networks = dataField
                .split('\n')
                .map((n: string) => n.trim())
                .filter((n: string) => n.length > 0);

              setSavedNetworks(networks);
              resolve(networks);
            } else {
              // No saved networks or just success message
              setSavedNetworks([]);
              resolve([]);
            }
          } else {
            const errorMsg = parsedResponse?.message || 'Failed to get saved networks';
            setError(errorMsg);
            reject(new Error(errorMsg));
          }
        },
        (error: string) => {
          clearTimeout(timeoutId);
          const errorMsg = `Failed to get saved networks: ${error}`;
          setError(errorMsg);
          reject(new Error(errorMsg));
        }
      );
    });
  }, [connection]);

  // Check 4G status
  const check4GStatus = useCallback(async (): Promise<string> => {
    setError(null);

    return new Promise((resolve, reject) => {
      if (!connection.ros || !connection.online) {
        const errorMsg = 'Not connected to robot';
        setError(errorMsg);
        reject(new Error(errorMsg));
        return;
      }

      const service = new ROSLIB.Service({
        ros: connection.ros,
        name: '/check_4g',
        serviceType: 'std_srvs/srv/Trigger'
      });

      if (!service) {
        const errorMsg = 'Failed to connect to 4G check service';
        setError(errorMsg);
        reject(new Error(errorMsg));
        return;
      }

      const request = {};

      // Add timeout for service call
      const timeoutId = setTimeout(() => {
        console.log('4G status timeout - service not responding');
        setFourGStatus('');
        resolve('');
      }, 10000); // 10 second timeout

      service.callService(
        request,
        (response: any) => {
          clearTimeout(timeoutId);

          // Handle string responses
          let parsedResponse = response;
          if (typeof response === 'string') {
            try {
              parsedResponse = JSON.parse(response);
            } catch (e) {
              parsedResponse = { success: true, message: response };
            }
          }

          if (parsedResponse && (parsedResponse.success || parsedResponse.message || parsedResponse.table)) {
            // Parse the data which contains interface status
            // Format: "interface UP/DOWN [IP]"
            const status = parsedResponse.table || parsedResponse.message || '';
            setFourGStatus(status);
            resolve(status);
          } else {
            const errorMsg = '4G interface not configured';
            setError(errorMsg);
            reject(new Error(errorMsg));
          }
        },
        (error: string) => {
          clearTimeout(timeoutId);
          const errorMsg = `Failed to check 4G status: ${error}`;
          setError(errorMsg);
          reject(new Error(errorMsg));
        }
      );
    });
  }, [connection]);

  // Update network status file
  const updateNetworkStatus = useCallback(async (): Promise<boolean> => {
    setError(null);

    return new Promise((resolve, reject) => {
      if (!connection.ros || !connection.online) {
        const errorMsg = 'Not connected to robot';
        setError(errorMsg);
        reject(new Error(errorMsg));
        return;
      }

      const service = new ROSLIB.Service({
        ros: connection.ros,
        name: '/update_network_status',
        serviceType: 'std_srvs/srv/Trigger'
      });

      if (!service) {
        const errorMsg = 'Failed to connect to network status service';
        setError(errorMsg);
        reject(new Error(errorMsg));
        return;
      }

      const request = {};

      // Add timeout for service call
      const timeoutId = setTimeout(() => {
        console.log('Network status update timeout - service not responding');
        resolve(false);
      }, 10000); // 10 second timeout

      service.callService(
        request,
        (response: any) => {
          clearTimeout(timeoutId);

          // Handle string responses
          let parsedResponse = response;
          if (typeof response === 'string') {
            try {
              parsedResponse = JSON.parse(response);
            } catch (e) {
              parsedResponse = { success: false, message: response };
            }
          }

          if (parsedResponse && parsedResponse.success) {
            resolve(true);
          } else {
            const errorMsg = parsedResponse?.message || 'Failed to update network status';
            setError(errorMsg);
            reject(new Error(errorMsg));
          }
        },
        (error: string) => {
          clearTimeout(timeoutId);
          const errorMsg = `Failed to update status: ${error}`;
          setError(errorMsg);
          reject(new Error(errorMsg));
        }
      );
    });
  }, [connection]);

  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    isToggling,
    savedNetworks,
    fourGStatus,
    error,
    toggleWifiRadio,
    getSavedNetworks,
    check4GStatus,
    updateNetworkStatus,
    clearError
  };
}