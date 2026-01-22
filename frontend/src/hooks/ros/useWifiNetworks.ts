'use client';

import { useState, useCallback } from 'react';
import * as ROSLIB from 'roslib';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { WifiNetwork, parseNetworkList } from '@/types/WifiControl';

export function useWifiNetworks() {
  const { connection } = useRobotConnection();
  const [isScanning, setIsScanning] = useState(false);
  const [networks, setNetworks] = useState<WifiNetwork[]>([]);
  const [error, setError] = useState<string | null>(null);

  const scanNetworks = useCallback(async (
    rescan: boolean = false,
    interfaceName?: string
  ): Promise<WifiNetwork[]> => {
    setIsScanning(true);
    setError(null);

    return new Promise((resolve, reject) => {
      // Check if we have a ROS connection
      if (!connection.ros) {
        const errorMsg = 'ROS not initialized';
        console.log('WiFi scan skipped - ROS not initialized');
        setError(null); // Don't show error for this case
        setIsScanning(false);
        setNetworks([]);
        resolve([]);
        return;
      }

      // Check if actually connected
      if (!connection.online) {
        const errorMsg = 'Not connected to robot';
        console.log('WiFi scan skipped - not connected to robot');
        setError(null); // Don't show error for this case
        setIsScanning(false);
        setNetworks([]);
        resolve([]);
        return;
      }

      try {
        const service = new ROSLIB.Service({
          ros: connection.ros,
          name: '/available_networks',
          serviceType: 'bot_jetson_stats_interfaces/srv/GetAvailableNetworks'
        });

        if (!service) {
          throw new Error('Failed to create service client');
        }

        const request = {
          rescan: rescan || false,
          interface: interfaceName || ''
        };

        // Add timeout for service call
        const timeoutId = setTimeout(() => {
          console.log('WiFi scan timeout - service not responding');
          setIsScanning(false);
          setError(null);
          setNetworks([]);
          resolve([]);
        }, 10000); // 10 second timeout

        service.callService(
          request,
          (response: any) => {
            clearTimeout(timeoutId); // Clear timeout on response
            setIsScanning(false);

            console.log('Raw WiFi service response:', response);

            // Validate response exists and is not empty
            if (!response || (typeof response === 'string' && response.trim() === '')) {
              console.log('Empty response from WiFi service');
              setNetworks([]);
              resolve([]);
              return;
            }

            // Handle string responses (might be raw JSON string)
            let parsedResponse = response;
            if (typeof response === 'string') {
              try {
                parsedResponse = JSON.parse(response);
              } catch (e) {
                // Not JSON, treat as raw nmcli output directly
                console.log('Treating response as raw nmcli output');
                parsedResponse = { success: true, message: response };
              }
            }

            console.log('WiFi scan response:', parsedResponse);

            // Try to get network data from various possible fields
            let networkData = '';
            if (typeof parsedResponse === 'string') {
              // Direct string response - might be nmcli output
              networkData = parsedResponse;
            } else if (parsedResponse) {
              // Check table field first (this is where nmcli output is)
              if (parsedResponse.table) {
                console.log('Found network data in table field');
                networkData = parsedResponse.table;
              } else {
                // Look for network data in other fields
                networkData = parsedResponse.networks ||
                            parsedResponse.data ||
                            parsedResponse.output ||
                            '';

                // Only use message field if it's not a simple success indicator
                if (!networkData && parsedResponse.message &&
                    !['ok', 'success', 'true', 'false'].includes(parsedResponse.message.toLowerCase().trim())) {
                  networkData = parsedResponse.message;
                }
              }
            }

            // Only parse if we have actual network data
            if (networkData && networkData.trim() &&
                !['ok', 'success', 'true', 'false'].includes(networkData.toLowerCase().trim())) {
              try {
                const parsedNetworks = parseNetworkList(networkData);
                console.log('Parsed networks:', parsedNetworks);
                setNetworks(parsedNetworks);
                resolve(parsedNetworks);
              } catch (parseError) {
                const errorMsg = `Failed to parse network list: ${parseError}`;
                console.error(errorMsg, parseError);
                // Don't show parse errors to user
                setError(null);
                setNetworks([]);
                resolve([]);
              }
            } else {
              // No network data or just success message
              console.log('No network data in response');
              setNetworks([]);
              resolve([]);
            }
          },
          (error: any) => {
            clearTimeout(timeoutId); // Clear timeout on error
            console.error('WiFi service error:', error);
            const errorMsg = typeof error === 'string' ? error : 'Service unavailable';

            // Don't show error if service simply doesn't exist
            if (errorMsg.includes('Service') || errorMsg.includes('unavailable')) {
              console.log('WiFi service not available on robot');
              setError(null);
              setNetworks([]);
              resolve([]);
            } else {
              setError(`Service call failed: ${errorMsg}`);
              reject(new Error(errorMsg));
            }
            setIsScanning(false);
          }
        );
      } catch (error) {
        console.error('Failed to setup WiFi service:', error);
        setError(null); // Don't show error in UI
        setIsScanning(false);
        setNetworks([]);
        resolve([]);
      }
    });
  }, [connection]);

  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    networks,
    isScanning,
    error,
    scanNetworks,
    clearError
  };
}