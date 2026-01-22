'use client';

import { useState, useCallback } from 'react';
import * as ROSLIB from 'roslib';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { WifiAuthType } from '@/types/WifiControl';

interface ConnectOptions {
  ssid: string;
  authType: WifiAuthType;
  psk?: string;
  identity?: string;
  password?: string;
  saveProfile?: boolean;
}

export function useWifiControl() {
  const { connection } = useRobotConnection();
  const [isConnecting, setIsConnecting] = useState(false);
  const [connectionError, setConnectionError] = useState<string | null>(null);

  const connectToNetwork = useCallback(async (options: ConnectOptions): Promise<boolean> => {
    setIsConnecting(true);
    setConnectionError(null);

    return new Promise((resolve, reject) => {
      if (!connection.ros || !connection.online) {
        const errorMsg = 'Not connected to robot';
        setConnectionError(errorMsg);
        setIsConnecting(false);
        reject(new Error(errorMsg));
        return;
      }

      try {
        const service = new ROSLIB.Service({
          ros: connection.ros,
          name: '/connect_wifi',
          serviceType: 'bot_jetson_stats_interfaces/srv/ConnectWifi'
        });

      const request: any = {
        ssid: options.ssid,
        auth_type: options.authType,
        save_profile: options.saveProfile || false
      };

      // Add authentication fields based on auth type
      if (options.authType === WifiAuthType.PSK && options.psk) {
        request.psk = options.psk;
      } else if (options.authType === WifiAuthType.WPA2_ENTERPRISE) {
        if (options.identity) request.identity = options.identity;
        if (options.password) request.password = options.password;
      }

        // Add timeout for service call
        const timeoutId = setTimeout(() => {
          console.log('WiFi connect timeout - service not responding');
          setConnectionError('Connection timeout - service not responding');
          setIsConnecting(false);
          reject(new Error('Connection timeout'));
        }, 30000); // 30 second timeout for connection

        service.callService(
          request,
          (response: any) => {
            clearTimeout(timeoutId);
            setIsConnecting(false);

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
              const errorMsg = parsedResponse?.message || 'Failed to connect to network';
              setConnectionError(errorMsg);
              reject(new Error(errorMsg));
            }
          },
          (error: any) => {
            clearTimeout(timeoutId);
            const errorMsg = typeof error === 'string' ? error : 'Connection failed';
            setConnectionError(errorMsg);
            setIsConnecting(false);
            reject(new Error(errorMsg));
          }
        );
      } catch (error) {
        setConnectionError('Failed to connect - service unavailable');
        setIsConnecting(false);
        reject(new Error('Service unavailable'));
      }
    });
  }, [connection]);

  const disconnectFromNetwork = useCallback(async (): Promise<boolean> => {
    // To disconnect, we can turn off WiFi and turn it back on
    // Or connect to no network - this depends on backend implementation
    // For now, we'll implement a placeholder
    setConnectionError('Disconnect functionality needs backend support');
    return Promise.resolve(false);
  }, []);

  const clearError = useCallback(() => {
    setConnectionError(null);
  }, []);

  return {
    isConnecting,
    connectionError,
    connectToNetwork,
    disconnectFromNetwork,
    clearError
  };
}