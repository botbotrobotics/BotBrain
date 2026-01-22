'use client';

import * as ROSLIB from 'roslib';
import RobotConnectionData from '@/interfaces/RobotConnectionData';
import React, {
  createContext,
  useState,
  useContext,
  ReactNode,
  useEffect,
  useRef,
} from 'react';
// Import CBOR library
import 'cbor-js';
import { Database } from '@/types/database.types';
import { auditLogger } from '@/utils/audit-logger';

type Robot = Database['public']['Tables']['robots']['Row'];

// Declare the CBOR global variable from the imported library
declare global {
  interface Window {
    CBOR: {
      decode: (data: ArrayBuffer) => any;
      encode: (data: any) => ArrayBuffer;
    };
  }
  var CBOR: {
    decode: (data: ArrayBuffer) => any;
    encode: (data: any) => ArrayBuffer;
  };
}

// Extend ROSLIB.Ros options type to include encoding
declare module 'roslib' {
  interface Options {
    encoding?: string;
  }
}

export type ConnectionStatus = 'idle' | 'connecting' | 'connected' | 'error';

interface RobotContextType {
  connection: RobotConnectionData;
  connectionStatus: ConnectionStatus;
  lastError: string | null;
  updateRobot: (newData: Partial<RobotConnectionData>) => void;
  connectToRobot: (ipAddress: string, timeoutMs?: number) => Promise<void>;
  connectToRobotWithInfo: (robot: Robot, timeoutMs?: number) => Promise<void>;
  disconnectRobot: () => void;
}

// Default IP - can be configured by the user
const DEFAULT_IP = process.env.NEXT_PUBLIC_ROS_IP ?? '192.168.1.95';
const DEFAULT_PORT = process.env.NEXT_PUBLIC_ROS_PORT ?? '9090';

const createRos = (ipAddress: string) => {
  // Clean up the address - remove @ prefix if present
  let cleanAddress = ipAddress.startsWith('@') ? ipAddress.substring(1) : ipAddress;
  let url = cleanAddress;

  // For standard IP addresses
  if (!url.startsWith('ws://') && !url.startsWith('wss://') && !url.startsWith('http://') && !url.startsWith('https://')) {
    url = `ws://${url}:${DEFAULT_PORT}`;
  }
  // For HTTP URLs
  else if (url.startsWith('http://')) {
    url = url.replace('http://', 'ws://');
    if (!url.includes(':')) {
      url = `${url}:${DEFAULT_PORT}`;
    }
  }
  // For HTTPS URLs
  else if (url.startsWith('https://')) {
    url = url.replace('https://', 'wss://');
    if (!url.includes(':')) {
      url = `${url}:${DEFAULT_PORT}`;
    }
  }
  // For explicit websocket URLs that don't specify port
  else if ((url.startsWith('ws://') && !url.includes(':', 5)) ||
           (url.startsWith('wss://') && !url.includes(':', 6))) {
    url = `${url}:${DEFAULT_PORT}`;
  }

  console.log('Attempting connection to:', url);

  // Use type assertion to handle the encoding property
  const options: any = {
    url,
    // Enhanced connection options for better real-time performance
    transportLibrary: 'websocket',
    groovyCompatibility: false,
    // Enable CBOR encoding/decoding for better performance
    encoding: 'cbor',
  };

  return new ROSLIB.Ros(options);
};

const RobotConnectionContext = createContext<RobotContextType | undefined>(
  undefined
);

export const RobotConnectionProvider: React.FC<{ children: ReactNode }> = ({
  children,
}) => {
  const [connection, setRobot] = useState<RobotConnectionData>({
    ros: undefined,
    online: false,
    connectedRobot: null,
  });
  const [connectionStatus, setConnectionStatus] =
    useState<ConnectionStatus>('idle');
  const [lastError, setLastError] = useState<string | null>(null);
  
  // Track if disconnection was manual
  const isManualDisconnectRef = useRef(false);

  const updateRobot = (newData: Partial<RobotConnectionData>) => {
    setRobot((prev) => ({ ...prev, ...newData }));
  };

  /**
   * Connects to the robot with the given IP address
   * @param ipAddress The IP address to connect to
   * @param timeoutMs The connection timeout in milliseconds (optional)
   */
  const connectToRobot = async (ipAddress: string, timeoutMs?: number): Promise<void> => {
    try {
      setConnectionStatus('connecting');
      setLastError(null);
      // Don't reset the flag here - wait until after connection

      // If no timeout specified, try to get from user settings or use default
      let connectionTimeout = timeoutMs;
      if (!connectionTimeout) {
        try {
          // Import the shared Supabase client
          const { createSupabaseClient } = await import('@/utils/supabase-client');
          const supabase = createSupabaseClient();

          // Get the current user session
          const { data: { session } } = await supabase.auth.getSession();

          if (session?.user) {
            // Get user profile with timeout setting
            const { data: profile } = await supabase
              .from('user_profiles')
              .select('connection_timeout')
              .eq('user_id', session.user.id)
              .single();

            if (profile?.connection_timeout) {
              connectionTimeout = profile.connection_timeout;
            }
          }
        } catch (error) {
          console.log('Could not load user timeout preference:', error);
        }
      }

      // Default to 20 seconds if still not set
      if (!connectionTimeout) {
        connectionTimeout = 20000;
      }

      // Disconnect existing connection if any
      if (connection.ros && connection.ros.isConnected) {
        // Set manual disconnect flag to prevent auto-reconnect
        isManualDisconnectRef.current = true;
        connection.ros.close();
      }

      const ros = createRos(ipAddress);

      // Create a promise that resolves when connected or rejects on error
      await new Promise<void>((resolve, reject) => {
        const onConnect = () => {
          console.log('Connection established successfully');
          ros.removeListener('connection', onConnect);
          ros.removeListener('error', onError);
          resolve();
        };

        const onError = (error: any) => {
          console.error('Connection error details:', error);
          ros.removeListener('connection', onConnect);
          ros.removeListener('error', onError);
          reject(error);
        };

        ros.addListener('connection', onConnect);
        ros.addListener('error', onError);

        // Use the configurable timeout
        console.log(`Using connection timeout: ${connectionTimeout}ms`);
        setTimeout(() => {
          ros.removeListener('connection', onConnect);
          ros.removeListener('error', onError);
          reject(new Error('Connection timeout'));
        }, connectionTimeout);
      });

      // Successfully connected - now reset the manual disconnect flag
      console.log('Connected to ROS via websocket with CBOR encoding!');
      isManualDisconnectRef.current = false;
      updateRobot({ ros, online: true });
      setConnectionStatus('connected');

      // Setup disconnect listener with auto-reconnect
      ros.on('close', () => {
        console.log('Connection closed.');
        const currentRobot = connection.connectedRobot;
        updateRobot({ online: false });
        setConnectionStatus('idle');

        // Only attempt to reconnect if it wasn't a manual disconnect
        if (!isManualDisconnectRef.current) {
          console.log('Trying to reconnect...');
          if (currentRobot) {
            // If we have robot info, reconnect with full info (pass the saved timeout)
            connectToRobotWithInfo(currentRobot, connectionTimeout).catch((error) => {
              console.log('Automatic reconnection failed', error);
            });
          } else {
            // Otherwise just reconnect with IP (pass the saved timeout)
            connectToRobot(ipAddress, connectionTimeout).catch((error) => {
              console.log('Automatic reconnection failed', error);
            });
          }
        } else {
          console.log('Manual disconnect - not reconnecting');
          // Reset the flag after processing
          isManualDisconnectRef.current = false;
        }
      });

      // Add keep-alive ping to maintain connection
      const keepAliveInterval = setInterval(() => {
        if (ros && ros.isConnected) {
          // Send a simple service call to keep the connection alive
          try {
            const pingService = new ROSLIB.Service({
              ros: ros,
              name: '/rosapi/get_time',
              serviceType: 'rosapi/GetTime',
            });

            pingService.callService(
              {},
              () => {},
              (error) => {
                console.log('Ping error, connection may be unstable', error);
                // Clear and reinitialize connection if ping fails
                if (ros.isConnected) {
                  ros.close();
                }
              }
            );
          } catch {
            // Ignore errors on ping
          }
        } else {
          clearInterval(keepAliveInterval);
        }
      }, 10000); // Send ping every 10 seconds instead of 30
    } catch (error) {
      console.log('Error connecting to ROS');
      console.error(error);
      // Reset manual disconnect flag on error too
      isManualDisconnectRef.current = false;
      updateRobot({ online: false });
      setConnectionStatus('error');
      setLastError(
        error instanceof Error ? error.message : 'Unknown connection error'
      );

      // Don't auto-retry on manual connection attempts
      throw error;
    }
  };

  /**
   * Initialize the ROS connection - try to connect to favorite robot
   */
  const initializeRosConnection = async () => {
    // Check if we should auto-connect to a favorite robot
    const supabaseUrl = process.env.NEXT_PUBLIC_SUPABASE_URL;
    const supabaseAnonKey = process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY;
    
    if (!supabaseUrl || !supabaseAnonKey) {
      console.log('Supabase not configured, skipping auto-connect');
      return;
    }

    try {
      // Import the shared Supabase client
      const { createSupabaseClient } = await import('@/utils/supabase-client');
      const supabase = createSupabaseClient();
      
      // Get the current user session
      const { data: { session } } = await supabase.auth.getSession();
      
      if (!session?.user) {
        console.log('No user session, skipping auto-connect');
        return;
      }

      // Check for saved robot ID in localStorage
      const savedRobotId = localStorage.getItem('connectedRobotId');
      
      // Query for favorite robot or saved robot
      let robot: Robot | null = null;

      if (savedRobotId) {
        // Try to connect to the last connected robot
        const { data, error } = await supabase
          .from('robots')
          .select('*')
          .eq('user_id', session.user.id)
          .eq('id', savedRobotId)
          .single();
        
        robot = data;
      } else {
        // Otherwise, connect to favorite robot
        const { data, error } = await supabase
          .from('robots')
          .select('*')
          .eq('user_id', session.user.id)
          .eq('is_favorite', true)
          .single();
        
        robot = data;
      }

      if (!robot) {
        console.log('No favorite/saved robot found');
        return;
      }

      // Try to connect to the robot
      console.log(`Attempting to auto-connect to ${robot.name}`);
      await connectToRobotWithInfo(robot);
    } catch (error) {
      console.log('Auto-connect failed:', error);
      // Don't throw, just log - this is a best-effort auto-connect
    }
  };

  /**
   * Connects to a robot with full robot information
   * @param robot The robot object containing all details
   * @param timeoutMs The connection timeout in milliseconds (optional)
   */
  const connectToRobotWithInfo = async (robot: Robot, timeoutMs?: number): Promise<void> => {
    try {
      // First connect to the robot
      await connectToRobot(robot.address, timeoutMs);
      
      // If successful, update the connected robot info
      updateRobot({ connectedRobot: robot });
      
      // Save the robot ID to local storage for persistence
      if (typeof window !== 'undefined') {
        localStorage.setItem('connectedRobotId', robot.id);
      }
      
      // Log successful connection
      await auditLogger.logRobotConnect(robot.id, robot.name, robot.address);
    } catch (error) {
      // Log failed connection
      await auditLogger.logRobotConnectionFailed(
        robot.address, 
        error instanceof Error ? error.message : 'Unknown error'
      );
      
      // If connection fails, clear any connected robot info
      updateRobot({ connectedRobot: null });
      if (typeof window !== 'undefined') {
        localStorage.removeItem('connectedRobotId');
      }
      throw error;
    }
  };

  /**
   * Disconnects from the current robot
   */
  const disconnectRobot = () => {
    // Set manual disconnect flag before closing
    isManualDisconnectRef.current = true;
    
    // Log disconnection
    if (connection.connectedRobot) {
      auditLogger.logRobotDisconnect(connection.connectedRobot.id, connection.connectedRobot.name);
    }
    
    if (connection.ros && connection.ros.isConnected) {
      connection.ros.close();
    }
    updateRobot({ ros: undefined, online: false, connectedRobot: null });
    setConnectionStatus('idle');
    if (typeof window !== 'undefined') {
      localStorage.removeItem('connectedRobotId');
    }
  };

  useEffect(() => {
    // Wait a bit for auth to initialize
    const timer = setTimeout(() => {
      initializeRosConnection();
    }, 1000);

    return () => clearTimeout(timer);
  }, []);

  return (
    <RobotConnectionContext.Provider
      value={{
        connection,
        connectionStatus,
        lastError,
        updateRobot,
        connectToRobot,
        connectToRobotWithInfo,
        disconnectRobot,
      }}
    >
      {children}
    </RobotConnectionContext.Provider>
  );
};

export const useRobotConnection = () => {
  const context = useContext(RobotConnectionContext);
  if (!context) {
    throw new Error(
      'useRobotConnection must be used within a RobotConnectionProvider'
    );
  }
  return context;
};
