'use client';

import { useCallback, useRef } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useNotifications } from '@/contexts/NotificationsContext';
import * as ROSLIB from 'roslib';

interface MappingDatabase {
  name: string;
  path: string;
  size?: string;
  lastModified?: string;
}

interface ServiceResponse {
  success: boolean;
  message: string;
  db_files?: string[];
}

export function useRosMappingServices() {
  const { connection, connectionStatus } = useRobotConnection();
  const { dispatch } = useNotifications();
  const ros = connection.ros;
  const isConnected = connectionStatus === 'connected';
  const servicesRef = useRef<{ [key: string]: ROSLIB.Service<unknown, unknown> }>({});

  const addNotification = useCallback((notification: { type: 'success' | 'error' | 'info' | 'warning'; title: string; message: string }) => {
    dispatch({
      type: 'ADD_NOTIFICATION',
      payload: notification,
    });
  }, [dispatch]);

  const getService = useCallback((serviceName: string, serviceType: string) => {
    if (!ros || !isConnected) {
      throw new Error('ROS is not connected');
    }

    const key = `${serviceName}_${serviceType}`;
    if (!servicesRef.current[key]) {
      servicesRef.current[key] = new ROSLIB.Service({
        ros,
        name: serviceName,
        serviceType,
      });
    }
    return servicesRef.current[key];
  }, [ros, isConnected]);

  const listDatabaseFiles = useCallback(async (): Promise<string[]> => {
    try {
      const service = getService(
        '/list_db_files',
        'bot_localization_interfaces/srv/ListDbFiles'
      );

      return new Promise((resolve, reject) => {
        const request = {};

        service.callService(request, (res: unknown) => {
          const response = res as ServiceResponse;
          if (response.success) {
            resolve(response.db_files || []);
          } else {
            reject(new Error(response.message || 'Failed to list database files'));
          }
        }, (error: unknown) => {
          reject(new Error(String(error)));
        });
      });
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to list maps';
      addNotification({
        type: 'error',
        title: 'List Maps Failed',
        message,
      });
      throw error;
    }
  }, [getService, addNotification]);

  const loadDatabase = useCallback(async (databasePath: string, clearDb: boolean = false): Promise<void> => {
    try {
      const service = getService(
        '/load_database',
        'bot_localization_interfaces/srv/LoadDB'
      );

      return new Promise((resolve, reject) => {
        const request = {
          database_path: databasePath,
          clear_db: clearDb,
        };

        service.callService(request, (res: unknown) => {
          const response = res as ServiceResponse;
          if (response.success) {
            addNotification({
              type: 'success',
              title: 'Map Loaded',
              message: `Successfully loaded map: ${databasePath}`,
            });
            resolve();
          } else {
            reject(new Error(response.message || 'Failed to load database'));
          }
        }, (error: unknown) => {
          reject(new Error(String(error)));
        });
      });
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to load map';
      addNotification({
        type: 'error',
        title: 'Load Map Failed',
        message,
      });
      throw error;
    }
  }, [getService, addNotification]);

  const setMappingMode = useCallback(async (databasePath?: string, clearDb: boolean = false): Promise<void> => {
    try {
      const service = getService(
        '/set_mapping',
        'bot_localization_interfaces/srv/SetMapping'
      );

      return new Promise((resolve, reject) => {
        const request = {
          database_path: databasePath || '',
          clear_db: clearDb,
        };

        service.callService(request, (res: unknown) => {
          const response = res as ServiceResponse;
          if (response.success) {
            const mapName = databasePath || 'current database';
            addNotification({
              type: 'success',
              title: 'Mapping Mode Started',
              message: `Now mapping with ${mapName}`,
            });
            resolve();
          } else {
            reject(new Error(response.message || 'Failed to set mapping mode'));
          }
        }, (error: unknown) => {
          reject(new Error(String(error)));
        });
      });
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to start mapping';
      addNotification({
        type: 'error',
        title: 'Start Mapping Failed',
        message,
      });
      throw error;
    }
  }, [getService, addNotification]);

  const setLocalizationMode = useCallback(async (): Promise<void> => {
    try {
      const service = getService(
        '/set_localization',
        'bot_localization_interfaces/srv/SetLocalization'
      );

      return new Promise((resolve, reject) => {
        const request = {};

        service.callService(request, (res: unknown) => {
          const response = res as ServiceResponse;
          if (response.success) {
            addNotification({
              type: 'success',
              title: 'Localization Mode',
              message: 'Switched to localization mode',
            });
            resolve();
          } else {
            reject(new Error(response.message || 'Failed to set localization mode'));
          }
        }, (error: unknown) => {
          reject(new Error(String(error)));
        });
      });
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to switch to localization';
      addNotification({
        type: 'error',
        title: 'Localization Failed',
        message,
      });
      throw error;
    }
  }, [getService, addNotification]);

  const saveDatabase = useCallback(async (): Promise<void> => {
    try {
      const service = getService(
        '/save_database',
        'bot_localization_interfaces/srv/SaveDatabase'
      );

      return new Promise((resolve, reject) => {
        const request = {};

        service.callService(request, (res: unknown) => {
          const response = res as ServiceResponse;
          if (response.success) {
            addNotification({
              type: 'success',
              title: 'Map Saved',
              message: 'Map has been saved successfully',
            });
            resolve();
          } else {
            reject(new Error(response.message || 'Failed to save database'));
          }
        }, (error: unknown) => {
          reject(new Error(String(error)));
        });
      });
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to save map';
      addNotification({
        type: 'error',
        title: 'Save Map Failed',
        message,
      });
      throw error;
    }
  }, [getService, addNotification]);

  const deleteDatabase = useCallback(async (databasePath: string): Promise<void> => {
    try {
      const service = getService(
        '/delete_database',
        'bot_localization_interfaces/srv/DeleteDB'
      );

      return new Promise((resolve, reject) => {
        const request = {
          database_path: databasePath,
        };

        service.callService(request, (res: unknown) => {
          const response = res as ServiceResponse;
          if (response.success) {
            addNotification({
              type: 'success',
              title: 'Map Deleted',
              message: `Successfully deleted map: ${databasePath}`,
            });
            resolve();
          } else {
            reject(new Error(response.message || 'Failed to delete database'));
          }
        }, (error: unknown) => {
          reject(new Error(String(error)));
        });
      });
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to delete map';
      addNotification({
        type: 'error',
        title: 'Delete Map Failed',
        message,
      });
      throw error;
    }
  }, [getService, addNotification]);

  const getCurrentDatabase = useCallback(async (): Promise<string | null> => {
    try {
      const service = getService(
        '/get_current_database',
        'std_srvs/srv/Trigger'
      );

      return new Promise((resolve) => {
        service.callService({}, (res: unknown) => {
          const response = res as { success: boolean; message: string };
          if (response.success) {
            // message contains the database name/path
            resolve(response.message || null);
          } else {
            resolve(null); // No active database, not an error
          }
        }, (error: unknown) => {
          // Service unavailable - resolve null (no active map)
          console.warn('get_current_database service unavailable:', error);
          resolve(null);
        });
      });
    } catch {
      return null;
    }
  }, [getService]);

  const formatDatabaseInfo = useCallback((dbFiles: string[]): MappingDatabase[] => {
    return dbFiles.map(file => ({
      name: file.replace('.db', ''),
      path: file,
      size: 'Unknown',
      lastModified: 'Unknown',
    }));
  }, []);

  return {
    isConnected,
    listDatabaseFiles,
    loadDatabase,
    setMappingMode,
    setLocalizationMode,
    saveDatabase,
    deleteDatabase,
    getCurrentDatabase,
    formatDatabaseInfo,
  };
}