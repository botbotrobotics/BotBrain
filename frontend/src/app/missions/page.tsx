'use client';

import { useState, useRef, useEffect, useMemo } from 'react';
import Container from '@/components/ui/container';
import RosCameraImg from '@/components/ros-camera-img';
import RobotActionButton from '@/components/ui/robot-action-button';
import { Menu, X, Map as MapIcon, Plus, Play, Square, Edit2, Trash2, Save, Check, Navigation2, Route, Loader2 } from 'lucide-react';
import useRobotStatus from '@/hooks/ros/useRobotStatus';
import useRobotActionsTransitions from '@/hooks/ros/useRobotActionsTransitions';
import { useRobotCustomModeContext } from '@/contexts/RobotCustomModesContext';
import { RobotActionTypeName } from '@/types/RobotActionTypes';
import MissionMapView from '@/components/mission-map-view';
import { missionsService, MissionWithWaypoints } from '@/services/missions';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { useNotifications } from '@/contexts/NotificationsContext';
import { useRosMappingServices } from '@/hooks/ros/useRosMappingServices';
import { useMapMissionCompatibility } from '@/hooks/useMapMissionCompatibility';
import { MissionMapWarningBanner } from '@/components/mission-map-warning-banner';

// Force dynamic rendering to prevent static generation issues
export const dynamic = 'force-dynamic';

interface Waypoint {
  id: string;
  x: number;
  y: number;
  theta: number;
  order_index: number;
  is_reached?: boolean | null;
  mission_id: string;
  created_at: string | null;
  updated_at: string | null;
  reached_at: string | null;
}

export default function Missions() {
  const [showLeftPanel, setShowLeftPanel] = useState(false);
  const [showRightPanel, setShowRightPanel] = useState(false);
  const [missions, setMissions] = useState<MissionWithWaypoints[]>([]);
  const [selectedMission, setSelectedMission] = useState<MissionWithWaypoints | null>(null);
  const [activeMissionId, setActiveMissionId] = useState<string | null>(null);
  const [isCreatingMission, setIsCreatingMission] = useState(false);
  const [newMissionName, setNewMissionName] = useState('');
  const [editingMissionId, setEditingMissionId] = useState<string | null>(null);
  const [editingMissionName, setEditingMissionName] = useState('');
  const [showNavPlan, setShowNavPlan] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [isSaving, setIsSaving] = useState(false);

  useEffect(() => {
    // Set page title
    document.title = 'Missions - BotBot';
  }, []);
  const { light, statusBeforeEmergency, antiCollision } = useRobotCustomModeContext();
  const { robotStatus } = useRobotStatus();
  const robotActions = useRobotActionsTransitions(robotStatus);
  const robotActionsBeforeEmergency = useRobotActionsTransitions(
    statusBeforeEmergency ?? robotStatus
  );
  const [isInEmergencyMode, setEmergencyMode] = useState(false);
  const [currentWaypointCount, setCurrentWaypointCount] = useState(0);
  const { user } = useSupabase();
  const { dispatch: notificationDispatch } = useNotifications();
  const { getCurrentDatabase, isConnected: rosConnected } = useRosMappingServices();

  // Map-mission compatibility check
  const {
    currentMapName,
    isCompatible: isMapCompatible,
    switchToMissionMap,
    isRobotConnected
  } = useMapMissionCompatibility(selectedMission?.map_name || null);
  const [isSwitchingMap, setIsSwitchingMap] = useState(false);

  const handleSwitchMap = async () => {
    setIsSwitchingMap(true);
    try {
      const success = await switchToMissionMap();
      if (success) {
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'success',
            title: 'Map Switched',
            message: `Loaded map: ${selectedMission?.map_name?.replace(/\.db$/i, '')}`
          }
        });
      }
    } catch (error) {
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'error',
          title: 'Failed to switch map',
          message: 'Could not load the mission map'
        }
      });
    } finally {
      setIsSwitchingMap(false);
    }
  };

  const btnCustomClasses =
    'hover:dark:bg-botbot-dark/80 dark:bg-botbot-dark focus:dark:bg-botbot-darker';

  const emergencyOnBtnClasses = `font-bold p-3 md:p-4 bg-red-500 
    text-white text-md md:text-lg
    hover:bg-red-600
    focus:bg-red-700 mb-2 rounded-default-border`;

  const emergencyOffBtnClasses = `font-bold p-3 md:p-4 bg-green-500 
    text-white text-md md:text-lg
    hover:bg-green-600
    focus:bg-green-700 mb-2 rounded-default-border`;

  useEffect(() => {
    setEmergencyMode(robotStatus === 'emergency');
  }, [robotStatus]);

  // Load missions from Supabase on mount
  useEffect(() => {
    if (!user) return;

    const loadMissions = async () => {
      try {
        setIsLoading(true);
        const data = await missionsService.getMissions();

        // Clean up stale is_active states on page load
        // If a mission is marked as active in DB but we just loaded the page,
        // navigation isn't actually running, so deactivate it
        const activeMission = data.find((m: MissionWithWaypoints) => m.is_active);
        if (activeMission) {
          console.log('Found stale active mission on page load, deactivating:', activeMission.name);
          try {
            await missionsService.setActiveMission(null);
            // Update the local data to reflect the deactivation
            activeMission.is_active = false;
          } catch (error) {
            console.error('Failed to deactivate stale mission:', error);
          }
        }

        setMissions(data);

        // Select the first mission or the previously active one
        if (activeMission) {
          setSelectedMission(activeMission);
        } else if (data.length > 0 && !selectedMission) {
          setSelectedMission(data[0]);
        }
      } catch (error) {
        console.error('Failed to load missions:', error);
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'error',
            title: 'Failed to load missions',
            message: 'Please try refreshing the page'
          }
        });
      } finally {
        setIsLoading(false);
      }
    };

    loadMissions();
    
    // Subscribe to real-time changes
    const subscription = missionsService.subscribeToMissionChanges(() => {
      loadMissions();
    });
    
    return () => {
      subscription.unsubscribe();
    };
  }, [user]);

  const createMission = async () => {
    if (!newMissionName.trim()) return;

    try {
      setIsSaving(true);

      // Capture the current map name when creating mission
      let mapName: string | null = null;
      if (rosConnected) {
        try {
          mapName = await getCurrentDatabase();
        } catch (error) {
          console.warn('Could not get current map:', error);
        }
      }

      const newMission = await missionsService.createMission({
        name: newMissionName.trim(),
        robot_id: null, // TODO: Get from robot connection context
        map_name: mapName
      });
      
      const missionWithWaypoints: MissionWithWaypoints = {
        ...newMission,
        waypoints: []
      };
      
      setMissions([missionWithWaypoints, ...missions]);
      setSelectedMission(missionWithWaypoints);
      setNewMissionName('');
      setIsCreatingMission(false);
      
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'success',
          title: 'Mission created',
          message: `Mission "${newMission.name}" has been created`
        }
      });
    } catch (error) {
      console.error('Failed to create mission:', error);
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'error',
          title: 'Failed to create mission',
          message: 'Please try again'
        }
      });
    } finally {
      setIsSaving(false);
    }
  };

  const updateMission = async (missionId: string) => {
    if (!editingMissionName.trim()) return;
    
    try {
      setIsSaving(true);
      const updated = await missionsService.updateMission(missionId, {
        name: editingMissionName.trim()
      });
      
      setMissions(missions.map(m => 
        m.id === missionId 
          ? { ...m, name: updated.name }
          : m
      ));
      
      if (selectedMission?.id === missionId) {
        setSelectedMission({ ...selectedMission, name: updated.name });
      }
      
      setEditingMissionId(null);
      setEditingMissionName('');
      
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'success',
          title: 'Mission updated',
          message: `Mission has been renamed to "${updated.name}"`
        }
      });
    } catch (error) {
      console.error('Failed to update mission:', error);
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'error',
          title: 'Failed to update mission',
          message: 'Please try again'
        }
      });
    } finally {
      setIsSaving(false);
    }
  };

  const deleteMission = async (missionId: string) => {
    try {
      setIsSaving(true);
      await missionsService.deleteMission(missionId);
      
      const remainingMissions = missions.filter(m => m.id !== missionId);
      setMissions(remainingMissions);
      
      if (selectedMission?.id === missionId) {
        setSelectedMission(remainingMissions[0] || null);
      }
      
      if (activeMissionId === missionId) {
        setActiveMissionId(null);
      }
      
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'success',
          title: 'Mission deleted',
          message: 'Mission has been successfully deleted'
        }
      });
    } catch (error) {
      console.error('Failed to delete mission:', error);
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'error',
          title: 'Failed to delete mission',
          message: 'Please try again'
        }
      });
    } finally {
      setIsSaving(false);
    }
  };

  const playMission = async (missionId: string) => {
    // First, ensure any pending waypoints are saved
    if (saveTimeoutRef.current) {
      clearTimeout(saveTimeoutRef.current);
      saveTimeoutRef.current = null;
      
      // Save the latest waypoints immediately
      if (latestWaypointsRef.current.length > 0) {
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'info',
            title: 'Saving waypoints',
            message: 'Please wait while waypoints are saved...'
          }
        });
        
        await saveWaypoints(latestWaypointsRef.current);
      }
    }
    
    // Reload mission to get latest waypoints from DB
    try {
      setIsSaving(true);
      const freshMission = await missionsService.getMission(missionId);
      
      if (!freshMission || freshMission.waypoints.length === 0) {
        console.warn('Cannot play mission: no waypoints defined in database');
        notificationDispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'error',
            title: 'No waypoints found',
            message: 'Please add waypoints and wait for them to save'
          }
        });
        return;
      }
      
      await missionsService.setActiveMission(missionId);
      await missionsService.resetWaypointsStatus(missionId);
      
      setActiveMissionId(missionId);
      console.log('Playing mission:', missionId, 'with waypoints:', freshMission.waypoints);
      
      // Update the selected mission with fresh data to ensure MissionMapView gets the latest waypoints
      setSelectedMission(freshMission);
      setCurrentWaypointCount(freshMission.waypoints.length);
      
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'success',
          title: 'Mission started',
          message: `Mission "${freshMission.name}" is now active with ${freshMission.waypoints.length} waypoints`
        }
      });
    } catch (error) {
      console.error('Failed to start mission:', error);
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'error',
          title: 'Failed to start mission',
          message: 'Please try again'
        }
      });
    } finally {
      setIsSaving(false);
    }
  };

  const stopMission = async () => {
    try {
      setIsSaving(true);
      console.log('Attempting to stop mission...');
      
      await missionsService.setActiveMission(null);
      
      setActiveMissionId(null);
      console.log('Mission stopped successfully');
      
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'success',
          title: 'Mission stopped',
          message: 'Mission has been stopped'
        }
      });
    } catch (error) {
      console.error('Failed to stop mission - Full error:', error);
      console.error('Error type:', typeof error);
      console.error('Error constructor:', error?.constructor?.name);
      console.error('Error properties:', Object.keys(error || {}));
      console.error('Error stringified:', JSON.stringify(error, null, 2));
      
      // Try different ways to extract error message
      let errorMessage = 'Unknown error occurred';
      if (error instanceof Error) {
        errorMessage = error.message;
      } else if (typeof error === 'string') {
        errorMessage = error;
      } else if (error && typeof error === 'object') {
        errorMessage = (error as any).message || (error as any).error || (error as any).error_description || JSON.stringify(error);
      }
      
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'error',
          title: 'Failed to stop mission',
          message: errorMessage
        }
      });
    } finally {
      setIsSaving(false);
    }
  };

  // Use useRef to store the save timeout and latest waypoints
  const saveTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const latestWaypointsRef = useRef<any[]>([]);
  
  const handleWaypointsChange = (waypoints: any[]) => {
    console.log('handleWaypointsChange called with waypoints:', waypoints);
    
    // Store latest waypoints
    latestWaypointsRef.current = waypoints;
    
    // Update the current waypoint count immediately
    setCurrentWaypointCount(waypoints.length);
    
    if (!selectedMission || !selectedMission.id) return;
    
    // Don't save if mission is brand new and hasn't been saved to DB yet
    if (!missions.find(m => m.id === selectedMission.id)) {
      console.log('Mission not yet saved to database, skipping waypoint save');
      return;
    }
    
    // Clear existing timeout
    if (saveTimeoutRef.current) {
      clearTimeout(saveTimeoutRef.current);
    }
    
    // Set new timeout for debounced save
    saveTimeoutRef.current = setTimeout(() => {
      saveWaypoints(waypoints);
    }, 1000);
  };
  
  const saveWaypoints = async (waypoints: any[]) => {
    if (!selectedMission || !selectedMission.id) return;
    
    try {
      setIsSaving(true);
      const waypointsForSave = waypoints.map((wp, index) => ({
        x: wp.x,
        y: wp.y,
        theta: wp.theta,
        order_index: index,
        is_reached: wp.is_reached || false
      }));
      
      console.log('Saving waypoints:', waypointsForSave);
      
      const savedWaypoints = await missionsService.updateWaypoints(
        selectedMission.id, 
        waypointsForSave
      );
      
      console.log('Waypoints saved successfully:', savedWaypoints);
      
      // Update the missions list with the saved waypoints
      const updatedMissions = missions.map(m => 
        m.id === selectedMission.id 
          ? { ...m, waypoints: savedWaypoints }
          : m
      );
      setMissions(updatedMissions);
      
      // Update selected mission but don't trigger re-render of map
      const updatedMission = { ...selectedMission, waypoints: savedWaypoints };
      setSelectedMission(updatedMission);
      
    } catch (error) {
      console.error('Failed to update waypoints:', error);
      // Better error logging
      if (error instanceof Error) {
        console.error('Error message:', error.message);
        console.error('Error stack:', error.stack);
      }
      
      notificationDispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          type: 'error',
          title: 'Failed to save waypoints',
          message: error instanceof Error ? error.message : 'Your changes may not have been saved'
        }
      });
    } finally {
      setIsSaving(false);
    }
  };
  
  // Clean up timeout on unmount
  useEffect(() => {
    return () => {
      if (saveTimeoutRef.current) {
        clearTimeout(saveTimeoutRef.current);
      }
    };
  }, []);

  const handleNavigationComplete = async () => {
    console.log('Navigation completed - deactivating mission');

    // Deactivate the mission in the database
    try {
      await missionsService.setActiveMission(null);
    } catch (error) {
      console.error('Failed to deactivate mission in database:', error);
    }

    // Update local state
    setActiveMissionId(null);

    // Update the selected mission's is_active flag in local state
    if (selectedMission) {
      setSelectedMission({ ...selectedMission, is_active: false });
      setMissions(missions.map(m =>
        m.id === selectedMission.id
          ? { ...m, is_active: false }
          : m
      ));
    }

    notificationDispatch({
      type: 'ADD_NOTIFICATION',
      payload: {
        type: 'info',
        title: 'Navigation Complete',
        message: 'Mission has finished'
      }
    });
  };

  const getToggleActionButtons = (
    action: RobotActionTypeName
  ): RobotActionTypeName => {
    if (action.includes('light')) return light ? 'lightOff' : 'lightOn';
    else if (action.includes('Collision'))
      return antiCollision ? 'antiCollisionOff' : 'antiCollisionOn';
    return action;
  };
  
  // Memoize the waypoints transformation to prevent unnecessary re-renders
  const transformedWaypoints = useMemo(() => {
    if (!selectedMission?.waypoints) return [];
    
    return selectedMission.waypoints.map((wp: any) => ({
      id: wp.id,
      x: wp.x,
      y: wp.y,
      theta: wp.theta,
      order: wp.order_index,
      isReached: wp.is_reached || false
    }));
  }, [selectedMission?.waypoints]);
  
  // Update waypoint count when mission changes
  useEffect(() => {
    setCurrentWaypointCount(transformedWaypoints.length);
  }, [transformedWaypoints]);

  return (
    <div className="flex flex-col h-full">
      {/* Main Content Area */}
      <div className="flex-1 flex overflow-hidden relative">
        {/* Left Sidebar - Navigation Missions List */}
        <div className={`
          absolute sm:relative sm:translate-x-0 z-20 h-full
          w-72 border-r border-gray-200 dark:border-black 
          bg-gray-50 dark:bg-botbot-darkest overflow-hidden flex flex-col
          transition-transform duration-300 ease-in-out
          ${showLeftPanel ? 'translate-x-0' : '-translate-x-full'}
        `}>
          {/* Mobile Close Button */}
          <button
            onClick={() => setShowLeftPanel(false)}
            className="sm:hidden absolute top-2 right-2 p-1 rounded-lg bg-gray-200 dark:bg-botbot-dark z-10"
          >
            <X className="w-4 h-4" />
          </button>

          {/* Left Panel Header */}
          <div className="p-4 border-b border-gray-200 dark:border-gray-700">
            <div className="flex items-center justify-between">
              <h3 className="text-lg font-semibold text-gray-800 dark:text-white flex items-center gap-2">
                <Navigation2 className="w-5 h-5" />
                Missions
              </h3>
              <button
                onClick={() => setIsCreatingMission(true)}
                className="p-2 bg-primary hover:bg-primary/90 text-white rounded-lg transition-colors"
                title="Create new mission"
              >
                <Plus className="w-4 h-4" />
              </button>
            </div>
          </div>

          {/* Left Panel Content */}
          <div className="flex-1 overflow-y-auto p-4">
            {isLoading ? (
              <div className="flex items-center justify-center py-8">
                <Loader2 className="w-8 h-8 animate-spin text-gray-400" />
              </div>
            ) : (
              <>
            {/* New Mission Form */}
            {isCreatingMission && (
              <div className="mb-4 p-3 bg-white dark:bg-botbot-dark rounded-lg shadow-sm border border-gray-200 dark:border-gray-700">
                <input
                  type="text"
                  value={newMissionName}
                  onChange={(e) => setNewMissionName(e.target.value)}
                  placeholder="Mission name..."
                  className="w-full px-3 py-2 bg-gray-50 dark:bg-botbot-darker text-gray-800 dark:text-white rounded-md border border-gray-300 dark:border-gray-600 focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple mb-2"
                  autoFocus
                  onKeyPress={(e) => e.key === 'Enter' && createMission()}
                />
                <div className="flex gap-2">
                  <button
                    onClick={createMission}
                    disabled={!newMissionName.trim() || isSaving}
                    className="flex-1 px-3 py-1.5 bg-green-500 hover:bg-green-600 disabled:bg-gray-400 text-white rounded-md transition-colors text-sm font-medium"
                  >
                    {isSaving ? (
                      <Loader2 className="w-4 h-4 inline mr-1 animate-spin" />
                    ) : (
                      <Check className="w-4 h-4 inline mr-1" />
                    )}
                    Create
                  </button>
                  <button
                    onClick={() => {
                      setIsCreatingMission(false);
                      setNewMissionName('');
                    }}
                    className="flex-1 px-3 py-1.5 bg-gray-500 hover:bg-gray-600 text-white rounded-md transition-colors text-sm font-medium"
                  >
                    <X className="w-4 h-4 inline mr-1" />
                    Cancel
                  </button>
                </div>
              </div>
            )}

            {/* Missions List */}
            <div className="space-y-2">
              {missions.length === 0 && !isCreatingMission ? (
                <div className="text-center py-8">
                  <Navigation2 className="w-12 h-12 mx-auto text-gray-400 dark:text-gray-600 mb-3" />
                  <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                    No missions yet
                  </p>
                  <button
                    onClick={() => setIsCreatingMission(true)}
                    className="px-4 py-2 bg-primary hover:bg-primary/90 text-white rounded-lg transition-colors text-sm font-medium"
                  >
                    <Plus className="w-4 h-4 inline mr-1" />
                    Create First Mission
                  </button>
                </div>
              ) : (
                missions.map((mission) => (
                  <div
                    key={mission.id}
                    onClick={() => setSelectedMission(mission)}
                    className={`p-3 rounded-lg border transition-all cursor-pointer ${
                      selectedMission?.id === mission.id
                        ? 'bg-primary/10 border-primary dark:border-botbot-purple'
                        : 'bg-white dark:bg-botbot-dark border-gray-200 dark:border-gray-700 hover:bg-gray-50 dark:hover:bg-botbot-darker'
                    }`}
                  >
                    {editingMissionId === mission.id ? (
                      <div className="space-y-2">
                        <input
                          type="text"
                          value={editingMissionName}
                          onChange={(e) => setEditingMissionName(e.target.value)}
                          className="w-full px-2 py-1 bg-gray-50 dark:bg-botbot-darker text-gray-800 dark:text-white rounded border border-gray-300 dark:border-gray-600 focus:outline-none focus:ring-1 focus:ring-primary dark:focus:ring-botbot-purple"
                          autoFocus
                          onKeyPress={(e) => {
                            if (e.key === 'Enter') updateMission(mission.id);
                            if (e.key === 'Escape') {
                              setEditingMissionId(null);
                              setEditingMissionName('');
                            }
                          }}
                          onClick={(e) => e.stopPropagation()}
                        />
                        <div className="flex gap-2">
                          <button
                            onClick={(e) => {
                              e.stopPropagation();
                              updateMission(mission.id);
                            }}
                            disabled={isSaving}
                            className="flex-1 px-2 py-1 bg-green-500 hover:bg-green-600 disabled:bg-gray-400 text-white rounded text-xs"
                          >
                            {isSaving ? (
                              <Loader2 className="w-3 h-3 inline animate-spin" />
                            ) : (
                              <Save className="w-3 h-3 inline" />
                            )}
                          </button>
                          <button
                            onClick={(e) => {
                              e.stopPropagation();
                              setEditingMissionId(null);
                              setEditingMissionName('');
                            }}
                            className="flex-1 px-2 py-1 bg-gray-500 hover:bg-gray-600 text-white rounded text-xs"
                          >
                            <X className="w-3 h-3 inline" />
                          </button>
                        </div>
                      </div>
                    ) : (
                      <>
                        <div className="flex items-center justify-between mb-1">
                          <h4 className="font-medium text-gray-800 dark:text-white">
                            {mission.name}
                          </h4>
                          {activeMissionId === mission.id ? (
                            <span className="px-2 py-0.5 bg-green-500 text-white text-xs rounded-full animate-pulse">
                              Active
                            </span>
                          ) : mission.is_active ? (
                            <span className="px-2 py-0.5 bg-yellow-500 text-white text-xs rounded-full">
                              Paused
                            </span>
                          ) : null}
                        </div>
                        <div className="space-y-1 mb-2">
                          <p className="text-xs text-gray-500 dark:text-gray-400">
                            Created {mission.created_at ? new Date(mission.created_at).toLocaleDateString() : 'Unknown'}
                          </p>
                          <p className="text-xs text-gray-600 dark:text-gray-300 flex items-center gap-1">
                            <Route className="w-3 h-3" />
                            {mission.waypoints?.length || 0} waypoints
                          </p>
                          {mission.map_name && (
                            <p className="text-xs text-gray-500 dark:text-gray-400 flex items-center gap-1">
                              <MapIcon className="w-3 h-3" />
                              {mission.map_name.replace(/\.db$/i, '').replace(/_/g, ' ')}
                            </p>
                          )}
                        </div>
                        <div className="flex gap-2 mt-2">
                          <button
                            onClick={(e) => {
                              e.stopPropagation();
                              setEditingMissionId(mission.id);
                              setEditingMissionName(mission.name);
                            }}
                            className="flex-1 px-3 py-1.5 bg-gray-500 hover:bg-gray-600 text-white rounded-md transition-colors text-sm font-medium flex items-center justify-center gap-1"
                          >
                            <Edit2 className="w-3 h-3" />
                            Edit
                          </button>
                          <button
                            onClick={(e) => {
                              e.stopPropagation();
                              if (confirm(`Delete mission "${mission.name}"?`)) {
                                deleteMission(mission.id);
                              }
                            }}
                            className="flex-1 px-3 py-1.5 bg-red-500 hover:bg-red-600 text-white rounded-md transition-colors text-sm font-medium flex items-center justify-center gap-1"
                          >
                            <Trash2 className="w-3 h-3" />
                            Delete
                          </button>
                        </div>
                      </>
                    )}
                  </div>
                ))
              )}
            </div>
              </>
            )}
          </div>
        </div>

        {/* Center - Main Mission Editor */}
        <div className="flex-1 bg-gray-100 dark:bg-botbot-darker p-2 sm:p-4 overflow-hidden flex flex-col">
          <Container className="w-full h-full flex flex-col" customContentClasses="p-0 flex flex-col h-full">
            <div className="flex flex-col h-full">
              {/* Mission Header */}
              <div className="flex items-center justify-between px-4 pt-4 pb-2">
                <h3 className="font-semibold text-gray-700 dark:text-gray-300 flex items-center gap-2">
                  <MapIcon className="w-5 h-5" />
                  {selectedMission ? `Mission: ${selectedMission.name}` : 'Mission Editor'}
                </h3>
                {activeMissionId && (
                  <div className="flex items-center gap-2 px-3 py-1 bg-green-500/20 text-green-600 dark:text-green-400 rounded-full text-sm">
                    <div className="w-2 h-2 bg-green-500 rounded-full animate-pulse" />
                    Mission Running
                  </div>
                )}
                {!activeMissionId && selectedMission?.is_active && (
                  <div className="flex items-center gap-2 px-3 py-1 bg-yellow-500/20 text-yellow-600 dark:text-yellow-400 rounded-full text-sm">
                    <div className="w-2 h-2 bg-yellow-500 rounded-full" />
                    Mission Paused
                  </div>
                )}
              </div>
              
              {/* Waypoint Toolbar */}
              {selectedMission && (
                <div className="px-4 pb-2">
                  <div className="bg-gray-100 dark:bg-botbot-dark rounded-lg p-3 flex items-center justify-between">
                    <div className="flex items-center gap-4">
                      <div className="flex items-center gap-2">
                        <Route className="w-4 h-4 text-gray-600 dark:text-gray-400" />
                        <span className="text-sm font-medium text-gray-700 dark:text-gray-300">
                          Waypoints: {currentWaypointCount}
                        </span>
                      </div>
                      
                      <div className="flex items-center gap-2">
                        <div className="w-3 h-3 bg-blue-500 rounded-full" />
                        <span className="text-xs text-gray-600 dark:text-gray-400">Pending</span>
                        <div className="w-3 h-3 bg-orange-500 rounded-full animate-pulse" />
                        <span className="text-xs text-gray-600 dark:text-gray-400">Active</span>
                        <div className="w-3 h-3 bg-green-500 rounded-full" />
                        <span className="text-xs text-gray-600 dark:text-gray-400">Reached</span>
                      </div>
                    </div>
                    
                    <div className="flex items-center gap-2">
                      <button
                        onClick={() => setShowNavPlan(!showNavPlan)}
                        className={`px-3 py-1.5 rounded-md transition-colors text-sm font-medium flex items-center gap-1 ${
                          showNavPlan 
                            ? 'bg-green-500 hover:bg-green-600 text-white' 
                            : 'bg-gray-500 hover:bg-gray-600 text-white'
                        }`}
                        title="Toggle navigation plan overlay"
                      >
                        <Route className="w-3 h-3" />
                        Nav Plan
                      </button>
                      {isSaving && (
                        <span className="text-xs text-gray-500 dark:text-gray-400 mr-2">
                          <Loader2 className="w-3 h-3 inline animate-spin mr-1" />
                          Saving waypoints...
                        </span>
                      )}
                      {activeMissionId === selectedMission.id ? (
                        <button
                          onClick={stopMission}
                          disabled={isSaving}
                          className="px-3 py-1.5 bg-red-500 hover:bg-red-600 disabled:bg-gray-400 text-white rounded-md transition-colors text-sm font-medium flex items-center gap-1"
                        >
                          <Square className="w-3 h-3" />
                          Stop
                        </button>
                      ) : (
                        <button
                          onClick={() => playMission(selectedMission.id)}
                          disabled={
                            currentWaypointCount === 0 ||
                            isSaving ||
                            !!(selectedMission.map_name && !isMapCompatible && isRobotConnected)
                          }
                          className={`px-3 py-1.5 rounded-md transition-colors text-sm font-medium flex items-center gap-1 ${
                            currentWaypointCount > 0 && !isSaving && (isMapCompatible || !selectedMission.map_name || !isRobotConnected)
                              ? 'bg-green-500 hover:bg-green-600 text-white'
                              : 'bg-gray-400 cursor-not-allowed text-gray-200'
                          }`}
                          title={
                            selectedMission.map_name && !isMapCompatible && isRobotConnected
                              ? 'Switch to the correct map first'
                              : currentWaypointCount > 0
                                ? (selectedMission.is_active ? 'Resume mission' : 'Start mission')
                                : 'Add waypoints first'
                          }
                        >
                          <Play className="w-3 h-3" />
                          {selectedMission.is_active ? 'Resume' : 'Start'}
                        </button>
                      )}
                    </div>
                  </div>
                </div>
              )}
              
              {/* Map Container */}
              <div className="flex-1 px-4 pb-4 min-h-0">
                {selectedMission ? (
                  <div className="w-full h-full relative overflow-hidden rounded-lg flex flex-col">
                    {/* Map Mismatch Warning Banner */}
                    {selectedMission.map_name && !isMapCompatible && isRobotConnected && (
                      <MissionMapWarningBanner
                        missionMapName={selectedMission.map_name}
                        currentMapName={currentMapName}
                        onSwitchMap={handleSwitchMap}
                        isLoading={isSwitchingMap}
                        className="mb-2 flex-shrink-0"
                      />
                    )}

                    {/* Map View - with dimming when map mismatched */}
                    <div className={`flex-1 min-h-0 ${
                      selectedMission.map_name && !isMapCompatible && isRobotConnected
                        ? 'opacity-50 pointer-events-none'
                        : ''
                    }`}>
                      <MissionMapView
                        key={selectedMission.id}  // Force re-render when mission changes
                        className="w-full h-full"
                        onWaypointsChange={handleWaypointsChange}
                        activeMissionId={activeMissionId}
                        onNavigationComplete={handleNavigationComplete}
                        initialWaypoints={transformedWaypoints}
                        showNavPlan={showNavPlan}
                      />
                    </div>
                  </div>
                ) : (
                  <div className="w-full h-full bg-white dark:bg-botbot-darkest rounded-lg flex items-center justify-center">
                    <div className="text-center">
                      <Navigation2 className="w-16 h-16 mx-auto text-gray-400 dark:text-gray-600 mb-4" />
                      <h3 className="text-lg font-medium text-gray-700 dark:text-gray-300 mb-2">
                        No Mission Selected
                      </h3>
                      <p className="text-sm text-gray-500 dark:text-gray-400 mb-4">
                        Select a mission from the left panel or create a new one to start planning waypoints
                      </p>
                      <button
                        onClick={() => setShowLeftPanel(true)}
                        className="sm:hidden px-4 py-2 bg-primary hover:bg-primary/90 text-white rounded-lg transition-colors text-sm font-medium"
                      >
                        Open Mission List
                      </button>
                    </div>
                  </div>
                )}
              </div>
            </div>
          </Container>
        </div>

        {/* Right Sidebar - Camera Feed & Controls */}
        <div className={`
          absolute sm:relative sm:translate-x-0 z-20 h-full right-0
          w-72 border-l border-gray-200 dark:border-black 
          bg-gray-50 dark:bg-botbot-darkest overflow-hidden flex flex-col
          transition-transform duration-300 ease-in-out
          ${showRightPanel ? 'translate-x-0' : 'translate-x-full'}
        `}>
          {/* Mobile Close Button */}
          <button
            onClick={() => setShowRightPanel(false)}
            className="sm:hidden absolute top-2 left-2 p-1 rounded-lg bg-gray-200 dark:bg-botbot-dark z-10"
          >
            <X className="w-4 h-4" />
          </button>

          {/* Camera Feed at Top */}
          <div className="p-2 border-b border-gray-200 dark:border-gray-700">
            <h3 className="text-sm font-semibold text-gray-700 dark:text-gray-300 mb-2">Camera View</h3>
            <div className="w-full h-48 rounded-lg overflow-hidden border border-gray-300 dark:border-gray-700">
              <RosCameraImg cameraType="camera" overlay="none" />
            </div>
          </div>

          {/* Robot Controls Below Camera */}
          <div className="flex-1 overflow-y-auto p-2">
            <Container className="w-full" title="Robot Controls">
              <div className="space-y-2">
                {/* Emergency button first */}
                <div className="w-full">
                  {isInEmergencyMode ? (
                    <RobotActionButton
                      className={emergencyOffBtnClasses}
                      action="emergencyOff"
                    />
                  ) : (
                    <RobotActionButton
                      className={emergencyOnBtnClasses}
                      action="emergencyOn"
                    />
                  )}
                </div>
                
                {/* Other action buttons in single column */}
                <div className="w-full space-y-2">
                  {/* Other action buttons */}
                  {(!isInEmergencyMode
                    ? robotActions
                    : robotActionsBeforeEmergency
                  ).map((action, index) => (
                    <RobotActionButton
                      key={index}
                      className={btnCustomClasses + " w-full"}
                      action={getToggleActionButtons(action)}
                    />
                  ))}
                </div>
              </div>
            </Container>
          </div>
        </div>
      </div>

      {/* Mobile Toggle Buttons */}
      <div className="sm:hidden flex justify-between p-2 border-t border-gray-200 dark:border-black bg-white dark:bg-botbot-dark">
        <button
          onClick={() => setShowLeftPanel(!showLeftPanel)}
          className="p-2 rounded-lg bg-gray-100 dark:bg-botbot-darker"
        >
          <Menu className="w-5 h-5" />
        </button>
        <button
          onClick={() => setShowRightPanel(!showRightPanel)}
          className="p-2 rounded-lg bg-gray-100 dark:bg-botbot-darker"
        >
          <Menu className="w-5 h-5" />
        </button>
      </div>
    </div>
  );
}