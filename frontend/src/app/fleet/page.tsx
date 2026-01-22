'use client';

import { useState, useEffect } from 'react';
import { useRouter } from 'next/navigation';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useLanguage } from '@/contexts/LanguageContext';
import { Database } from '@/types/database.types';
import Button from '@/components/ui/button';
import Popup from '@/components/ui/popup';
import InputField from '@/components/ui/input-field';
import { Bot, Plus, Star, Wifi, WifiOff, Trash2, Edit2, RefreshCw } from 'lucide-react';
import { auditLogger } from '@/utils/audit-logger';
import { getRobotProfileOptions } from '@/config/robot-profiles';
import { useRobotAvailability } from '@/hooks/useRobotAvailability';
import ConnectionStatusIndicator from '@/components/fleet/ConnectionStatusIndicator';

type Robot = Database['public']['Tables']['robots']['Row'];

export default function FleetPage() {
  const router = useRouter();
  const { supabase, user, loading: authLoading } = useSupabase();
  const { connection, connectionStatus, connectToRobotWithInfo, disconnectRobot } = useRobotConnection();
  const { t } = useLanguage();
  
  const [robots, setRobots] = useState<Robot[]>([]);
  const [loading, setLoading] = useState(true);
  const [showAddRobotModal, setShowAddRobotModal] = useState(false);
  const [editingRobot, setEditingRobot] = useState<Robot | null>(null);

  // Robot availability checking
  const { availability, isChecking, refreshAvailability } = useRobotAvailability(robots);
  
  // Delete confirmation modal states
  const [showDeleteModal, setShowDeleteModal] = useState(false);
  const [robotToDelete, setRobotToDelete] = useState<Robot | null>(null);
  const [isDeleting, setIsDeleting] = useState(false);
  
  // Form fields
  const [robotName, setRobotName] = useState('');
  const [robotAddress, setRobotAddress] = useState('');
  const [robotKey, setRobotKey] = useState('');
  const [robotType, setRobotType] = useState('Go2-R1');
  const [formError, setFormError] = useState('');
  const [saving, setSaving] = useState(false);

  // Set page title
  useEffect(() => {
    document.title = 'Fleet - BotBot';
  }, []);

  // Load robots
  useEffect(() => {
    // Don't do anything while auth is still loading
    if (authLoading) {
      return;
    }
    
    // Only redirect after auth has loaded and there's no user
    if (!user) {
      router.push('/');
      return;
    }
    
    // If we have a user but no supabase client yet, wait
    if (!supabase) {
      return;
    }
    
    loadRobots();
  }, [user, supabase, authLoading, router]);

  const loadRobots = async () => {
    if (!supabase) return;
    
    try {
      const { data, error } = await supabase
        .from('robots')
        .select('*')
        .order('created_at', { ascending: false });

      if (error) throw error;
      
      // Sort robots to show favorites first
      const sortedRobots = (data || []).sort((a, b) => {
        if (a.is_favorite && !b.is_favorite) return -1;
        if (!a.is_favorite && b.is_favorite) return 1;
        return 0;
      });
      
      setRobots(sortedRobots);
    } catch (error) {
      console.error('Error loading robots:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleAddRobot = () => {
    setEditingRobot(null);
    setRobotName('');
    setRobotAddress('');
    setRobotKey('');
    setRobotType('Go2-R1');
    setFormError('');
    setShowAddRobotModal(true);
  };

  const handleEditRobot = (robot: Robot) => {
    setEditingRobot(robot);
    setRobotName(robot.name);
    setRobotAddress(robot.address);
    setRobotKey(robot.key || '');
    setRobotType(robot.type);
    setFormError('');
    setShowAddRobotModal(true);
  };

  const handleSaveRobot = async () => {
    if (!supabase) {
      setFormError('Database connection not available');
      return;
    }
    
    if (!robotName.trim()) {
      setFormError('Robot name is required');
      return;
    }
    
    if (!robotAddress.trim()) {
      setFormError('Robot address is required');
      return;
    }

    setSaving(true);
    setFormError('');

    try {
      if (editingRobot) {
        // Update existing robot
        const { error } = await supabase
          .from('robots')
          .update({
            name: robotName.trim(),
            address: robotAddress.trim(),
            key: robotKey.trim() || null,
            type: robotType,
            updated_at: new Date().toISOString(),
          })
          .eq('id', editingRobot.id);

        if (error) throw error;
        
        // Log robot update
        await auditLogger.log({
          event_type: 'system',
          event_action: 'robot_updated',
          robot_id: editingRobot.id,
          robot_name: robotName.trim(),
          event_details: {
            old_name: editingRobot.name,
            old_address: editingRobot.address,
            new_name: robotName.trim(),
            new_address: robotAddress.trim()
          }
        });
      } else {
        // Create new robot
        const { data: newRobot, error } = await supabase
          .from('robots')
          .insert({
            user_id: user!.id,
            name: robotName.trim(),
            address: robotAddress.trim(),
            key: robotKey.trim() || null,
            type: robotType,
            is_favorite: robots.length === 0, // First robot is automatically favorite
          })
          .select()
          .single();

        if (error) throw error;
        
        // Log robot addition
        if (newRobot) {
          await auditLogger.log({
            event_type: 'system',
            event_action: 'robot_added',
            robot_id: newRobot.id,
            robot_name: newRobot.name,
            event_details: {
              address: newRobot.address,
              type: newRobot.type
            }
          });
        }
      }

      await loadRobots();
      setShowAddRobotModal(false);
    } catch (error: any) {
      setFormError(error.message || 'Failed to save robot');
    } finally {
      setSaving(false);
    }
  };

  const handleDeleteRobot = async (robot: Robot) => {
    setRobotToDelete(robot);
    setShowDeleteModal(true);
  };

  const confirmDeleteRobot = async () => {
    if (!supabase || !robotToDelete) return;
    
    setIsDeleting(true);
    
    try {
      // If deleting the connected robot, disconnect first
      if (connection.connectedRobot?.id === robotToDelete.id) {
        disconnectRobot();
      }

      const { error } = await supabase
        .from('robots')
        .delete()
        .eq('id', robotToDelete.id);

      if (error) throw error;
      
      // Log robot deletion
      await auditLogger.log({
        event_type: 'system',
        event_action: 'robot_deleted',
        robot_id: robotToDelete.id,
        robot_name: robotToDelete.name,
        event_details: {
          address: robotToDelete.address,
          type: robotToDelete.type
        }
      });
      
      await loadRobots();
      setShowDeleteModal(false);
      setRobotToDelete(null);
    } catch (error) {
      console.error('Error deleting robot:', error);
      // You might want to show an error message here
    } finally {
      setIsDeleting(false);
    }
  };

  const handleSetFavorite = async (robot: Robot) => {
    if (!supabase) return;
    
    try {
      // First, remove favorite status from all other robots
      const { error: removeError } = await supabase
        .from('robots')
        .update({ is_favorite: false })
        .neq('id', robot.id);

      if (removeError) throw removeError;

      // Then set the new favorite
      const { error } = await supabase
        .from('robots')
        .update({ is_favorite: true })
        .eq('id', robot.id);

      if (error) throw error;
      await loadRobots();
    } catch (error) {
      console.error('Error setting favorite:', error);
    }
  };

  const handleConnectRobot = async (robot: Robot) => {
    try {
      await connectToRobotWithInfo(robot);
    } catch (error) {
      console.error('Failed to connect to robot:', error);
    }
  };

  const handleDisconnectRobot = () => {
    disconnectRobot();
  };

  if (authLoading || loading) {
    return (
      <div className="w-full h-full flex items-center justify-center">
        <div className="text-gray-600 dark:text-gray-400">
          {authLoading ? t('fleet', 'checkingAuth') : t('fleet', 'loadingRobots')}
        </div>
      </div>
    );
  }

  return (
    <div className="w-full h-[calc(100vh-56px-24px)] flex flex-col md:flex-row items-stretch justify-between relative px-1 overflow-hidden">
      {/* Main content area */}
      <div className="w-full h-full flex flex-col pt-2 px-1 overflow-hidden">
        <div className="w-full h-full bg-white/5 backdrop-blur-sm rounded-lg p-4 overflow-y-auto">
          <div className="max-w-6xl mx-auto">
            {/* Header with Add and Refresh buttons */}
            <div className="flex justify-between items-center mb-6">
              <button
                onClick={refreshAvailability}
                disabled={isChecking}
                className="inline-flex items-center justify-center rounded-md px-3 py-1.5 text-sm font-medium bg-gray-200 dark:bg-gray-700 text-gray-700 dark:text-gray-200 hover:bg-gray-300 dark:hover:bg-gray-600 disabled:opacity-50 disabled:cursor-not-allowed focus:outline-none transition-colors"
              >
                <RefreshCw className={`mr-1 h-4 w-4 ${isChecking ? 'animate-spin' : ''}`} />
                {isChecking ? t('fleet', 'checking') : t('fleet', 'refresh')}
              </button>
              <button
                onClick={handleAddRobot}
                className="inline-flex items-center justify-center rounded-md px-3 py-1.5 text-sm font-medium bg-primary dark:bg-botbot-purple text-white hover:bg-primary/90 dark:hover:bg-botbot-purple/90 focus:outline-none transition-colors"
              >
                <Plus className="mr-1 h-4 w-4" />
                {t('fleet', 'addRobot')}
              </button>
            </div>

            {robots.length === 0 ? (
              <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-sm p-8">
                <div className="flex flex-col items-center justify-center py-12">
                  <Bot className="w-24 h-24 text-gray-300 dark:text-gray-600 mb-4" />
                  <h2 className="text-xl font-semibold text-gray-600 dark:text-gray-400 mb-2">
                    {t('fleet', 'noRobotsTitle')}
                  </h2>
                  <p className="text-gray-500 dark:text-gray-500 mb-6">
                    {t('fleet', 'noRobotsDescription')}
                  </p>
                  <button
                    onClick={handleAddRobot}
                    className="inline-flex items-center justify-center rounded-md px-4 py-2 text-sm font-medium bg-primary dark:bg-botbot-purple text-white hover:bg-primary/90 dark:hover:bg-botbot-purple/90 focus:outline-none transition-colors"
                  >
                    <Plus className="mr-2 h-4 w-4" />
                    {t('fleet', 'addNewRobot')}
                  </button>
                </div>
              </div>
            ) : (
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                {robots.map((robot) => {
                  const isConnected = connection.connectedRobot?.id === robot.id && connection.online;
                  const isConnecting = connection.connectedRobot?.id === robot.id && connectionStatus === 'connecting';
                  const robotAvailability = availability.get(robot.id);

                  return (
                    <div
                      key={robot.id}
                      className={`relative bg-white dark:bg-botbot-darker rounded-lg shadow-md p-6 transition-all duration-200 ${
                        isConnected ? 'ring-2 ring-green-500' : ''
                      }`}
                    >
                      {/* Status and Favorite indicators */}
                      <div className="absolute top-4 right-4 flex items-center gap-2">
                        <ConnectionStatusIndicator
                          status={robotAvailability?.status || 'checking'}
                          lastChecked={robotAvailability?.lastChecked}
                          size="sm"
                        />
                        {robot.is_favorite && (
                          <Star className="w-5 h-5 text-yellow-500 fill-yellow-500" />
                        )}
                      </div>

                      {/* Robot icon and name */}
                      <div className="flex items-center mb-4">
                        <div className="w-12 h-12 rounded-full bg-purple-100 dark:bg-purple-900/20 flex items-center justify-center mr-3">
                          <span className="text-lg font-semibold text-purple-600 dark:text-purple-400">
                            {robot.name.slice(0, 2).toUpperCase()}
                          </span>
                        </div>
                        <div>
                          <h3 className="text-lg font-semibold text-gray-800 dark:text-white">
                            {robot.name}
                          </h3>
                          <p className="text-sm text-gray-500 dark:text-gray-400">{robot.type}</p>
                        </div>
                      </div>

                      {/* Connection status */}
                      <div className="flex items-center mb-4">
                        {isConnected ? (
                          <>
                            <Wifi className="w-4 h-4 text-green-500 mr-2" />
                            <span className="text-sm text-green-600 dark:text-green-400">{t('fleet', 'connected')}</span>
                          </>
                        ) : isConnecting ? (
                          <>
                            <div className="w-4 h-4 border-2 border-purple-500 border-t-transparent rounded-full animate-spin mr-2" />
                            <span className="text-sm text-purple-600 dark:text-purple-400">{t('fleet', 'connecting')}</span>
                          </>
                        ) : (
                          <>
                            <WifiOff className="w-4 h-4 text-gray-400 mr-2" />
                            <span className="text-sm text-gray-500 dark:text-gray-400">{t('fleet', 'disconnected')}</span>
                          </>
                        )}
                      </div>

                      {/* Robot address */}
                      <p className="text-xs text-gray-500 dark:text-gray-400 mb-4 truncate">
                        {robot.address}
                      </p>

                      {/* Action buttons */}
                      <div className="flex gap-2">
                        {isConnected ? (
                          <button
                            onClick={handleDisconnectRobot}
                            className="flex-1 px-3 py-2 bg-red-500 text-white rounded-md text-sm font-medium hover:bg-red-600 transition-colors flex items-center justify-center"
                          >
                            <WifiOff className="w-4 h-4 mr-1.5" />
                            {t('fleet', 'disconnect')}
                          </button>
                        ) : isConnecting ? (
                          <button
                            disabled
                            className="flex-1 px-3 py-2 bg-purple-500 text-white rounded-md text-sm font-medium opacity-75 cursor-not-allowed flex items-center justify-center"
                          >
                            <div className="w-4 h-4 border-2 border-white border-t-transparent rounded-full animate-spin mr-1.5" />
                            {t('fleet', 'connecting')}
                          </button>
                        ) : connection.connectedRobot && connection.connectedRobot.id !== robot.id ? (
                          <button
                            onClick={() => handleConnectRobot(robot)}
                            className="flex-1 px-3 py-2 bg-green-500 text-white rounded-md text-sm font-medium hover:bg-green-600 transition-colors flex items-center justify-center"
                          >
                            <Wifi className="w-4 h-4 mr-1.5" />
                            {t('fleet', 'switchTo')}
                          </button>
                        ) : (
                          <button
                            onClick={() => handleConnectRobot(robot)}
                            className="flex-1 px-3 py-2 bg-green-500 text-white rounded-md text-sm font-medium hover:bg-green-600 transition-colors flex items-center justify-center"
                          >
                            <Wifi className="w-4 h-4 mr-1.5" />
                            {t('fleet', 'connect')}
                          </button>
                        )}
                        
                        {!robot.is_favorite && (
                          <button
                            onClick={() => handleSetFavorite(robot)}
                            className="p-2 text-gray-600 dark:text-gray-400 hover:text-yellow-500 transition-colors"
                            title={t('fleet', 'setAsFavorite')}
                          >
                            <Star className="w-4 h-4" />
                          </button>
                        )}
                        
                        <button
                          onClick={() => handleEditRobot(robot)}
                          className="p-2 text-gray-600 dark:text-gray-400 hover:text-blue-500 transition-colors"
                          title={t('fleet', 'editRobotTooltip')}
                        >
                          <Edit2 className="w-4 h-4" />
                        </button>
                        
                        <button
                          onClick={() => handleDeleteRobot(robot)}
                          className="p-2 text-gray-600 dark:text-gray-400 hover:text-red-500 transition-colors"
                          title={t('fleet', 'deleteRobotTooltip')}
                        >
                          <Trash2 className="w-4 h-4" />
                        </button>
                      </div>
                    </div>
                  );
                })}
              </div>
            )}
          </div>
        </div>
      </div>

      {/* Add/Edit Robot Modal */}
      <Popup
        isOpen={showAddRobotModal}
        onClose={() => setShowAddRobotModal(false)}
        title={editingRobot ? t('fleet', 'editRobot') : t('fleet', 'addNewRobot')}
        className="w-full max-w-md"
      >
        <div className="space-y-4">
          {/* Robot Name */}
          <div>
            <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-2">
              {t('fleet', 'robotName')}
            </label>
            <InputField
              value={robotName}
              onChange={(e) => setRobotName(e.target.value)}
              placeholder={t('fleet', 'robotNamePlaceholder')}
              className="w-full"
            />
          </div>

          {/* Robot Address */}
          <div>
            <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-2">
              {t('fleet', 'address')}
            </label>
            <InputField
              value={robotAddress}
              onChange={(e) => setRobotAddress(e.target.value)}
              placeholder={t('fleet', 'addressPlaceholder')}
              className="w-full"
            />
            <p className="text-xs text-gray-500 dark:text-gray-400 mt-1">
              {t('fleet', 'addressHelpText')}
            </p>
          </div>

          {/* Robot Key */}
          <div>
            <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-2">
              {t('fleet', 'key')}
            </label>
            <InputField
              value={robotKey}
              onChange={(e) => setRobotKey(e.target.value)}
              placeholder={t('fleet', 'keyPlaceholder')}
              className="w-full"
            />
            <p className="text-xs text-gray-500 dark:text-gray-400 mt-1">
              {t('fleet', 'keyHelpText')}
            </p>
          </div>

          {/* Robot Type */}
          <div>
            <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-2">
              {t('fleet', 'robotType')}
            </label>
            <select
              value={robotType}
              onChange={(e) => setRobotType(e.target.value)}
              className="w-full p-2 border border-gray-300 dark:border-botbot-darker rounded-md 
                         focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent
                         bg-white dark:bg-botbot-dark text-gray-800 dark:text-gray-100"
            >
              {getRobotProfileOptions().map((option) => (
                <option key={option.value} value={option.value}>
                  {option.label} - {option.description}
                </option>
              ))}
            </select>
          </div>

          {/* Error message */}
          {formError && (
            <div className="text-red-500 text-sm">{formError}</div>
          )}

          {/* Buttons */}
          <div className="flex gap-3 pt-4">
            <Button
              label={t('fleet', 'cancel')}
              colorPalette="white"
              onClick={() => setShowAddRobotModal(false)}
              customClasses="flex-1 mt-0"
            />
            <Button
              label={saving ? t('fleet', 'saving') : (editingRobot ? t('fleet', 'update') : t('fleet', 'addRobot'))}
              colorPalette="default"
              onClick={handleSaveRobot}
              customClasses={`flex-1 mt-0 ${saving ? 'opacity-50 cursor-not-allowed' : ''}`}
            />
          </div>
        </div>
      </Popup>

      {/* Delete Confirmation Modal */}
      <Popup
        isOpen={showDeleteModal}
        onClose={() => {
          if (!isDeleting) {
            setShowDeleteModal(false);
            setRobotToDelete(null);
          }
        }}
        title=""
        className="w-full max-w-md"
      >
        <div className="flex flex-col items-center text-center space-y-6 py-4">
          {/* Warning Icon */}
          <div className="w-20 h-20 rounded-full bg-red-100 dark:bg-red-900/20 flex items-center justify-center">
            <Trash2 className="w-10 h-10 text-red-600 dark:text-red-400" />
          </div>
          
          {/* Title and Message */}
          <div className="space-y-2">
            <h3 className="text-xl font-semibold text-gray-800 dark:text-white">
              {t('fleet', 'deleteRobotTitle')}
            </h3>
            <p className="text-gray-600 dark:text-gray-400 max-w-sm">
              {t('fleet', 'deleteRobotMessage')} <span className="font-semibold text-gray-800 dark:text-white">"{robotToDelete?.name}"</span>?
            </p>
            <p className="text-sm text-gray-500 dark:text-gray-500">
              {t('fleet', 'deleteRobotWarning')}
            </p>
          </div>
          
          {/* Robot Details */}
          {robotToDelete && (
            <div className="w-full bg-gray-50 dark:bg-botbot-darker/50 rounded-lg p-4 space-y-2">
              <div className="flex items-center justify-between text-sm">
                <span className="text-gray-600 dark:text-gray-400">{t('fleet', 'type')}:</span>
                <span className="text-gray-800 dark:text-gray-200">{robotToDelete.type}</span>
              </div>
              <div className="flex items-center justify-between text-sm">
                <span className="text-gray-600 dark:text-gray-400">{t('fleet', 'address')}:</span>
                <span className="text-gray-800 dark:text-gray-200 truncate max-w-[200px]">{robotToDelete.address}</span>
              </div>
              {robotToDelete.is_favorite && (
                <div className="flex items-center justify-center pt-2">
                  <Star className="w-4 h-4 text-yellow-500 fill-yellow-500 mr-1" />
                  <span className="text-sm text-yellow-600 dark:text-yellow-400">{t('fleet', 'favoriteRobot')}</span>
                </div>
              )}
            </div>
          )}
          
          {/* Action Buttons */}
          <div className="flex gap-3 w-full pt-2">
            <Button
              label={t('fleet', 'cancel')}
              colorPalette="white"
              onClick={() => {
                if (!isDeleting) {
                  setShowDeleteModal(false);
                  setRobotToDelete(null);
                }
              }}
              customClasses={`flex-1 mt-0 ${isDeleting ? 'opacity-50 cursor-not-allowed' : ''}`}
            />
            <Button
              label={isDeleting ? t('fleet', 'deleting') : t('fleet', 'delete')}
              colorPalette="warning"
              onClick={confirmDeleteRobot}
              customClasses={`flex-1 mt-0 ${isDeleting ? 'opacity-50 cursor-not-allowed' : ''}`}
            />
          </div>
        </div>
      </Popup>
    </div>
  );
} 