'use client';

import React, { useState, useCallback, useEffect, useMemo } from 'react';
import Image from 'next/image';
import { useRouter } from 'next/navigation';
import NavMenu from './nav-menu';
import { useHeader } from '../contexts/HeaderContext';
import { useTheme } from '@/contexts/ThemeContext';
import { Bot, Battery, Wifi, WifiOff, Loader2, Clock, UploadCloud, DownloadCloud, Smartphone, Radio, Globe, ChevronDown, Plus, Star, Check, Zap, TrendingDown, AlertTriangle, Activity, Signal, AlertCircle } from 'lucide-react';
import { useRobotConnection } from '../contexts/RobotConnectionContext';
import useThrottledBatteryState from '@/hooks/ros/useThrottledBatteryState';
import useNetworkMetrics from '@/hooks/ros/useNetworkMetrics';
import useNetworkModeStatus from '@/hooks/ros/useNetworkModeStatus';
import useBatteryTimeEstimator from '@/hooks/useBatteryTimeEstimator';
import { formatLatency, formatDataRate } from '@/utils/format-utils';
import RobotConnectionPopup from './robot-connection-popup';
import { useLanguage } from '@/contexts/LanguageContext';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { Database } from '@/types/database.types';
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from '@/components/ui/dropdown-menu';

type UserProfile = Database['public']['Tables']['user_profiles']['Row'];
type Robot = Database['public']['Tables']['robots']['Row'];

export default function RobotHeader() {
  const router = useRouter();
  const { themeColor } = useTheme();
  const { connection, connectionStatus, lastError, connectToRobotWithInfo, disconnectRobot } = useRobotConnection();
  const { extrasBarVisible, setExtrasBarVisible } = useHeader();
  const batteryState = useThrottledBatteryState(); // Using throttled version to reduce CPU cycles
  const batteryEstimate = useBatteryTimeEstimator(batteryState);
  const networkMetrics = useNetworkMetrics(1000); // Update every second
  const networkModeStatus = useNetworkModeStatus();
  const { t } = useLanguage();
  const { user, supabase } = useSupabase();
  
  // User profile state
  const [userProfile, setUserProfile] = useState<UserProfile | null>(null);

  // Dark mode detection
  const [isDarkMode, setIsDarkMode] = useState(false);

  // Robot list for dropdown
  const [robots, setRobots] = useState<Robot[]>([]);
  const [loadingRobots, setLoadingRobots] = useState(false);
  
  // Easter egg states
  const [_clickCount, setClickCount] = useState(0);
  const [clickTimes, setClickTimes] = useState<number[]>([]);
  const [showEasterEgg, setShowEasterEgg] = useState(false);

  // Connection popup state
  const [showConnectionPopup, setShowConnectionPopup] = useState(false);
  
  // Connection status animation
  const [showConnectionStatus, setShowConnectionStatus] = useState(false);
  const [connectionStatusMessage, setConnectionStatusMessage] = useState('');
  const [isConnectionError, setIsConnectionError] = useState(false);

  // Detect dark mode
  useEffect(() => {
    const checkDarkMode = () => {
      setIsDarkMode(document.documentElement.classList.contains('dark'));
    };
    
    checkDarkMode();
    
    // Watch for theme changes
    const observer = new MutationObserver(checkDarkMode);
    observer.observe(document.documentElement, { 
      attributes: true, 
      attributeFilter: ['class'] 
    });
    
    return () => observer.disconnect();
  }, []);

  // Fetch user profile on mount
  useEffect(() => {
    const fetchUserProfile = async () => {
      if (!user || !supabase) return;

      try {
        const { data: profile } = await supabase
          .from('user_profiles')
          .select('*')
          .eq('user_id', user.id)
          .single();

        if (profile) {
          setUserProfile(profile);
        }
      } catch (error) {
        console.error('Error fetching user profile:', error);
      }
    };

    fetchUserProfile();
  }, [user, supabase]);

  // Fetch robots for dropdown
  useEffect(() => {
    const fetchRobots = async () => {
      if (!user || !supabase) return;

      setLoadingRobots(true);
      try {
        const { data, error } = await supabase
          .from('robots')
          .select('*')
          .order('is_favorite', { ascending: false })
          .order('name', { ascending: true });

        if (error) throw error;

        setRobots(data || []);
      } catch (error) {
        console.error('Error fetching robots:', error);
      } finally {
        setLoadingRobots(false);
      }
    };

    fetchRobots();
  }, [user, supabase]);

  // Monitor connection status changes
  useEffect(() => {
    // When connection status changes, show a status indicator
    if (connectionStatus === 'connected') {
      const robotName = connection.connectedRobot?.name || 'robot';
      setConnectionStatusMessage(`Conectado ao robô ${robotName}`);
      setIsConnectionError(false);
      setShowConnectionStatus(true);
    } else if (connectionStatus === 'error') {
      setConnectionStatusMessage(lastError || 'Falha na conexão com o robô');
      setIsConnectionError(true);
      setShowConnectionStatus(true);
    } else if (connectionStatus === 'connecting') {
      setConnectionStatusMessage('Conectando ao robô...');
      setIsConnectionError(false);
      setShowConnectionStatus(true);
    }
    
    // Hide after 3 seconds if not connecting
    if (connectionStatus !== 'connecting') {
      const timer = setTimeout(() => {
        setShowConnectionStatus(false);
      }, 3000);
      
      return () => clearTimeout(timer);
    }
  }, [connectionStatus, lastError, connection.online]);

  // Prevent default behavior on logo
  useEffect(() => {
    const logoElement = document.getElementById('logo-easter-egg');
    if (logoElement) {
      const preventEvent = (e: Event) => {
        e.preventDefault();
      };
      
      logoElement.addEventListener('mousedown', preventEvent);
      logoElement.addEventListener('focus', preventEvent);
      
      return () => {
        logoElement.removeEventListener('mousedown', preventEvent);
        logoElement.removeEventListener('focus', preventEvent);
      };
    }
  }, []);

  const imgPath = `/botbot-logo${isDarkMode ? '-white' : ''}.png`;

  // Easter egg click handler
  const handleLogoClick = useCallback(() => {
    const currentTime = Date.now();
    const newClickTimes = [...clickTimes, currentTime].filter(
      time => currentTime - time < 10000 // Only keep clicks within last 10 seconds
    );
    
    setClickTimes(newClickTimes);
    setClickCount(newClickTimes.length);

    if (newClickTimes.length >= 5) {
      setShowEasterEgg(true);
      setClickTimes([]);
      setClickCount(0);
      
      // Hide the easter egg after 4 seconds
      setTimeout(() => {
        setShowEasterEgg(false);
      }, 4000);
    }
  }, [clickTimes]);

  // Navigate to fleet page
  const openFleetPage = () => {
    router.push('/fleet');
  };

  // Handle robot connection from dropdown
  const handleConnectRobot = async (robot: Robot) => {
    try {
      await connectToRobotWithInfo(robot);
    } catch (error) {
      console.error('Failed to connect to robot:', error);
    }
  };

  // Handle disconnect
  const handleDisconnectRobot = () => {
    disconnectRobot();
  };

  // Calculate battery percentage with memoization to prevent unnecessary re-renders
  const batteryPercentage = useMemo(() => {
    return connection.online ? Math.round(batteryState.percentage * 100) : 0;
  }, [connection.online, batteryState.percentage]);

  // Get connection icon based on network mode and connection status
  const getConnectionIcon = () => {
    // If not connected to robot, show connection status icons
    if (connectionStatus === 'connecting') {
      return <Loader2 className="h-5 w-5 text-purple-600 dark:text-purple-400 animate-spin" />;
    }
    
    if (connectionStatus !== 'connected') {
      return <WifiOff className="h-5 w-5 text-red-500" />;
    }
    
    // When connected, show network mode icon
    switch (networkModeStatus.mode) {
      case 'wifi':
        return (
          <div className="flex items-center space-x-1">
            <Wifi className="h-5 w-5 text-green-600 dark:text-green-500" />
            {networkModeStatus.ssid && (
              <span className="text-xs text-gray-600 dark:text-gray-400">
                {networkModeStatus.ssid}
              </span>
            )}
          </div>
        );
      case '4g':
        return (
          <div className="flex items-center space-x-1">
            <Smartphone className="h-5 w-5 text-blue-600 dark:text-blue-500" />
            <span className="text-xs text-gray-600 dark:text-gray-400">4G</span>
          </div>
        );
      case 'hotspot':
        return (
          <div className="flex items-center space-x-1">
            <Radio className="h-5 w-5 text-orange-600 dark:text-orange-500" />
            {networkModeStatus.ssid && (
              <span className="text-xs text-gray-600 dark:text-gray-400">
                {networkModeStatus.ssid}
              </span>
            )}
          </div>
        );
      case 'offline':
      default:
        return <Globe className="h-5 w-5 text-gray-500 dark:text-gray-400" />;
    }
  };

  // Get connection status text with robot name
  const getConnectionText = () => {
    if (connection.connectedRobot && connection.online) {
      return `Connected to ${connection.connectedRobot.name}`;
    }
    
    switch (connectionStatus) {
      case 'connected':
        return t('robotHeader', 'connected');
      case 'connecting':
        return t('robotHeader', 'connecting');
      case 'error':
      case 'idle':
      default:
        return t('robotHeader', 'disconnected');
    }
  };

  // Get connection text color
  const getConnectionTextClass = () => {
    switch (connectionStatus) {
      case 'connected':
        return '';
      case 'connecting':
        return 'text-purple-600 dark:text-purple-400';
      case 'error':
      case 'idle':
      default:
        return 'text-red-500';
    }
  };

  return (
    <>
      <header className="px-3 sm:px-4 lg:px-6 py-1 flex flex-row items-center justify-between">
        {/* Left section: Connection info and indicators */}
        <div className="flex flex-row items-center justify-start flex-1">
          <DropdownMenu>
            <DropdownMenuTrigger className="flex items-center focus:outline-none group">
              <span className={`heading-text text-start whitespace-nowrap group-hover:underline ${getConnectionTextClass()}`}>
                {getConnectionText()}
              </span>
              <ChevronDown className="w-4 h-4 text-gray-500 dark:text-gray-400 ml-1 transition-transform duration-200 group-data-[state=open]:rotate-180" />
            </DropdownMenuTrigger>

            <DropdownMenuContent className="w-72 sm:w-80 bg-white dark:bg-botbot-darker border border-gray-200 dark:border-gray-700 shadow-xl" align="start" sideOffset={8}>
                <DropdownMenuLabel className="text-xs text-gray-500 dark:text-gray-400 uppercase tracking-wide px-3 py-2">
                  Select Robot
                </DropdownMenuLabel>

                {loadingRobots ? (
                  <div className="flex items-center justify-center py-4">
                    <Loader2 className="w-5 h-5 animate-spin text-purple-600 dark:text-purple-400" />
                  </div>
                ) : robots.length === 0 ? (
                  <div className="px-3 py-4 text-center">
                    <p className="text-sm text-gray-500 dark:text-gray-400 mb-3">
                      No robots configured
                    </p>
                    <button
                      onClick={openFleetPage}
                      className="inline-flex items-center justify-center px-3 py-1.5 text-xs font-medium bg-purple-600 dark:bg-purple-500 text-white rounded-md hover:bg-purple-700 dark:hover:bg-purple-600 transition-colors"
                    >
                      <Plus className="w-3 h-3 mr-1" />
                      Add Robot
                    </button>
                  </div>
                ) : (
                  <>
                    <div className="max-h-80 overflow-y-auto overflow-x-hidden">
                      {robots.map((robot) => {
                        const isConnected = connection.connectedRobot?.id === robot.id && connection.online;
                        const isConnecting = connection.connectedRobot?.id === robot.id && connectionStatus === 'connecting';

                        return (
                          <DropdownMenuItem
                            key={robot.id}
                            className={`px-3 py-3 cursor-pointer transition-all duration-200 ${
                              isConnected
                                ? 'bg-green-50 dark:bg-green-900/20 hover:bg-green-100 dark:hover:bg-green-900/30'
                                : 'hover:bg-gray-50 dark:hover:bg-botbot-dark/50'
                            } focus:bg-gray-50 dark:focus:bg-botbot-dark/50`}
                            onClick={() => {
                              if (!isConnected && !isConnecting) {
                                handleConnectRobot(robot);
                              } else if (isConnected) {
                                handleDisconnectRobot();
                              }
                            }}
                          >
                            <div className="flex items-center justify-between w-full gap-2">
                              <div className="flex items-center gap-3 min-w-0 flex-1">
                                <div className={`w-10 h-10 rounded-full flex items-center justify-center flex-shrink-0 ${
                                  isConnected
                                    ? 'bg-green-600 dark:bg-green-500'
                                    : 'bg-purple-100 dark:bg-purple-900/30'
                                }`}>
                                  {isConnected ? (
                                    <Wifi className="w-5 h-5 text-white" />
                                  ) : (
                                    <span className="text-xs font-semibold text-purple-600 dark:text-purple-400">
                                      {robot.name.slice(0, 2).toUpperCase()}
                                    </span>
                                  )}
                                </div>
                                <div className="min-w-0 flex-1">
                                  <div className="flex items-center gap-2">
                                    <p className={`text-sm font-medium truncate ${
                                      isConnected
                                        ? 'text-green-900 dark:text-green-300'
                                        : 'text-gray-900 dark:text-white'
                                    }`}>
                                      {robot.name}
                                    </p>
                                    {robot.is_favorite && (
                                      <Star className="w-3 h-3 text-yellow-500 fill-yellow-500 flex-shrink-0" />
                                    )}
                                  </div>
                                  <p className="text-xs text-gray-500 dark:text-gray-400 mt-0.5">
                                    {robot.type}
                                  </p>
                                  <p className="text-xs text-gray-400 dark:text-gray-500 truncate mt-0.5 pr-2">
                                    {robot.address}
                                  </p>
                                </div>
                              </div>

                              <div className="flex items-center flex-shrink-0">
                                {isConnected ? (
                                  <div className="flex items-center gap-2 bg-green-600 dark:bg-green-500 text-white px-2.5 py-1 rounded-full">
                                    <Check className="w-4 h-4" />
                                    <span className="text-xs font-medium">Connected</span>
                                  </div>
                                ) : isConnecting ? (
                                  <div className="flex items-center gap-2 bg-purple-600 dark:bg-purple-500 text-white px-2.5 py-1 rounded-full">
                                    <Loader2 className="w-4 h-4 animate-spin" />
                                    <span className="text-xs font-medium">Connecting</span>
                                  </div>
                                ) : null}
                              </div>
                            </div>
                          </DropdownMenuItem>
                        );
                      })}
                    </div>

                    <DropdownMenuSeparator className="bg-gray-200 dark:bg-gray-700" />

                    <DropdownMenuItem
                      className="px-3 py-2 cursor-pointer hover:bg-gray-50 dark:hover:bg-botbot-dark/50 focus:bg-gray-50 dark:focus:bg-botbot-dark/50"
                      onClick={openFleetPage}
                    >
                      <div className="flex items-center justify-center w-full">
                        <Bot className="w-4 h-4 mr-2 text-purple-600 dark:text-purple-400" />
                        <span className="text-sm font-medium text-purple-600 dark:text-purple-400">
                          Manage Fleet
                        </span>
                      </div>
                    </DropdownMenuItem>
                  </>
                )}
            </DropdownMenuContent>
          </DropdownMenu>

          {/* All indicators - only show on xl screens */}
          {connection.online && (
            <div className="hidden xl:flex items-center ml-6 space-x-4">
              {/* Battery indicator with dropdown - fixed width */}
              <DropdownMenu>
                <DropdownMenuTrigger className="flex items-center focus:outline-none group cursor-pointer">
                  <Battery className={`h-4 w-4 mr-1 transition-colors ${
                    batteryEstimate.isCharging ? 'text-green-600 dark:text-green-400' :
                    batteryPercentage <= 20 ? 'text-red-500 dark:text-red-400' :
                    batteryPercentage <= 50 ? 'text-yellow-600 dark:text-yellow-400' :
                    'text-purple-600 dark:text-purple-400'
                  }`} />
                  <div className="h-2 w-24 bg-gray-200 dark:bg-botbot-dark/50 rounded-full overflow-hidden">
                    <div
                      className={`h-full rounded-full transition-all duration-500 ease-out ${
                        batteryEstimate.isCharging ? 'bg-green-600 dark:bg-green-400 animate-pulse' :
                        batteryPercentage <= 20 ? 'bg-red-500 dark:bg-red-400' :
                        batteryPercentage <= 50 ? 'bg-yellow-600 dark:bg-yellow-400' :
                        'bg-purple-600 dark:bg-purple-400'
                      }`}
                      style={{ width: `${batteryPercentage}%` }}
                    />
                  </div>
                  <span className={`text-xs ml-1 w-10 text-right inline-block transition-colors ${
                    batteryEstimate.isCharging ? 'text-green-600 dark:text-green-400' :
                    batteryPercentage <= 20 ? 'text-red-500 dark:text-red-400' :
                    batteryPercentage <= 50 ? 'text-yellow-600 dark:text-yellow-400' :
                    'text-gray-600 dark:text-botbot-accent'
                  }`}>{batteryPercentage}%</span>
                  <ChevronDown className="w-4 h-4 text-gray-500 dark:text-gray-400 ml-1 transition-transform duration-200 group-data-[state=open]:rotate-180" />
                </DropdownMenuTrigger>

                <DropdownMenuContent
                  className="w-80 bg-white dark:bg-botbot-darker border border-gray-200 dark:border-gray-700 shadow-xl"
                  align="center"
                  sideOffset={8}
                >
                  {/* Header */}
                  <div className="px-4 py-3 border-b border-gray-200 dark:border-gray-700">
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-2">
                        <Battery className={`h-5 w-5 ${
                          batteryEstimate.isCharging ? 'text-green-600 dark:text-green-400' :
                          batteryPercentage <= 20 ? 'text-red-500 dark:text-red-400' :
                          batteryPercentage <= 50 ? 'text-yellow-600 dark:text-yellow-400' :
                          'text-purple-600 dark:text-purple-400'
                        }`} />
                        <span className="text-sm font-semibold text-gray-900 dark:text-white">
                          Battery Status
                        </span>
                      </div>
                      <span className={`text-2xl font-bold ${
                        batteryEstimate.isCharging ? 'text-green-600 dark:text-green-400' :
                        batteryPercentage <= 20 ? 'text-red-500 dark:text-red-400' :
                        batteryPercentage <= 50 ? 'text-yellow-600 dark:text-yellow-400' :
                        'text-gray-900 dark:text-white'
                      }`}>
                        {batteryPercentage}%
                      </span>
                    </div>
                  </div>

                  {/* Time Remaining Section */}
                  <div className="px-4 py-4 space-y-3">
                    {batteryEstimate.isCharging ? (
                      <div className="flex items-center justify-between">
                        <div className="flex items-center gap-2">
                          <Zap className="h-4 w-4 text-green-600 dark:text-green-400" />
                          <span className="text-sm text-gray-600 dark:text-gray-400">Charging</span>
                        </div>
                        {batteryEstimate.timeToFullCharge && (
                          <span className="text-sm font-medium text-green-600 dark:text-green-400">
                            Full in {Math.round(batteryEstimate.timeToFullCharge)}m
                          </span>
                        )}
                      </div>
                    ) : (
                      <>
                        {/* Time Remaining */}
                        <div className="flex items-center justify-between">
                          <div className="flex items-center gap-2">
                            <Clock className="h-4 w-4 text-gray-500 dark:text-gray-400" />
                            <span className="text-sm text-gray-600 dark:text-gray-400">Time Remaining</span>
                          </div>
                          <div className="text-right">
                            <div className="text-lg font-semibold text-gray-900 dark:text-white">
                              {batteryEstimate.formattedTime}
                            </div>
                            {batteryEstimate.confidence !== 'high' && (
                              <div className="flex items-center justify-end gap-1 mt-1">
                                {batteryEstimate.confidence === 'calculating' && (
                                  <Loader2 className="h-3 w-3 animate-spin text-purple-600 dark:text-purple-400" />
                                )}
                                <span className={`text-xs transition-colors duration-500 ${
                                  batteryEstimate.confidence === 'calculating' ? 'text-purple-600 dark:text-purple-400 animate-pulse' :
                                  batteryEstimate.confidence === 'medium' ? 'text-yellow-600 dark:text-yellow-400' :
                                  'text-orange-600 dark:text-orange-400'
                                }`}>
                                  {batteryEstimate.confidence === 'calculating' ? 'Refining estimate...' :
                                   batteryEstimate.confidence === 'medium' ? 'Good accuracy' :
                                   'Initial estimate'}
                                </span>
                              </div>
                            )}
                          </div>
                        </div>

                        {/* Discharge Rate */}
                        <div className="flex items-center justify-between">
                          <div className="flex items-center gap-2">
                            <TrendingDown className="h-4 w-4 text-gray-500 dark:text-gray-400" />
                            <span className="text-sm text-gray-600 dark:text-gray-400">Discharge Rate</span>
                          </div>
                          <span className="text-sm font-medium text-gray-900 dark:text-white">
                            {batteryEstimate.dischargeRate.toFixed(1)}% /hr
                          </span>
                        </div>

                        {/* Estimated Shutdown */}
                        {batteryEstimate.estimatedShutdownTime && (
                          <div className="flex items-center justify-between">
                            <div className="flex items-center gap-2">
                              <AlertTriangle className="h-4 w-4 text-gray-500 dark:text-gray-400" />
                              <span className="text-sm text-gray-600 dark:text-gray-400">Est. Shutdown</span>
                            </div>
                            <span className="text-sm font-medium text-gray-900 dark:text-white">
                              {batteryEstimate.estimatedShutdownTime.toLocaleTimeString([], {
                                hour: '2-digit',
                                minute: '2-digit'
                              })}
                            </span>
                          </div>
                        )}
                      </>
                    )}

                    {/* Technical Details - Updated at 0.5Hz */}
                    {batteryEstimate.currentVoltage > 0 && (
                      <div className="pt-3 border-t border-gray-200 dark:border-gray-700 space-y-2">
                        <div className="flex items-center justify-between">
                          <span className="text-xs text-gray-500 dark:text-gray-400">Voltage</span>
                          <span className="text-xs text-gray-600 dark:text-gray-300">
                            {batteryEstimate.currentVoltage.toFixed(2)}V
                          </span>
                        </div>
                        {batteryEstimate.currentAmperage !== 0 && (
                          <div className="flex items-center justify-between">
                            <span className="text-xs text-gray-500 dark:text-gray-400">Current</span>
                            <span className="text-xs text-gray-600 dark:text-gray-300">
                              {Math.abs(batteryEstimate.currentAmperage).toFixed(2)}A
                            </span>
                          </div>
                        )}
                      </div>
                    )}
                  </div>

                  {/* Warning for low battery */}
                  {!batteryEstimate.isCharging && batteryPercentage <= 20 && (
                    <div className="px-4 py-2 bg-red-50 dark:bg-red-900/20 border-t border-red-200 dark:border-red-800">
                      <div className="flex items-center gap-2">
                        <AlertTriangle className="h-4 w-4 text-red-600 dark:text-red-400" />
                        <span className="text-xs text-red-600 dark:text-red-400 font-medium">
                          Low battery - Please charge soon
                        </span>
                      </div>
                    </div>
                  )}
                </DropdownMenuContent>
              </DropdownMenu>

              {/* Connection info dropdown */}
              <DropdownMenu>
                <DropdownMenuTrigger className="flex items-center focus:outline-none group cursor-pointer">
                  {getConnectionIcon()}
                  <ChevronDown className="w-4 h-4 text-gray-500 dark:text-gray-400 ml-1 transition-transform duration-200 group-data-[state=open]:rotate-180" />
                </DropdownMenuTrigger>

                <DropdownMenuContent
                  className="w-80 bg-white dark:bg-botbot-darker border border-gray-200 dark:border-gray-700 shadow-xl"
                  align="center"
                  sideOffset={8}
                >
                  {/* Header */}
                  <div className="px-4 py-3 border-b border-gray-200 dark:border-gray-700">
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-2">
                        <Signal className="h-5 w-5 text-purple-600 dark:text-purple-400" />
                        <span className="text-sm font-semibold text-gray-900 dark:text-white">
                          Connection Status
                        </span>
                      </div>
                      <div className="flex items-center gap-2">
                        {getConnectionIcon()}
                      </div>
                    </div>
                  </div>

                  {/* Connection Details */}
                  <div className="px-4 py-4 space-y-3">
                    {/* Latency */}
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-2">
                        <Clock className="h-4 w-4 text-gray-500 dark:text-gray-400" />
                        <span className="text-sm text-gray-600 dark:text-gray-400">Latency</span>
                      </div>
                      <div className="text-right">
                        <LatencyIndicatorDetailed />
                      </div>
                    </div>

                    {/* Download Rate */}
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-2">
                        <DownloadCloud className="h-4 w-4 text-green-500 dark:text-green-400" />
                        <span className="text-sm text-gray-600 dark:text-gray-400">Download</span>
                      </div>
                      <div className="text-right">
                        <DataInIndicatorDetailed />
                      </div>
                    </div>

                    {/* Upload Rate */}
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-2">
                        <UploadCloud className="h-4 w-4 text-blue-500 dark:text-blue-400" />
                        <span className="text-sm text-gray-600 dark:text-gray-400">Upload</span>
                      </div>
                      <div className="text-right">
                        <DataOutIndicatorDetailed />
                      </div>
                    </div>

                    {/* Packet Loss */}
                    <div className="flex items-center justify-between">
                      <div className="flex items-center gap-2">
                        <AlertCircle className="h-4 w-4 text-gray-500 dark:text-gray-400" />
                        <span className="text-sm text-gray-600 dark:text-gray-400">Packet Loss</span>
                      </div>
                      <div className="text-right">
                        <PacketLossIndicator />
                      </div>
                    </div>

                    {/* Network Mode Section */}
                    {networkModeStatus.mode !== 'offline' && (
                      <div className="pt-3 border-t border-gray-200 dark:border-gray-700">
                        <div className="flex items-center justify-between">
                          <span className="text-xs text-gray-500 dark:text-gray-400">Network Mode</span>
                          <div className="flex items-center gap-2">
                            {networkModeStatus.mode === 'wifi' && (
                              <>
                                <Wifi className="h-4 w-4 text-green-600 dark:text-green-500" />
                                <span className="text-xs text-gray-600 dark:text-gray-300">
                                  WiFi {networkModeStatus.ssid && `(${networkModeStatus.ssid})`}
                                </span>
                              </>
                            )}
                            {networkModeStatus.mode === '4g' && (
                              <>
                                <Smartphone className="h-4 w-4 text-blue-600 dark:text-blue-500" />
                                <span className="text-xs text-gray-600 dark:text-gray-300">4G LTE</span>
                              </>
                            )}
                            {networkModeStatus.mode === 'hotspot' && (
                              <>
                                <Radio className="h-4 w-4 text-orange-600 dark:text-orange-500" />
                                <span className="text-xs text-gray-600 dark:text-gray-300">
                                  Hotspot {networkModeStatus.ssid && `(${networkModeStatus.ssid})`}
                                </span>
                              </>
                            )}
                          </div>
                        </div>
                      </div>
                    )}
                  </div>

                  {/* Warning for high packet loss */}
                  <ConnectionQualityWarning />
                </DropdownMenuContent>
              </DropdownMenu>
            </div>
          )}
        </div>

        {/* Center: Navigation menu */}
        <div className="flex justify-center flex-shrink-0">
          <NavMenu />
        </div>

        {/* Right section: Logo and avatar */}
        <div className="hidden xl:flex items-center justify-end flex-1">
          {/* User Avatar - positioned to not affect logo width */}
          {userProfile?.avatar_url && (
            <div className="flex items-center mr-3">
              <img
                src={userProfile.avatar_url}
                alt="User Avatar"
                className="h-[34px] w-auto object-contain"
                style={{
                  maxHeight: '34px'
                }}
              />
            </div>
          )}

          {/* Logo container with fixed width - only show if not hiding branding */}
          {!userProfile?.hide_branding && (
            <div
              id="logo-easter-egg"
              onClick={handleLogoClick}
              className="outline-none select-none flex-shrink-0"
              style={{
                touchAction: 'manipulation',
                WebkitTapHighlightColor: 'transparent',
                userSelect: 'none',
                width: '125px'
              }}
            >
              <Image
                src={imgPath}
                alt="Avatar"
                width={125}
                height={34}
                style={{
                  width: '125px',
                  height: '34px',
                  outline: 'none',
                  WebkitUserSelect: 'none',
                  MozUserSelect: 'none',
                  msUserSelect: 'none'
                }}
                priority
                draggable={false}
                className="outline-none focus:outline-none focus:ring-0 focus:ring-offset-0 active:outline-none"
              />
            </div>
          )}
        </div>

        {/* Connection Popup */}
        <RobotConnectionPopup 
          isOpen={showConnectionPopup} 
          onClose={() => setShowConnectionPopup(false)} 
        />
        
        {/* Easter egg animation */}
        {showEasterEgg && (
          <>
            <div className="fixed inset-0 pointer-events-none z-50">
              {/* Psychedelic background effect */}
              <div className="absolute inset-0 animate-psychedelic opacity-70" />
              
              {/* Color overlay that changes colors */}
              <div className="absolute inset-0 mix-blend-overlay bg-gradient-radial from-purple-500 to-transparent animate-pulse" />
              <div className="absolute inset-0 mix-blend-color-dodge bg-gradient-conic from-yellow-500 via-pink-500 to-blue-500 animate-spin" style={{ animationDuration: '3s' }} />
              
              {/* Center elements */}
              <div className="absolute inset-0 flex items-center justify-center">
                <div className="relative">
                  <div 
                    className="absolute animate-emoji1 text-9xl"
                    style={{ 
                      filter: 'drop-shadow(0 0 8px rgba(255,0,255,0.5))',
                      WebkitBackfaceVisibility: 'hidden'
                    }}
                  >
                    ❤️
                  </div>
                  <div 
                    className="absolute animate-emoji2 text-9xl" 
                    style={{ 
                      filter: 'drop-shadow(0 0 8px rgba(0,200,255,0.5))',
                      WebkitBackfaceVisibility: 'hidden'
                    }}
                  >
                    🤖
                  </div>
                  <div 
                    className="absolute animate-emoji3 text-9xl" 
                    style={{ 
                      filter: 'drop-shadow(0 0 8px rgba(255,255,0,0.5))',
                      WebkitBackfaceVisibility: 'hidden'
                    }}
                  >
                    🇧🇷
                  </div>
                </div>
              </div>
              
              {/* Signature */}
              <div 
                className="fixed bottom-6 left-0 right-0 text-center text-lg font-bold" 
                style={{ 
                  color: '#FF0000',
                  fontFamily: 'cursive, Arial, sans-serif',
                  textShadow: '0 0 5px rgba(255,0,0,0.3), 0 0 10px rgba(255,255,255,0.5)',
                  opacity: 0,
                  animation: 'fadeIn 1s ease-in forwards 0.5s'
                }}
              >
                Criado no Brasil com Amor
              </div>
            </div>
          </>
        )}
      </header>

      {/* Animated connection status message */}
      {showConnectionStatus && (
        <div className={`fixed bottom-5 left-1/2 transform -translate-x-1/2 py-2 px-4 rounded-full z-50 flex items-center ${
          isConnectionError ? 'bg-red-500 text-white' : 'bg-green-100 dark:bg-botbot-dark text-gray-800 dark:text-white'
        } shadow-lg transition-all duration-300 ease-in-out`}>
          <div className={`w-2 h-2 rounded-full mr-2 ${
            isConnectionError ? 'bg-white animate-pulse' : 'bg-green-500'
          }`}></div>
          <span className="text-sm whitespace-nowrap">{connectionStatusMessage}</span>
        </div>
      )}
    </>
  );
}

// Latency indicator component
function LatencyIndicator() {
  const networkMetrics = useNetworkMetrics(1000);
  const latencyText = formatLatency(networkMetrics.latency);
  
  // Color based on latency levels
  const getLatencyColor = () => {
    if (networkMetrics.latency < 0) return 'text-gray-400 dark:text-gray-500'; // Error/NA
    if (networkMetrics.latency < 50) return 'text-green-600 dark:text-green-400'; // Good
    if (networkMetrics.latency < 150) return 'text-yellow-600 dark:text-yellow-400'; // Medium
    return 'text-red-600 dark:text-red-400'; // Poor
  };
  
  return (
    <span className={`text-xs ${getLatencyColor()}`}>{latencyText}</span>
  );
}

// Data In indicator component
function DataInIndicator() {
  const networkMetrics = useNetworkMetrics(1000);
  return (
    <span className="text-xs text-gray-600 dark:text-botbot-accent">
      {formatDataRate(networkMetrics.dataIn)}
    </span>
  );
}

// Data Out indicator component
function DataOutIndicator() {
  const networkMetrics = useNetworkMetrics(1000);
  return (
    <span className="text-xs text-gray-600 dark:text-botbot-accent">
      {formatDataRate(networkMetrics.dataOut)}
    </span>
  );
}

// Detailed latency indicator with color coding
function LatencyIndicatorDetailed() {
  const networkMetrics = useNetworkMetrics(1000);
  const latencyText = formatLatency(networkMetrics.latency);

  // Color based on latency levels
  const getLatencyColor = () => {
    if (networkMetrics.latency < 0) return 'text-gray-400 dark:text-gray-500'; // Error/NA
    if (networkMetrics.latency < 50) return 'text-green-600 dark:text-green-400'; // Good
    if (networkMetrics.latency < 150) return 'text-yellow-600 dark:text-yellow-400'; // Medium
    return 'text-red-600 dark:text-red-400'; // Poor
  };

  const getQualityText = () => {
    if (networkMetrics.latency < 0) return 'Measuring...';
    if (networkMetrics.latency < 50) return 'Excellent';
    if (networkMetrics.latency < 150) return 'Good';
    if (networkMetrics.latency < 300) return 'Fair';
    return 'Poor';
  };

  return (
    <div className="text-right">
      <div className={`text-sm font-medium ${getLatencyColor()}`}>{latencyText}</div>
      <div className="text-xs text-gray-500 dark:text-gray-400">{getQualityText()}</div>
    </div>
  );
}

// Detailed data in indicator
function DataInIndicatorDetailed() {
  const networkMetrics = useNetworkMetrics(1000);
  const dataRateText = formatDataRate(networkMetrics.dataIn);

  return (
    <div className="text-right">
      <div className="text-sm font-medium text-gray-900 dark:text-white">{dataRateText}</div>
      {networkMetrics.dataIn > 0 && (
        <div className="text-xs text-green-600 dark:text-green-400">Active</div>
      )}
    </div>
  );
}

// Detailed data out indicator
function DataOutIndicatorDetailed() {
  const networkMetrics = useNetworkMetrics(1000);
  const dataRateText = formatDataRate(networkMetrics.dataOut);

  return (
    <div className="text-right">
      <div className="text-sm font-medium text-gray-900 dark:text-white">{dataRateText}</div>
      {networkMetrics.dataOut > 0 && (
        <div className="text-xs text-blue-600 dark:text-blue-400">Active</div>
      )}
    </div>
  );
}

// Packet loss indicator
function PacketLossIndicator() {
  const networkMetrics = useNetworkMetrics(1000);
  const packetLoss = networkMetrics.packetLoss;

  // Color based on packet loss levels
  const getPacketLossColor = () => {
    if (packetLoss === 0) return 'text-green-600 dark:text-green-400';
    if (packetLoss < 1) return 'text-green-600 dark:text-green-400';
    if (packetLoss < 5) return 'text-yellow-600 dark:text-yellow-400';
    return 'text-red-600 dark:text-red-400';
  };

  const getQualityText = () => {
    if (packetLoss === 0) return 'No loss';
    if (packetLoss < 1) return 'Minimal';
    if (packetLoss < 5) return 'Acceptable';
    return 'High loss';
  };

  return (
    <div className="text-right">
      <div className={`text-sm font-medium ${getPacketLossColor()}`}>{packetLoss}%</div>
      <div className="text-xs text-gray-500 dark:text-gray-400">{getQualityText()}</div>
    </div>
  );
}

// Connection quality warning component
function ConnectionQualityWarning() {
  const networkMetrics = useNetworkMetrics(1000);
  const { connectionStatus } = useRobotConnection();

  const hasHighLatency = networkMetrics.latency > 300;
  const hasHighPacketLoss = networkMetrics.packetLoss > 5;
  const isDisconnected = connectionStatus !== 'connected';

  if (!hasHighLatency && !hasHighPacketLoss && !isDisconnected) {
    return null;
  }

  return (
    <div className="px-4 py-2 bg-yellow-50 dark:bg-yellow-900/20 border-t border-yellow-200 dark:border-yellow-800">
      <div className="flex items-center gap-2">
        <AlertTriangle className="h-4 w-4 text-yellow-600 dark:text-yellow-400 flex-shrink-0" />
        <div className="text-xs text-yellow-600 dark:text-yellow-400 font-medium">
          {isDisconnected && 'Connection lost - attempting to reconnect'}
          {!isDisconnected && hasHighLatency && !hasHighPacketLoss && 'High latency detected'}
          {!isDisconnected && !hasHighLatency && hasHighPacketLoss && 'High packet loss detected'}
          {!isDisconnected && hasHighLatency && hasHighPacketLoss && 'Poor connection quality'}
        </div>
      </div>
    </div>
  );
}