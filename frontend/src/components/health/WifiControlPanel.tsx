'use client';

import React, { useState, useEffect } from 'react';
import * as ROSLIB from 'roslib';
import {
  Wifi,
  WifiOff,
  RefreshCw,
  Lock,
  LockOpen,
  Building2,
  Check,
  ChevronDown,
  ChevronUp,
  Smartphone,
  Loader2,
  AlertCircle,
  WifiIcon,
  Trash2,
  Globe,
  Network
} from 'lucide-react';
import { useWifiNetworks } from '@/hooks/ros/useWifiNetworks';
import { useWifiControl } from '@/hooks/ros/useWifiControl';
import { useWifiRadio } from '@/hooks/ros/useWifiRadio';
import useNetworkModeStatus from '@/hooks/ros/useNetworkModeStatus';
import { useNotifications } from '@/contexts/NotificationsContext';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { WifiNetwork, WifiAuthType, signalToBars } from '@/types/WifiControl';
import { useLanguage } from '@/contexts/LanguageContext';

interface ConnectionDialogProps {
  network: WifiNetwork;
  onConnect: (credentials: { psk?: string; identity?: string; password?: string; saveProfile: boolean }) => void;
  onCancel: () => void;
  isConnecting: boolean;
}

function ConnectionDialog({ network, onConnect, onCancel, isConnecting }: ConnectionDialogProps) {
  const { t } = useLanguage();
  const [psk, setPsk] = useState('');
  const [identity, setIdentity] = useState('');
  const [password, setPassword] = useState('');
  const [saveProfile, setSaveProfile] = useState(true);

  const handleConnect = () => {
    const credentials: any = { saveProfile };
    if (network.security === WifiAuthType.PSK) {
      credentials.psk = psk;
    } else if (network.security === WifiAuthType.WPA2_ENTERPRISE) {
      credentials.identity = identity;
      credentials.password = password;
    }
    onConnect(credentials);
  };

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 z-50 flex items-center justify-center p-4">
      <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-xl p-6 max-w-md w-full">
        <h3 className="text-lg font-semibold text-gray-800 dark:text-white mb-4">
          {t('health', 'connectToSsid').replace('{ssid}', network.ssid)}
        </h3>

        {network.security === WifiAuthType.PSK && (
          <div className="mb-4">
            <label className="block text-sm text-gray-600 dark:text-gray-400 mb-2">
              {t('health', 'password')}
            </label>
            <input
              type="password"
              value={psk}
              onChange={(e) => setPsk(e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 dark:border-gray-600 rounded-lg
                       bg-white dark:bg-botbot-darker text-gray-800 dark:text-white
                       focus:outline-none focus:ring-2 focus:ring-violet-500"
              placeholder={t('health', 'wifiPasswordPlaceholder')}
              autoFocus
            />
          </div>
        )}

        {network.security === WifiAuthType.WPA2_ENTERPRISE && (
          <>
            <div className="mb-4">
              <label className="block text-sm text-gray-600 dark:text-gray-400 mb-2">
                {t('health', 'username')}
              </label>
              <input
                type="text"
                value={identity}
                onChange={(e) => setIdentity(e.target.value)}
                className="w-full px-3 py-2 border border-gray-300 dark:border-gray-600 rounded-lg
                         bg-white dark:bg-botbot-darker text-gray-800 dark:text-white
                         focus:outline-none focus:ring-2 focus:ring-violet-500"
                placeholder={t('health', 'usernamePlaceholder')}
                autoFocus
              />
            </div>
            <div className="mb-4">
              <label className="block text-sm text-gray-600 dark:text-gray-400 mb-2">
                {t('health', 'password')}
              </label>
              <input
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                className="w-full px-3 py-2 border border-gray-300 dark:border-gray-600 rounded-lg
                         bg-white dark:bg-botbot-darker text-gray-800 dark:text-white
                         focus:outline-none focus:ring-2 focus:ring-violet-500"
                placeholder={t('health', 'passwordPlaceholder')}
              />
            </div>
          </>
        )}

        <div className="mb-6">
          <label className="flex items-center gap-2">
            <input
              type="checkbox"
              checked={saveProfile}
              onChange={(e) => setSaveProfile(e.target.checked)}
              className="rounded border-gray-300 dark:border-gray-600 text-violet-600
                       focus:ring-violet-500"
            />
            <span className="text-sm text-gray-600 dark:text-gray-400">
              {t('health', 'saveNetworkForAutomaticConnection')}
            </span>
          </label>
        </div>

        <div className="flex gap-3 justify-end">
          <button
            onClick={onCancel}
            disabled={isConnecting}
            className="px-4 py-2 text-gray-600 dark:text-gray-400 hover:text-gray-800
                     dark:hover:text-white transition-colors disabled:opacity-50"
          >
            {t('common', 'cancel')}
          </button>
          <button
            onClick={handleConnect}
            disabled={isConnecting || (network.security === WifiAuthType.PSK && !psk) ||
                     (network.security === WifiAuthType.WPA2_ENTERPRISE && (!identity || !password))}
            className="px-4 py-2 bg-violet-600 text-white rounded-lg hover:bg-violet-700
                     transition-colors disabled:opacity-50 disabled:cursor-not-allowed
                     flex items-center gap-2"
          >
            {isConnecting ? (
              <>
                <Loader2 className="w-4 h-4 animate-spin" />
                {t('health', 'connecting')}
              </>
            ) : (
              <>
                <Wifi className="w-4 h-4" />
                {t('health', 'connect')}
              </>
            )}
          </button>
        </div>
      </div>
    </div>
  );
}

function SignalBars({ bars }: { bars: number }) {
  return (
    <div className="flex gap-0.5 items-end">
      {[1, 2, 3, 4, 5].map((level) => (
        <div
          key={level}
          className={`w-1 bg-gray-300 dark:bg-gray-600 rounded-sm transition-all
                     ${bars >= level ? 'bg-green-500 dark:bg-green-400' : ''}
                     ${level === 1 ? 'h-1' :
                       level === 2 ? 'h-2' :
                       level === 3 ? 'h-3' :
                       level === 4 ? 'h-4' : 'h-5'}`}
        />
      ))}
    </div>
  );
}

export function WifiControlPanel() {
  const { dispatch } = useNotifications();
  const { t } = useLanguage();
  const networkStatus = useNetworkModeStatus();
  const { connection } = useRobotConnection();
  const { networks, isScanning, error: scanError, scanNetworks } = useWifiNetworks();
  const { isConnecting, connectionError, connectToNetwork } = useWifiControl();
  const {
    isToggling,
    savedNetworks,
    fourGStatus,
    error: radioError,
    toggleWifiRadio,
    getSavedNetworks,
    check4GStatus,
    updateNetworkStatus
  } = useWifiRadio();
  const [wifiEnabled, setWifiEnabled] = useState(true); // Default to true, will be updated on mount
  const [selectedNetwork, setSelectedNetwork] = useState<WifiNetwork | null>(null);
  const [showSavedNetworks, setShowSavedNetworks] = useState(false);
  const [connectingSavedNetwork, setConnectingSavedNetwork] = useState<string | null>(null);
  const [forgettingNetwork, setForgettingNetwork] = useState<string | null>(null);

  // Load initial data and refresh when robot connection changes
  useEffect(() => {
    if (wifiEnabled && connection.online) {
      let mounted = true;
      let retryCount = 0;
      const maxRetries = 3;

      const loadData = async () => {
        if (!mounted) return;

        try {
          // Try to scan networks
          const networks = await scanNetworks(true);
          if (!mounted) return;

          // If we successfully scanned networks, WiFi is likely enabled
          if (networks.length > 0) {
            setWifiEnabled(true);
          }

          if (networks.length === 0 && retryCount < maxRetries) {
            retryCount++;
            console.log(`No networks found, retrying... (${retryCount}/${maxRetries})`);
            setTimeout(() => loadData(), 2000); // Retry after 2 seconds
          }
        } catch (err) {
          console.error('Failed to scan networks:', err);
          if (!mounted) return;

          if (retryCount < maxRetries) {
            retryCount++;
            setTimeout(() => loadData(), 2000);
          }
        }

        // Load other data
        try {
          if (mounted) {
            const savedNetworksList = await getSavedNetworks();
            console.log('Loaded saved networks:', savedNetworksList);
          }
        } catch (err) {
          // Silently ignore saved networks errors during initial load
        }

        try {
          if (mounted) {
            await check4GStatus();
          }
        } catch (err) {
          // Silently ignore 4G status errors during initial load
          // These are handled in the hook and displayed in the UI if needed
        }
      };

      // Immediate load when robot is connected
      loadData();

      return () => {
        mounted = false;
      };
    }
  }, [wifiEnabled, connection.online]); // eslint-disable-line react-hooks/exhaustive-deps

  // Auto-refresh network list every 15 seconds when connected
  useEffect(() => {
    if (!wifiEnabled || !connection.online) return;

    const interval = setInterval(async () => {
      try {
        // Refresh available networks silently
        await scanNetworks(false);
      } catch (err) {
        // Silently ignore scan errors during auto-refresh
      }

      try {
        // Refresh saved networks silently
        await getSavedNetworks();
      } catch (err) {
        // Silently ignore saved networks errors during auto-refresh
      }

      try {
        // Refresh 4G status
        await check4GStatus();
      } catch (err) {
        // Silently ignore 4G status errors during auto-refresh
      }
    }, 15000); // Refresh every 15 seconds for better responsiveness

    return () => clearInterval(interval);
  }, [wifiEnabled, connection.online]); // eslint-disable-line react-hooks/exhaustive-deps

  const handleWifiToggle = async () => {
    const newState = !wifiEnabled;

    try {
      // Call the service to toggle WiFi radio
      const success = await toggleWifiRadio(newState);

      if (success) {
        setWifiEnabled(newState);

        dispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            title: t('health', 'wifiRadioTitle'),
            type: 'success',
            message: newState ? t('health', 'wifiEnabledSuccess') : t('health', 'wifiDisabledSuccess')
          }
        });

        if (newState) {
          // If turning on WiFi, wait a bit then scan for networks
          setTimeout(async () => {
            try {
              await scanNetworks(true);
              await getSavedNetworks();
            } catch (err) {
              console.error('Failed to scan after WiFi enable:', err);
            }
          }, 2000);
        }
      } else {
        // Service call succeeded but returned false
        dispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            title: t('health', 'wifiRadioTitle'),
            type: 'error',
            message: t('health', 'wifiToggleFailedHardware')
          }
        });
      }
    } catch (err) {
      console.error('WiFi toggle error:', err);
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: t('health', 'wifiRadioTitle'),
          type: 'error',
          message: newState ? t('health', 'failedToEnableWifi') : t('health', 'failedToDisableWifi')
        }
      });
    }
  };

  const handleScan = async () => {
    try {
      console.log('Starting network scan...');
      const foundNetworks = await scanNetworks(true);
      console.log('Scan complete, found networks:', foundNetworks);

      if (foundNetworks.length > 0) {
        dispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            title: t('health', 'networkScanTitle'),
            type: 'success',
            message: (foundNetworks.length === 1 ? t('health', 'foundNetwork') : t('health', 'foundNetworks'))
              .replace('{count}', String(foundNetworks.length))
          }
        });
      } else {
        dispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            title: t('health', 'networkScanTitle'),
            type: 'info',
            message: t('health', 'noNetworksFoundScanMessage')
          }
        });
      }
    } catch (err) {
      console.error('Scan error:', err);
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: t('health', 'networkScanTitle'),
          type: 'error',
          message: scanError || t('health', 'failedToScanNetworks')
        }
      });
    }
  };

  const handleConnect = async (network: WifiNetwork, credentials: any) => {
    try {
      await connectToNetwork({
        ssid: network.ssid,
        authType: network.security,
        ...credentials
      });

      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: t('health', 'wifiConnectionTitle'),
          type: 'success',
          message: t('health', 'connectedToNetwork').replace('{networkName}', network.ssid)
        }
      });

      setSelectedNetwork(null);

      // Update network status after connection
      setTimeout(() => {
        updateNetworkStatus();
        getSavedNetworks();
      }, 2000);
    } catch (err) {
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: t('health', 'wifiConnectionTitle'),
          type: 'error',
          message: connectionError || t('health', 'failedToConnectNetwork').replace('{networkName}', network.ssid)
        }
      });
    }
  };

  const handleConnectToSavedNetwork = async (networkName: string) => {
    setConnectingSavedNetwork(networkName);
    try {
      // For saved networks, the credentials are already stored on the robot
      // We only need to send the SSID - the robot will use the saved credentials
      // This matches the ROS2 service call: ros2 service call /connect_wifi bot_jetson_stats_interfaces/srv/ConnectWifi "{ssid: 'networkName'}"

      if (!connection.ros || !connection.online) {
        throw new Error(t('health', 'notConnectedToRobot'));
      }

      const service = new ROSLIB.Service({
        ros: connection.ros,
        name: '/connect_wifi',
        serviceType: 'bot_jetson_stats_interfaces/srv/ConnectWifi'
      });

      // Only send the SSID for saved networks - robot will auto-connect with saved credentials
      const request = { ssid: networkName };

      await new Promise((resolve, reject) => {
        const timeoutId = setTimeout(() => {
          reject(new Error(t('health', 'connectionTimeout')));
        }, 30000);

        service.callService(
          request,
          (response: any) => {
            clearTimeout(timeoutId);
            if (response && (response.success || response === true)) {
              resolve(response);
            } else {
              reject(new Error(response?.message || t('health', 'failedToConnectSavedNetwork')));
            }
          },
          (error: string) => {
            clearTimeout(timeoutId);
            reject(new Error(error));
          }
        );
      });

      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: t('health', 'wifiConnectionTitle'),
          type: 'success',
          message: t('health', 'connectedToNetwork').replace('{networkName}', networkName)
        }
      });

      // Update network status after connection attempt
      setTimeout(() => {
        updateNetworkStatus();
        scanNetworks(false); // Refresh network list
        setConnectingSavedNetwork(null);
      }, 3000);
    } catch (err) {
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: t('health', 'wifiConnectionTitle'),
          type: 'error',
          message: t('health', 'failedToConnectSavedNetworkMessage').replace('{networkName}', networkName)
        }
      });
      setConnectingSavedNetwork(null);
    }
  };

  const handleForgetNetwork = async (networkName: string) => {
    // Confirm before forgetting
    if (!confirm(t('health', 'forgetNetworkConfirm').replace('{networkName}', networkName))) {
      return;
    }

    setForgettingNetwork(networkName);
    try {
      // Call the forget network service
      if (!connection.ros || !connection.online) {
        throw new Error(t('health', 'notConnectedToRobot'));
      }

      const service = new ROSLIB.Service({
        ros: connection.ros,
        name: '/forget_network',
        serviceType: 'bot_jetson_stats_interfaces/srv/ForgetNetwork'
      });

      const request = { ssid: networkName };

      await new Promise((resolve, reject) => {
        service.callService(
          request,
          (response: any) => {
            if (response && (response.success || response === true)) {
              resolve(response);
            } else {
              reject(new Error(response?.message || t('health', 'failedToForgetNetwork')));
            }
          },
          (error: string) => {
            reject(new Error(error));
          }
        );
      });

      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: t('health', 'networkForgottenTitle'),
          type: 'success',
          message: t('health', 'networkForgottenMessage').replace('{networkName}', networkName)
        }
      });

      // Refresh saved networks list
      await getSavedNetworks();
    } catch (err) {
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: t('health', 'forgetNetworkTitle'),
          type: 'error',
          message: t('health', 'failedToForgetNetworkMessage').replace('{networkName}', networkName)
        }
      });
    } finally {
      setForgettingNetwork(null);
    }
  };


  const getSecurityIcon = (security: WifiAuthType) => {
    switch (security) {
      case WifiAuthType.OPEN:
        return <LockOpen className="w-4 h-4 text-gray-500" />;
      case WifiAuthType.PSK:
        return <Lock className="w-4 h-4 text-yellow-600 dark:text-yellow-400" />;
      case WifiAuthType.WPA2_ENTERPRISE:
        return <Building2 className="w-4 h-4 text-violet-600 dark:text-violet-400" />;
    }
  };

  const parseNetworkInfo = () => {
    // Parse 4G status for IP address information
    let wifiIp = '';
    let fourGIp = '';
    let fourGActive = false;

    if (fourGStatus) {
      const isUp = fourGStatus.includes('UP');
      const ipMatch = fourGStatus.match(/(\d+\.\d+\.\d+\.\d+)/);

      // Determine if this is WiFi or 4G based on interface name or context
      if (fourGStatus.toLowerCase().includes('wlan') || fourGStatus.toLowerCase().includes('wifi')) {
        wifiIp = ipMatch ? ipMatch[1] : '';
      } else if (fourGStatus.toLowerCase().includes('wwan') || fourGStatus.toLowerCase().includes('4g') || fourGStatus.toLowerCase().includes('lte')) {
        fourGActive = isUp;
        fourGIp = ipMatch ? ipMatch[1] : '';
      } else if (isUp && ipMatch) {
        // If we can't determine the type, assume it's WiFi if we're connected to WiFi
        if (networkStatus?.mode === 'wifi') {
          wifiIp = ipMatch[1];
        } else {
          fourGIp = ipMatch[1];
        }
      }
    }

    return { wifiIp, fourGIp, fourGActive };
  };

  const networkInfo = parseNetworkInfo();
  const currentNetwork = networkStatus?.ssid || '';
  const isConnected = networkStatus?.mode === 'wifi' && currentNetwork;

  return (
    <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-6 border border-gray-100 dark:border-botbot-darker">
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center gap-3">
          <WifiIcon className="w-5 h-5 text-violet-600 dark:text-violet-400" />
          <h2 className="text-lg font-semibold text-gray-800 dark:text-white">{t('health', 'wifiSettings')}</h2>
        </div>
        <div className="flex items-center gap-3">
          <button
            onClick={handleScan}
            disabled={isScanning || !wifiEnabled}
            className="p-2 text-gray-600 dark:text-gray-400 hover:text-violet-600
                     dark:hover:text-violet-400 transition-colors disabled:opacity-50"
            title={t('health', 'scanForNetworks')}
          >
            <RefreshCw className={`w-4 h-4 ${isScanning ? 'animate-spin' : ''}`} />
          </button>
        </div>
      </div>

      {/* Current Connection Status */}
      <div className="mb-4 p-3 bg-gray-50 dark:bg-botbot-darker rounded-lg">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            {isConnected ? (
              <Wifi className="w-5 h-5 text-green-500" />
            ) : (
              <WifiOff className="w-5 h-5 text-gray-400" />
            )}
            <div>
              <p className="text-sm font-medium text-gray-800 dark:text-white">
                {isConnected ? currentNetwork : t('health', 'notConnectedTitle')}
              </p>
              {networkStatus?.mode && (
                <p className="text-xs text-gray-500 dark:text-gray-400">
                  {t('health', 'modeLabel').replace('{mode}', networkStatus.mode)}
                </p>
              )}
            </div>
          </div>
          <div className="flex items-center gap-4">
            {/* WiFi IP Address */}
            {isConnected && networkInfo.wifiIp && (
              <div className="flex items-center gap-2">
                <Network className="w-4 h-4 text-green-500" />
                <span className="text-xs text-gray-600 dark:text-gray-400">
                  {t('health', 'ipAddress').replace('{ip}', networkInfo.wifiIp)}
                </span>
              </div>
            )}
            {/* 4G Status */}
            {networkInfo.fourGActive && (
              <div className="flex items-center gap-2">
                <Smartphone className="w-4 h-4 text-blue-500" />
                <span className="text-xs text-gray-600 dark:text-gray-400">
                  4G: {networkInfo.fourGIp || t('health', 'activeStatus')}
                </span>
              </div>
            )}
            <div className="flex items-center gap-2">
              <span className="text-sm text-gray-600 dark:text-gray-400">
                {t('health', 'wifiStateLabel').replace('{state}', wifiEnabled ? t('health', 'enabled') : t('health', 'disabled'))}
              </span>
              <button
                onClick={handleWifiToggle}
                disabled={isToggling || !connection.online}
                className={`relative inline-flex h-6 w-11 items-center rounded-full transition-colors
                          ${wifiEnabled ? 'bg-violet-600' : 'bg-gray-300 dark:bg-gray-600'}
                          ${isToggling || !connection.online ? 'opacity-50 cursor-not-allowed' : ''}`}
                title={!connection.online ? t('health', 'connectToRobotToToggleWifi') :
                       wifiEnabled ? t('health', 'clickToDisableWifi') : t('health', 'clickToEnableWifi')}
              >
                {isToggling ? (
                  <Loader2 className="absolute left-1/2 -translate-x-1/2 w-3 h-3 animate-spin text-white" />
                ) : (
                  <span
                    className={`inline-block h-4 w-4 transform rounded-full bg-white transition-transform
                              ${wifiEnabled ? 'translate-x-6' : 'translate-x-1'}`}
                  />
                )}
              </button>
            </div>
          </div>
        </div>
      </div>

      {/* WiFi Disabled Message */}
      {!wifiEnabled && (
        <div className="mb-4 p-3 bg-gray-50 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded-lg">
          <div className="flex items-center gap-2">
            <WifiOff className="w-4 h-4 text-gray-500 dark:text-gray-400" />
            <p className="text-sm text-gray-600 dark:text-gray-400">
              {t('health', 'wifiDisabledHelp')}
            </p>
          </div>
        </div>
      )}

      {/* Available Networks */}
      {wifiEnabled && (
        <div className="mb-4">
          <div className="flex items-center justify-between mb-2">
            <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300">
              {t('health', 'availableNetworks')}
            </h3>
            {scanError && (
              <span className="text-xs text-amber-600 dark:text-amber-400 flex items-center gap-1">
                <AlertCircle className="w-3 h-3" />
                {t('health', 'scanError')}
              </span>
            )}
          </div>
          <div className="max-h-64 overflow-y-auto space-y-1 border border-gray-200 dark:border-gray-700 rounded-lg p-2">
            {isScanning ? (
              <div className="flex items-center justify-center py-8">
                <Loader2 className="w-6 h-6 animate-spin text-violet-600" />
              </div>
            ) : networks.length > 0 ? (
              networks.map((network) => (
                <div
                  key={network.ssid}
                  className="flex items-center justify-between p-2 hover:bg-gray-50
                           dark:hover:bg-botbot-darker rounded-lg transition-colors"
                >
                  <div className="flex items-center gap-3">
                    <SignalBars bars={network.bars || signalToBars(network.signal)} />
                    <span className="text-sm font-medium text-gray-800 dark:text-white">
                      {network.ssid}
                    </span>
                    {getSecurityIcon(network.security)}
                    {network.inUse && (
                      <Check className="w-4 h-4 text-green-500" />
                    )}
                  </div>
                  {!network.inUse && (
                    <button
                      onClick={() => {
                        if (network.security === WifiAuthType.OPEN) {
                          handleConnect(network, { saveProfile: true });
                        } else {
                          setSelectedNetwork(network);
                        }
                      }}
                      disabled={isConnecting}
                      className="px-3 py-1 text-xs font-medium text-violet-700 dark:text-violet-400
                               bg-violet-50 dark:bg-violet-900/20 rounded hover:bg-violet-100
                               dark:hover:bg-violet-900/30 transition-colors
                               disabled:opacity-50 disabled:cursor-not-allowed"
                    >
                      {t('health', 'connect')}
                    </button>
                  )}
                </div>
              ))
            ) : (
              <p className="text-sm text-gray-500 dark:text-gray-400 text-center py-4">
                {t('health', 'noNetworksFound')}
              </p>
            )}
          </div>
        </div>
      )}

      {/* Saved Networks */}
      <div className="mb-4">
        <div className="flex items-center justify-between">
          <button
            onClick={async () => {
              setShowSavedNetworks(!showSavedNetworks);
              // Refresh saved networks when expanding the section
              if (!showSavedNetworks && connection.online) {
                try {
                  await getSavedNetworks();
                } catch (err) {
                  // Silently ignore errors
                }
              }
            }}
            className="flex items-center gap-2 text-sm font-medium text-gray-700 dark:text-gray-300"
          >
            {t('health', 'savedNetworks').replace('{count}', String(savedNetworks.length))}
            {showSavedNetworks ? <ChevronUp className="w-4 h-4" /> : <ChevronDown className="w-4 h-4" />}
          </button>
          {showSavedNetworks && (
            <button
              onClick={async () => {
                try {
                  const networks = await getSavedNetworks();
                  console.log('Refreshed saved networks:', networks);
                } catch (err) {
                  console.error('Failed to refresh saved networks:', err);
                }
              }}
              className="p-1 text-gray-600 dark:text-gray-400 hover:text-violet-600
                       dark:hover:text-violet-400 transition-colors"
              title={t('health', 'refreshSavedNetworks')}
            >
              <RefreshCw className="w-3 h-3" />
            </button>
          )}
        </div>
        {showSavedNetworks && (
          <div className="mt-2">
            {savedNetworks.length > 0 ? (
              <div className="space-y-2 pl-4">
                {savedNetworks.map((network) => (
                  <div key={network} className="flex items-center justify-between p-2 hover:bg-gray-50
                                                 dark:hover:bg-botbot-darker rounded-lg transition-colors">
                    <div className="flex items-center gap-2 text-sm text-gray-600 dark:text-gray-400">
                      <Wifi className="w-3 h-3" />
                      <span>{network}</span>
                      {networkStatus?.ssid === network && (
                        <span className="text-xs text-green-600 dark:text-green-400 ml-2">{t('health', 'connectedTag')}</span>
                      )}
                    </div>
                    <div className="flex items-center gap-2">
                      {networkStatus?.ssid !== network && (
                        <button
                          onClick={() => handleConnectToSavedNetwork(network)}
                          disabled={connectingSavedNetwork !== null || forgettingNetwork !== null}
                          className="px-3 py-1 text-xs bg-violet-600 text-white rounded-lg
                                   hover:bg-violet-700 transition-colors disabled:opacity-50
                                   disabled:cursor-not-allowed flex items-center gap-1"
                          title={t('health', 'connectToNetwork').replace('{networkName}', network)}
                        >
                          {connectingSavedNetwork === network ? (
                            <>
                              <Loader2 className="w-3 h-3 animate-spin" />
                              {t('health', 'connecting')}
                            </>
                          ) : (
                            <>
                              <WifiIcon className="w-3 h-3" />
                              {t('health', 'connect')}
                            </>
                          )}
                        </button>
                      )}
                      <button
                        onClick={() => handleForgetNetwork(network)}
                        disabled={forgettingNetwork !== null || connectingSavedNetwork !== null}
                        className="p-1 text-gray-600 dark:text-gray-400 hover:text-red-600
                                 dark:hover:text-red-400 transition-colors disabled:opacity-50
                                 disabled:cursor-not-allowed"
                        title={t('health', 'forgetNetwork').replace('{networkName}', network)}
                      >
                        {forgettingNetwork === network ? (
                          <Loader2 className="w-3 h-3 animate-spin" />
                        ) : (
                          <Trash2 className="w-3 h-3" />
                        )}
                      </button>
                    </div>
                  </div>
                ))}
              </div>
            ) : (
              <p className="text-xs text-gray-500 dark:text-gray-400 pl-4">
                {t('health', 'noSavedNetworksFound')}
              </p>
            )}
          </div>
        )}
      </div>


      {/* Connection Dialog */}
      {selectedNetwork && (
        <ConnectionDialog
          network={selectedNetwork}
          onConnect={(credentials) => handleConnect(selectedNetwork, credentials)}
          onCancel={() => setSelectedNetwork(null)}
          isConnecting={isConnecting}
        />
      )}

      {/* Error Display */}
      {(() => {
        const error = scanError || connectionError || radioError;
        // Filter out "Not connected to robot" messages
        const shouldShowError = error && !error.includes('Not connected to robot');
        return shouldShowError ? (
          <div className="mt-4 p-3 bg-red-50 dark:bg-red-900/20 border border-red-200 dark:border-red-800 rounded-lg">
            <div className="flex items-center gap-2">
              <AlertCircle className="w-4 h-4 text-red-600 dark:text-red-400" />
              <p className="text-sm text-red-600 dark:text-red-400">
                {error}
              </p>
            </div>
          </div>
        ) : null;
      })()}
    </div>
  );
}
