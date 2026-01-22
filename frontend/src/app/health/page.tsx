'use client';

import { useState, useEffect } from 'react';
import {
  Cpu,
  Zap,
  Thermometer,
  HardDrive,
  Activity,
  Fan,
  Gauge,
  MemoryStick,
  Info,
  Server,
  Wifi,
  Smartphone,
  Radio,
  Globe,
  Clock,
  DownloadCloud,
  UploadCloud,
  Signal,
  Power
} from 'lucide-react';
import { useRosJetsonDiagnostics } from '@/hooks/ros/useRosJetsonDiagnostics';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useLanguage } from '@/contexts/LanguageContext';
import useNetworkModeStatus from '@/hooks/ros/useNetworkModeStatus';
import useNetworkMetrics from '@/hooks/ros/useNetworkMetrics';
import { formatLatency, formatDataRate } from '@/utils/format-utils';
import type { JetsonCpu } from '@/types/JetsonDiagnostics';
import { useStateMachineStatus } from '@/hooks/ros/useStateMachineStatus';
import { useStateMachineCommand } from '@/hooks/ros/useStateMachineCommand';
import type { NodeStatus, NodeStatusSignal } from '@/types/StateMachine';
import { useNotifications } from '@/contexts/NotificationsContext';
import { useRosDiagnostics } from '@/hooks/ros/useRosDiagnostics';
import { DiagnosticLevel, getDiagnosticLevelName, getDiagnosticLevelColor } from '@/types/RosDiagnostics';
import type { DiagnosticStatus } from '@/types/RosDiagnostics';
import { AlertCircle, AlertTriangle, CheckCircle, XCircle } from 'lucide-react';
import PowerUsageGraph from '@/components/health/PowerUsageGraph';
import CPUUsageGraph from '@/components/health/CPUUsageGraph';
import GPUUsageGraph from '@/components/health/GPUUsageGraph';
import { WifiControlPanel } from '@/components/health/WifiControlPanel';
import { useSystemReboot } from '@/hooks/ros/useSystemReboot';

// Mock data structure - fallback when not connected
const mockJetsonData = {
  board: {
    status: 'N/A',
    powerMode: 'N/A',
    jetsonClocks: false,
    uptime: 'N/A',
    model: 'N/A',
    jetpack: 'N/A',
    serialNumber: 'N/A',
  },
  cpus: [] as JetsonCpu[],
  gpu: {
    usage: 0,
    freq: { cur: 0, max: 0, min: 0 },
    governor: 'N/A',
  },
  memory: {
    ram: { used: 0, total: 0, percentage: 0 },
    swap: { used: 0, total: 0, percentage: 0 },
    emc: { usage: 0, freq: 0 },
  },
  temperatures: {
    cpu: 0,
    soc0: 0,
    soc1: 0,
    soc2: 0,
    tj: 0,
  },
  power: {
    current: 0,
    average: 0,
    rails: [] as { name: string; current: number; average: number }[],
  },
  fan: {
    speed: 0,
    mode: 'N/A',
    control: 'N/A',
  },
  disk: {
    used: 0,
    total: 0,
    percentage: 0,
  },
};

// CPU core component
function CpuCore({ id, usage, freq, t }: { id: number; usage: number; freq: number; t: any }) {
  const getUsageColor = (usage: number) => {
    if (usage < 30) return 'bg-green-500';
    if (usage < 60) return 'bg-yellow-500';
    if (usage < 80) return 'bg-orange-500';
    return 'bg-red-500';
  };

  return (
    <div className="bg-white/5 backdrop-blur-sm rounded-lg p-3 border border-white/10">
      <div className="flex justify-between items-center mb-2">
        <span className="text-xs font-medium text-gray-600 dark:text-gray-400">
          {t('health', 'core')} {id}
        </span>
        <span className="text-xs text-gray-500 dark:text-gray-500">
          {(freq / 1000).toFixed(0)} {t('health', 'mhz')}
        </span>
      </div>
      <div className="flex items-center gap-2">
        <div className="flex-1 h-2 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden">
          <div
            className={`h-full transition-all duration-300 ${getUsageColor(usage)}`}
            style={{ width: `${usage}%` }}
          />
        </div>
        <span className="text-sm font-semibold text-gray-800 dark:text-white min-w-[45px] text-right">
          {usage.toFixed(1)}%
        </span>
      </div>
    </div>
  );
}

// Temperature indicator component
function TempIndicator({ label, value, t }: { label: string; value: number; t: any }) {
  const getTempColor = (temp: number) => {
    if (temp <= 0) return 'text-gray-500';
    if (temp < 40) return 'text-blue-500';
    if (temp < 50) return 'text-green-500';
    if (temp < 60) return 'text-yellow-500';
    if (temp < 70) return 'text-orange-500';
    return 'text-red-500';
  };

  return (
    <div className="flex items-center justify-between py-2">
      <span className="text-sm text-gray-600 dark:text-gray-400">{label}</span>
      <span className={`text-sm font-medium ${getTempColor(value)}`}>
        {value > 0 ? `${value.toFixed(1)}${t('health', 'celsius')}` : t('health', 'notAvailable')}
      </span>
    </div>
  );
}

// Power rail component
function PowerRail({ name, current, average, t }: { name: string; current: number; average: number; t: any }) {
  return (
    <div className="flex items-center justify-between py-2 border-b border-gray-100 dark:border-gray-700 last:border-0">
      <span className="text-sm text-gray-600 dark:text-gray-400">{name}</span>
      <div className="flex gap-4 text-sm">
        <span className="text-gray-500 dark:text-gray-500">
          {t('health', 'current')}: <span className="font-medium text-gray-800 dark:text-white">{current}{t('health', 'milliwatts')}</span>
        </span>
        <span className="text-gray-500 dark:text-gray-500">
          {t('health', 'avg')}: <span className="font-medium text-gray-800 dark:text-white">{average}{t('health', 'milliwatts')}</span>
        </span>
      </div>
    </div>
  );
}

// Helper function to get signal color based on node state
function getNodeSignal(node: NodeStatus): NodeStatusSignal {
  const stateLower = node.state.toLowerCase();

  // Green: active/running/ok states
  if (node.active && (stateLower.includes('active') || stateLower.includes('running') || stateLower === 'ok')) {
    return 'green';
  }

  // Red: error/failed/fault states
  if (stateLower.includes('error') || stateLower.includes('failed') || stateLower.includes('fault')) {
    return 'red';
  }

  // Yellow: idle/inactive/waiting states or inactive nodes
  return 'yellow';
}

// Diagnostic Item Component
function DiagnosticItem({ diagnostic, t }: { diagnostic: DiagnosticStatus; t: any }) {
  const levelColor = getDiagnosticLevelColor(diagnostic.level);

  const getIcon = () => {
    switch (diagnostic.level) {
      case DiagnosticLevel.OK:
        return <CheckCircle className="w-4 h-4 text-green-500" />;
      case DiagnosticLevel.WARN:
        return <AlertTriangle className="w-4 h-4 text-yellow-500" />;
      case DiagnosticLevel.ERROR:
        return <XCircle className="w-4 h-4 text-red-500" />;
      case DiagnosticLevel.STALE:
        return <AlertCircle className="w-4 h-4 text-gray-500" />;
      default:
        return <AlertCircle className="w-4 h-4 text-gray-500" />;
    }
  };

  const getBgColor = () => {
    switch (diagnostic.level) {
      case DiagnosticLevel.OK:
        return 'bg-green-50 dark:bg-green-900/20 border-green-200 dark:border-green-900/30';
      case DiagnosticLevel.WARN:
        return 'bg-yellow-50 dark:bg-yellow-900/20 border-yellow-200 dark:border-yellow-900/30';
      case DiagnosticLevel.ERROR:
        return 'bg-red-50 dark:bg-red-900/20 border-red-200 dark:border-red-900/30';
      case DiagnosticLevel.STALE:
        return 'bg-gray-50 dark:bg-gray-900/20 border-gray-200 dark:border-gray-900/30';
      default:
        return 'bg-gray-50 dark:bg-gray-900/20 border-gray-200 dark:border-gray-900/30';
    }
  };

  const getTextColor = () => {
    switch (diagnostic.level) {
      case DiagnosticLevel.OK:
        return 'text-green-700 dark:text-green-300';
      case DiagnosticLevel.WARN:
        return 'text-yellow-700 dark:text-yellow-300';
      case DiagnosticLevel.ERROR:
        return 'text-red-700 dark:text-red-300';
      case DiagnosticLevel.STALE:
        return 'text-gray-700 dark:text-gray-300';
      default:
        return 'text-gray-700 dark:text-gray-300';
    }
  };

  return (
    <div className={`p-3 rounded-lg border ${getBgColor()}`}>
      <div className="flex items-start gap-3">
        <div className="mt-0.5">{getIcon()}</div>
        <div className="flex-1 min-w-0">
          <div className="flex items-center justify-between gap-2 mb-1">
            <h4 className={`text-sm font-medium truncate ${getTextColor()}`}>
              {diagnostic.name}
            </h4>
            <span className={`text-xs px-2 py-0.5 rounded-full ${
              diagnostic.level === DiagnosticLevel.OK ? 'bg-green-100 dark:bg-green-900/40 text-green-700 dark:text-green-400' :
              diagnostic.level === DiagnosticLevel.WARN ? 'bg-yellow-100 dark:bg-yellow-900/40 text-yellow-700 dark:text-yellow-400' :
              diagnostic.level === DiagnosticLevel.ERROR ? 'bg-red-100 dark:bg-red-900/40 text-red-700 dark:text-red-400' :
              'bg-gray-100 dark:bg-gray-900/40 text-gray-700 dark:text-gray-400'
            }`}>
              {getDiagnosticLevelName(diagnostic.level)}
            </span>
          </div>
          {diagnostic.message && (
            <p className="text-xs text-gray-600 dark:text-gray-400 mb-2">{diagnostic.message}</p>
          )}
          {diagnostic.hardware_id && (
            <p className="text-xs text-gray-500 dark:text-gray-500">
              <span className="font-medium">HW ID:</span> {diagnostic.hardware_id}
            </p>
          )}
          {diagnostic.values && diagnostic.values.length > 0 && (
            <div className="mt-2 space-y-1">
              {diagnostic.values.slice(0, 3).map((kv, idx) => (
                <div key={idx} className="flex items-center justify-between text-xs">
                  <span className="text-gray-500 dark:text-gray-500">{kv.key}:</span>
                  <span className="text-gray-700 dark:text-gray-300 font-mono">{kv.value}</span>
                </div>
              ))}
              {diagnostic.values.length > 3 && (
                <p className="text-xs text-gray-500 dark:text-gray-500">
                  +{diagnostic.values.length - 3} more values
                </p>
              )}
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

// State Machine Row Component
function StateMachineRow({
  node,
  onStart,
  onStop,
  onReset,
  t
}: {
  node: NodeStatus;
  onStart: (name: string) => void;
  onStop: (name: string) => void;
  onReset: (name: string) => void;
  t: any;
}) {
  const [showIdDialog, setShowIdDialog] = useState(false);
  const signal = getNodeSignal(node);
  const signalColor = signal === 'green' ? 'bg-green-500' : signal === 'red' ? 'bg-red-500' : 'bg-yellow-500';

  return (
    <>
      <tr className="border-b border-gray-100 dark:border-gray-700/50 last:border-0 hover:bg-gray-50 dark:hover:bg-gray-800/30 transition-colors">
        <td className="py-3 px-4">
          <div className={`w-3 h-3 rounded-full ${signalColor}`} />
        </td>
        <td className="py-3 px-4">
          <div className="flex items-center gap-2">
            <span className="text-sm font-medium text-gray-800 dark:text-white">{node.displayName}</span>
            <button
              onClick={() => setShowIdDialog(true)}
              className="w-4 h-4 flex items-center justify-center rounded-full bg-gray-200 dark:bg-gray-700 text-gray-500 dark:text-gray-400 hover:bg-violet-100 dark:hover:bg-violet-900/30 hover:text-violet-600 dark:hover:text-violet-400 transition-colors"
              title="View module ID"
            >
              <Info className="w-2.5 h-2.5" />
            </button>
          </div>
        </td>
        <td className="py-3 px-4 text-sm text-gray-600 dark:text-gray-400">{node.state}</td>
        <td className="py-3 px-4">
          <div className="flex gap-2 justify-end">
            <button
              onClick={() => onStart(node.name)}
              className="px-3 py-1 text-xs font-medium text-green-700 dark:text-green-400 bg-green-50 dark:bg-green-900/20 rounded hover:bg-green-100 dark:hover:bg-green-900/30 transition-colors"
            >
              Start
            </button>
            <button
              onClick={() => onStop(node.name)}
              className="px-3 py-1 text-xs font-medium text-red-700 dark:text-red-400 bg-red-50 dark:bg-red-900/20 rounded hover:bg-red-100 dark:hover:bg-red-900/30 transition-colors"
            >
              Stop
            </button>
            <button
              onClick={() => onReset(node.name)}
              className="px-3 py-1 text-xs font-medium text-blue-700 dark:text-blue-400 bg-blue-50 dark:bg-blue-900/20 rounded hover:bg-blue-100 dark:hover:bg-blue-900/30 transition-colors"
            >
              Reset
            </button>
          </div>
        </td>
      </tr>

      {/* Module ID Dialog */}
      {showIdDialog && (
        <tr>
          <td colSpan={4} className="p-0">
            <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/50 backdrop-blur-sm" onClick={() => setShowIdDialog(false)}>
              <div
                className="bg-white dark:bg-botbot-dark rounded-lg shadow-xl p-5 max-w-sm w-full mx-4 border border-gray-100 dark:border-botbot-darker"
                onClick={(e) => e.stopPropagation()}
              >
                <div className="flex items-center gap-3 mb-4">
                  <div className="w-10 h-10 rounded-full bg-violet-100 dark:bg-violet-900/20 flex items-center justify-center">
                    <Info className="w-5 h-5 text-violet-600 dark:text-violet-400" />
                  </div>
                  <div>
                    <h3 className="text-base font-semibold text-gray-900 dark:text-white">
                      {node.displayName}
                    </h3>
                    <p className="text-xs text-gray-500 dark:text-gray-400">Module Details</p>
                  </div>
                </div>
                <div className="bg-gray-50 dark:bg-botbot-darker rounded-lg p-3 mb-4">
                  <p className="text-xs text-gray-500 dark:text-gray-400 mb-1">{t('health', 'id')}</p>
                  <p className="text-sm font-mono text-gray-800 dark:text-white break-all">{node.name}</p>
                </div>
                <button
                  onClick={() => setShowIdDialog(false)}
                  className="w-full px-4 py-2 text-sm font-medium text-white bg-violet-600 rounded-lg hover:bg-violet-700 transition-colors"
                >
                  Close
                </button>
              </div>
            </div>
          </td>
        </tr>
      )}
    </>
  );
}

export default function HealthPage() {
  const { diagnosticsData, isConnected } = useRosJetsonDiagnostics();
  const { connection, connectionStatus } = useRobotConnection();
  const { t } = useLanguage();
  const [data, setData] = useState(mockJetsonData);
  const [isUsingFallback, setIsUsingFallback] = useState(false);
  const networkModeStatus = useNetworkModeStatus();
  const networkMetrics = useNetworkMetrics(1000); // Update every second

  // State Machine hooks
  const { nodeStatuses } = useStateMachineStatus();
  const { activateNode, deactivateNode, restartNode } = useStateMachineCommand();
  const { dispatch } = useNotifications();

  // Diagnostics hook
  const { diagnostics, issues, errors, warnings, lastUpdate: diagnosticsLastUpdate } = useRosDiagnostics();

  // System reboot hook
  const { rebootSystem, isRebooting } = useSystemReboot();
  const [showRebootConfirm, setShowRebootConfirm] = useState(false);

  useEffect(() => {
    // Set page title
    document.title = 'Health - BotBot';
  }, []);

  // Update data when diagnostics data is received
  useEffect(() => {
    if (diagnosticsData) {
      // Check if we're using fallback data
      const usingFallback = diagnosticsData.board.model === 'N/A';
      setIsUsingFallback(usingFallback);
      
      // Map the diagnostics data to our display format
      setData({
        board: {
          status: diagnosticsData.board.powerMode,
          powerMode: diagnosticsData.board.powerMode,
          jetsonClocks: diagnosticsData.board.jetsonClocks,
          uptime: diagnosticsData.board.uptime,
          model: diagnosticsData.board.model,
          jetpack: diagnosticsData.board.jetpack,
          serialNumber: diagnosticsData.board.serialNumber,
        },
        cpus: diagnosticsData.cpus,
        gpu: {
          usage: diagnosticsData.gpu.usage,
          freq: diagnosticsData.gpu.freq,
          governor: diagnosticsData.gpu.freq.governor,
        },
        memory: {
          ram: {
            used: diagnosticsData.memory.ram.used,
            total: diagnosticsData.memory.ram.total,
            percentage: (diagnosticsData.memory.ram.used / diagnosticsData.memory.ram.total) * 100,
          },
          swap: {
            used: diagnosticsData.memory.swap.used,
            total: diagnosticsData.memory.swap.total,
            percentage: diagnosticsData.memory.swap.total > 0 
              ? (diagnosticsData.memory.swap.used / diagnosticsData.memory.swap.total) * 100 
              : 0,
          },
          emc: {
            usage: diagnosticsData.memory.emc.usage,
            freq: diagnosticsData.memory.emc.freq,
          },
        },
        temperatures: diagnosticsData.temperatures,
        power: {
          current: diagnosticsData.power.currentTotal,
          average: diagnosticsData.power.averageTotal,
          rails: diagnosticsData.power.rails.map(rail => ({
            name: rail.name,
            current: rail.currentPower,
            average: rail.averagePower,
          })),
        },
        fan: {
          speed: diagnosticsData.fan.speed[0] || 0,
          mode: diagnosticsData.fan.mode,
          control: diagnosticsData.fan.control,
        },
        disk: {
          used: diagnosticsData.disk.used,
          total: diagnosticsData.disk.total,
          percentage: (diagnosticsData.disk.used / diagnosticsData.disk.total) * 100,
        },
      });
    }
  }, [diagnosticsData]);

  // State Machine command handlers
  const handleStartNode = async (nodeName: string) => {
    try {
      await activateNode(nodeName);
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'State Machine',
          type: 'success',
          message: `Successfully started ${nodeName}`,
        },
      });
    } catch (error) {
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'State Machine',
          type: 'error',
          message: `Failed to start ${nodeName}: ${error instanceof Error ? error.message : 'Unknown error'}`,
        },
      });
    }
  };

  const handleStopNode = async (nodeName: string) => {
    try {
      await deactivateNode(nodeName);
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'State Machine',
          type: 'success',
          message: `Successfully stopped ${nodeName}`,
        },
      });
    } catch (error) {
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'State Machine',
          type: 'error',
          message: `Failed to stop ${nodeName}: ${error instanceof Error ? error.message : 'Unknown error'}`,
        },
      });
    }
  };

  const handleResetNode = async (nodeName: string) => {
    try {
      await restartNode(nodeName);
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'State Machine',
          type: 'success',
          message: `Successfully reset ${nodeName}`,
        },
      });
    } catch (error) {
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'State Machine',
          type: 'error',
          message: `Failed to reset ${nodeName}: ${error instanceof Error ? error.message : 'Unknown error'}`,
        },
      });
    }
  };

  // System reboot handler
  const handleSystemReboot = async () => {
    try {
      await rebootSystem();
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'System Reboot',
          type: 'success',
          message: 'System reboot initiated successfully',
        },
      });
      setShowRebootConfirm(false);
    } catch (error) {
      dispatch({
        type: 'ADD_NOTIFICATION',
        payload: {
          title: 'System Reboot',
          type: 'error',
          message: `Failed to reboot system: ${error instanceof Error ? error.message : 'Unknown error'}`,
        },
      });
      setShowRebootConfirm(false);
    }
  };

  // Check data availability for each section
  const hasJetsonData = diagnosticsData && diagnosticsData.board.model !== 'N/A';
  const hasStateMachineData = nodeStatuses.length > 0;
  const hasDiagnosticsData = diagnostics.length > 0;
  const hasNetworkData = connection.online && (networkModeStatus.mode !== 'offline' || networkMetrics.latency >= 0);

  return (
    <div className="p-6 overflow-y-auto h-full">
      <div className="max-w-[1920px] mx-auto space-y-6">
        {/* Header */}
        <div className="mb-6">
          <h1 className="text-3xl font-bold text-gray-800 dark:text-white">
            {t('health', 'pageTitle')}
          </h1>
        </div>

        {/* Top Row - System Info and Connectivity */}
        <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
          {/* System Info Card */}
          <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-6 border border-gray-100 dark:border-botbot-darker">
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center gap-3">
                <Info className="w-5 h-5 text-violet-600 dark:text-violet-400" />
                <h2 className="text-lg font-semibold text-gray-800 dark:text-white">{t('health', 'systemInformation')}</h2>
                {!hasJetsonData && connection.online && (
                  <span className="text-xs px-2 py-0.5 rounded-full bg-gray-100 dark:bg-gray-900/40 text-gray-700 dark:text-gray-400">
                    Waiting for data...
                  </span>
                )}
              </div>
              {connection.online && (
                <button
                  onClick={() => setShowRebootConfirm(true)}
                  disabled={isRebooting}
                  className="flex items-center gap-2 px-4 py-2 text-sm font-medium text-red-700 dark:text-red-400 bg-red-50 dark:bg-red-900/20 rounded-lg hover:bg-red-100 dark:hover:bg-red-900/30 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  <Power className="w-4 h-4" />
                  <span>{isRebooting ? 'Rebooting...' : 'Reboot System'}</span>
                </button>
              )}
            </div>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              <div>
                <p className="text-sm text-gray-500 dark:text-gray-400">{t('health', 'model')}</p>
                <p className="text-sm font-medium text-gray-800 dark:text-white">
                  {hasJetsonData ? data.board.model : 'N/A'}
                </p>
              </div>
              <div>
                <p className="text-sm text-gray-500 dark:text-gray-400">{t('health', 'powerMode')}</p>
                <p className="text-sm font-medium text-gray-800 dark:text-white">
                  {hasJetsonData ? data.board.powerMode : 'N/A'}
                </p>
              </div>
              <div>
                <p className="text-sm text-gray-500 dark:text-gray-400">{t('health', 'uptime')}</p>
                <p className="text-sm font-medium text-gray-800 dark:text-white">
                  {hasJetsonData ? data.board.uptime : 'N/A'}
                </p>
              </div>
              <div>
                <p className="text-sm text-gray-500 dark:text-gray-400">{t('health', 'jetpack')}</p>
                <p className="text-sm font-medium text-gray-800 dark:text-white">
                  {hasJetsonData ? data.board.jetpack : 'N/A'}
                </p>
              </div>
              <div>
                <p className="text-sm text-gray-500 dark:text-gray-400">{t('health', 'serialNumber')}</p>
                <p className="text-sm font-medium text-gray-800 dark:text-white">
                  {hasJetsonData ? data.board.serialNumber : 'N/A'}
                </p>
              </div>
              <div>
                <p className="text-sm text-gray-500 dark:text-gray-400">{t('health', 'jetsonClocks')}</p>
                <p className="text-sm font-medium text-gray-800 dark:text-white">
                  {hasJetsonData ? (data.board.jetsonClocks ? t('health', 'active') : t('health', 'inactive')) : 'N/A'}
                </p>
              </div>
            </div>
          </div>

          {/* Connectivity Card */}
          <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-6 border border-gray-100 dark:border-botbot-darker">
            <div className="flex items-center gap-3 mb-4">
              <Signal className="w-5 h-5 text-violet-600 dark:text-violet-400" />
              <h2 className="text-lg font-semibold text-gray-800 dark:text-white">{t('health', 'connectivity')}</h2>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            {/* Network Mode */}
            <div className="space-y-2">
              <p className="text-sm text-gray-500 dark:text-gray-400">{t('health', 'networkMode')}</p>
              <div className="flex items-center gap-2">
                {networkModeStatus.mode === 'wifi' && (
                  <>
                    <Wifi className="h-5 w-5 text-green-600 dark:text-green-500" />
                    <div>
                      <p className="text-sm font-medium text-gray-800 dark:text-white">WiFi</p>
                      {networkModeStatus.ssid && (
                        <p className="text-xs text-gray-500 dark:text-gray-400">{networkModeStatus.ssid}</p>
                      )}
                    </div>
                  </>
                )}
                {networkModeStatus.mode === '4g' && (
                  <>
                    <Smartphone className="h-5 w-5 text-blue-600 dark:text-blue-500" />
                    <div>
                      <p className="text-sm font-medium text-gray-800 dark:text-white">4G/Cellular</p>
                      {networkModeStatus.interface && (
                        <p className="text-xs text-gray-500 dark:text-gray-400">{networkModeStatus.interface}</p>
                      )}
                    </div>
                  </>
                )}
                {networkModeStatus.mode === 'hotspot' && (
                  <>
                    <Radio className="h-5 w-5 text-orange-600 dark:text-orange-500" />
                    <div>
                      <p className="text-sm font-medium text-gray-800 dark:text-white">Hotspot</p>
                      {networkModeStatus.ssid && (
                        <p className="text-xs text-gray-500 dark:text-gray-400">{networkModeStatus.ssid}</p>
                      )}
                    </div>
                  </>
                )}
                {networkModeStatus.mode === 'offline' && (
                  <>
                    <Globe className="h-5 w-5 text-gray-500 dark:text-gray-400" />
                    <div>
                      <p className="text-sm font-medium text-gray-800 dark:text-white">Offline</p>
                      <p className="text-xs text-gray-500 dark:text-gray-400">No network</p>
                    </div>
                  </>
                )}
              </div>
            </div>

            {/* Latency */}
            <div className="space-y-2">
              <p className="text-sm text-gray-500 dark:text-gray-400">{t('health', 'latency')}</p>
              <div className="flex items-center gap-2">
                <Clock className="h-5 w-5 text-violet-600 dark:text-violet-400" />
                <div>
                  <p className={`text-sm font-medium ${
                    networkMetrics.latency < 0 ? 'text-gray-400 dark:text-gray-500' :
                    networkMetrics.latency < 50 ? 'text-green-600 dark:text-green-400' :
                    networkMetrics.latency < 150 ? 'text-yellow-600 dark:text-yellow-400' :
                    'text-red-600 dark:text-red-400'
                  }`}>
                    {formatLatency(networkMetrics.latency)}
                  </p>
                  <p className="text-xs text-gray-500 dark:text-gray-400">
                    {networkMetrics.latency < 0 ? 'Measuring...' :
                     networkMetrics.latency < 50 ? 'Excellent' :
                     networkMetrics.latency < 150 ? 'Good' : 'Poor'}
                  </p>
                </div>
              </div>
            </div>

            {/* Download Speed */}
            <div className="space-y-2">
              <p className="text-sm text-gray-500 dark:text-gray-400">{t('health', 'download')}</p>
              <div className="flex items-center gap-2">
                <DownloadCloud className="h-5 w-5 text-green-500 dark:text-green-400" />
                <div>
                  <p className="text-sm font-medium text-gray-800 dark:text-white">
                    {formatDataRate(networkMetrics.dataIn)}
                  </p>
                  <p className="text-xs text-gray-500 dark:text-gray-400">Incoming data</p>
                </div>
              </div>
            </div>

            {/* Upload Speed */}
            <div className="space-y-2">
              <p className="text-sm text-gray-500 dark:text-gray-400">{t('health', 'upload')}</p>
              <div className="flex items-center gap-2">
                <UploadCloud className="h-5 w-5 text-blue-500 dark:text-blue-400" />
                <div>
                  <p className="text-sm font-medium text-gray-800 dark:text-white">
                    {formatDataRate(networkMetrics.dataOut)}
                  </p>
                  <p className="text-xs text-gray-500 dark:text-gray-400">Outgoing data</p>
                </div>
              </div>
            </div>
          </div>

          {/* Connection Quality Bar */}
          <div className="mt-4 pt-4 border-t border-gray-100 dark:border-gray-700">
            <div className="flex justify-between items-center mb-2">
              <span className="text-sm text-gray-600 dark:text-gray-400">{t('health', 'connectionQuality')}</span>
              <span className="text-sm font-medium text-gray-800 dark:text-white">
                {networkMetrics.latency < 0 ? 'Measuring...' :
                 networkMetrics.latency < 50 && networkMetrics.dataIn > 0 ? 'Excellent' :
                 networkMetrics.latency < 150 && networkMetrics.dataIn > 0 ? 'Good' :
                 networkMetrics.dataIn > 0 ? 'Fair' : 'Poor'}
              </span>
            </div>
            <div className="h-2 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden">
              <div
                className={`h-full transition-all duration-500 ${
                  networkMetrics.latency < 0 ? 'bg-gray-400' :
                  networkMetrics.latency < 50 && networkMetrics.dataIn > 0 ? 'bg-green-500' :
                  networkMetrics.latency < 150 && networkMetrics.dataIn > 0 ? 'bg-yellow-500' :
                  networkMetrics.dataIn > 0 ? 'bg-orange-500' : 'bg-red-500'
                }`}
                style={{ 
                  width: `${
                    networkMetrics.latency < 0 ? 0 :
                    networkMetrics.latency < 50 && networkMetrics.dataIn > 0 ? 100 :
                    networkMetrics.latency < 150 && networkMetrics.dataIn > 0 ? 75 :
                    networkMetrics.dataIn > 0 ? 50 : 25
                  }%` 
                }}
              />
            </div>
          </div>
        </div>
      </div>

        {/* WiFi Settings Panel */}
        <WifiControlPanel />

        {/* Main Content - Two Column Layout */}
        <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
          {/* CPU Usage */}
          <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-6 border border-gray-100 dark:border-botbot-darker">
            <div className="flex items-center gap-3 mb-4">
              <Cpu className="w-5 h-5 text-violet-600 dark:text-violet-400" />
              <h2 className="text-lg font-semibold text-gray-800 dark:text-white">{t('health', 'cpuUsage')}</h2>
              {!hasJetsonData && connection.online && (
                <span className="text-xs px-2 py-0.5 rounded-full bg-gray-100 dark:bg-gray-900/40 text-gray-700 dark:text-gray-400">
                  N/A
                </span>
              )}
            </div>
            {/* Real-time CPU usage graph */}
            {hasJetsonData && data.cpus.length > 0 ? (
              <>
                <CPUUsageGraph
                  cpus={data.cpus}
                  t={t}
                />
                {/* Individual CPU cores */}
                <div className="mt-4 pt-4 border-t border-gray-200 dark:border-gray-700">
                  <p className="text-xs text-gray-500 dark:text-gray-400 mb-2 font-medium">Individual Cores</p>
                  <div className="grid grid-cols-2 md:grid-cols-3 gap-3">
                    {data.cpus.map((cpu) => (
                      <CpuCore key={cpu.id} {...cpu} t={t} />
                    ))}
                  </div>
                </div>
              </>
            ) : (
              <div className="text-center text-gray-500 dark:text-gray-400 py-4">
                {!connection.online ? t('health', 'notConnectedDescription') :
                 !hasJetsonData ? 'Waiting for CPU data...' :
                 t('health', 'noCpuData')}
              </div>
            )}
          </div>

          {/* GPU Usage */}
          <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-6 border border-gray-100 dark:border-botbot-darker">
              <div className="flex items-center gap-3 mb-4">
                <Server className="w-5 h-5 text-violet-600 dark:text-violet-400" />
                <h2 className="text-lg font-semibold text-gray-800 dark:text-white">{t('health', 'gpuStatus')}</h2>
                {!hasJetsonData && connection.online && (
                  <span className="text-xs px-2 py-0.5 rounded-full bg-gray-100 dark:bg-gray-900/40 text-gray-700 dark:text-gray-400">
                    N/A
                  </span>
                )}
              </div>
              {/* Real-time GPU usage graph */}
              {hasJetsonData ? (
                <GPUUsageGraph
                  usage={data.gpu.usage}
                  frequency={data.gpu.freq}
                  t={t}
                />
              ) : (
                <div className="text-center text-gray-500 dark:text-gray-400 py-4">
                  {!connection.online ? t('health', 'notConnectedDescription') :
                   'Waiting for GPU data...'}
                </div>
              )}
          </div>

          {/* Memory Usage */}
          <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-6 border border-gray-100 dark:border-botbot-darker">
              <div className="flex items-center gap-3 mb-4">
                <MemoryStick className="w-5 h-5 text-violet-600 dark:text-violet-400" />
                <h2 className="text-lg font-semibold text-gray-800 dark:text-white">{t('health', 'memory')}</h2>
                {!hasJetsonData && connection.online && (
                  <span className="text-xs px-2 py-0.5 rounded-full bg-gray-100 dark:bg-gray-900/40 text-gray-700 dark:text-gray-400">
                    N/A
                  </span>
                )}
              </div>
              <div className="space-y-4">
                <div>
                  <div className="flex justify-between items-center mb-2">
                    <span className="text-sm text-gray-600 dark:text-gray-400">{t('health', 'ram')}</span>
                    <span className="text-sm font-medium text-gray-800 dark:text-white">
                      {hasJetsonData && data.memory.ram.total > 0
                        ? `${data.memory.ram.used.toFixed(1)}${t('health', 'gigabytes')} / ${data.memory.ram.total.toFixed(1)}${t('health', 'gigabytes')}`
                        : t('health', 'notAvailable')
                      }
                    </span>
                  </div>
                  <div className="h-3 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden">
                    <div
                      className="h-full bg-blue-500 transition-all duration-300"
                      style={{ width: hasJetsonData ? `${data.memory.ram.percentage}%` : '0%' }}
                    />
                  </div>
                </div>
                <div>
                  <div className="flex justify-between items-center mb-2">
                    <span className="text-sm text-gray-600 dark:text-gray-400">{t('health', 'swap')}</span>
                    <span className="text-sm font-medium text-gray-800 dark:text-white">
                      {hasJetsonData && data.memory.swap.total > 0
                        ? `${data.memory.swap.used.toFixed(1)}${t('health', 'gigabytes')} / ${data.memory.swap.total.toFixed(1)}${t('health', 'gigabytes')}`
                        : t('health', 'notAvailable')
                      }
                    </span>
                  </div>
                  <div className="h-3 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden">
                    <div
                      className="h-full bg-purple-500 transition-all duration-300"
                      style={{ width: hasJetsonData ? `${data.memory.swap.percentage}%` : '0%' }}
                    />
                  </div>
                </div>
              </div>
            </div>

          {/* Temperature */}
          <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-6 border border-gray-100 dark:border-botbot-darker">
            <div className="flex items-center gap-3 mb-4">
              <Thermometer className="w-5 h-5 text-violet-600 dark:text-violet-400" />
              <h2 className="text-lg font-semibold text-gray-800 dark:text-white">{t('health', 'temperature')}</h2>
            </div>
            <div className="space-y-1">
              <TempIndicator label={t('health', 'cpu')} value={data.temperatures.cpu} t={t} />
              <TempIndicator label={t('health', 'soc0')} value={data.temperatures.soc0} t={t} />
              <TempIndicator label={t('health', 'soc1')} value={data.temperatures.soc1} t={t} />
              <TempIndicator label={t('health', 'soc2')} value={data.temperatures.soc2} t={t} />
              <TempIndicator label={t('health', 'tj')} value={data.temperatures.tj} t={t} />
            </div>
          </div>

          {/* Power Usage */}
          <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-6 border border-gray-100 dark:border-botbot-darker">
            <div className="flex items-center gap-3 mb-4">
              <Zap className="w-5 h-5 text-violet-600 dark:text-violet-400" />
              <h2 className="text-lg font-semibold text-gray-800 dark:text-white">{t('health', 'powerUsage')}</h2>
            </div>
            {/* Real-time power graph */}
            <PowerUsageGraph
              currentPower={data.power.current}
              averagePower={data.power.average}
              t={t}
            />
            {/* Power rails details */}
            {data.power.rails.length > 0 && (
              <div className="mt-4 pt-4 border-t border-gray-200 dark:border-gray-700">
                <p className="text-xs text-gray-500 dark:text-gray-400 mb-2 font-medium">Power Rails</p>
                <div className="space-y-1">
                  {data.power.rails.map((rail, index) => (
                    <PowerRail key={index} {...rail} t={t} />
                  ))}
                </div>
              </div>
            )}
          </div>

          {/* Fan Control */}
          <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-6 border border-gray-100 dark:border-botbot-darker">
              <div className="flex items-center gap-3 mb-4">
                <Fan className="w-5 h-5 text-violet-600 dark:text-violet-400" />
                <h2 className="text-lg font-semibold text-gray-800 dark:text-white">{t('health', 'fanControl')}</h2>
              </div>
              <div className="space-y-3">
                <div className="flex justify-between items-center">
                  <span className="text-sm text-gray-600 dark:text-gray-400">{t('health', 'speed')}</span>
                  <span className="text-lg font-medium text-gray-800 dark:text-white">{data.fan.speed.toFixed(1)}%</span>
                </div>
                <div className="h-3 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden">
                  <div
                    className="h-full bg-cyan-500 transition-all duration-300"
                    style={{ width: `${data.fan.speed}%` }}
                  />
                </div>
                <div className="flex justify-between text-sm">
                  <span className="text-gray-500 dark:text-gray-500">{t('health', 'mode')}: {data.fan.mode}</span>
                  <span className="text-gray-500 dark:text-gray-500">{t('health', 'control')}: {data.fan.control}</span>
                </div>
              </div>
          </div>

          {/* Storage */}
          <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-6 border border-gray-100 dark:border-botbot-darker">
              <div className="flex items-center gap-3 mb-4">
                <HardDrive className="w-5 h-5 text-violet-600 dark:text-violet-400" />
                <h2 className="text-lg font-semibold text-gray-800 dark:text-white">{t('health', 'storage')}</h2>
              </div>
              <div className="space-y-3">
                <div className="flex justify-between items-center">
                  <span className="text-sm text-gray-600 dark:text-gray-400">{t('health', 'used')}</span>
                  <span className="text-sm font-medium text-gray-800 dark:text-white">
                    {data.disk.total > 0 
                      ? `${data.disk.used.toFixed(1)}${t('health', 'gigabytes')} / ${data.disk.total.toFixed(1)}${t('health', 'gigabytes')}`
                      : t('health', 'notAvailable')
                    }
                  </span>
                </div>
                <div className="h-3 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden">
                  <div
                    className="h-full bg-indigo-500 transition-all duration-300"
                    style={{ width: `${data.disk.percentage}%` }}
                  />
                </div>
                {data.disk.total > 0 && (
                  <span className="text-sm text-gray-500 dark:text-gray-500">
                    {data.disk.percentage.toFixed(1)}{t('health', 'percentUsed')}
                  </span>
                )}
              </div>
          </div>
        </div>

        {/* Bottom Section - State Machine and Diagnostics */}
        <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
          {/* State Machine Status */}
          <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-6 border border-gray-100 dark:border-botbot-darker">
            <div className="flex items-center gap-3 mb-4">
              <Activity className="w-5 h-5 text-violet-600 dark:text-violet-400" />
              <h2 className="text-lg font-semibold text-gray-800 dark:text-white">{t('health', 'stateMachineStatus')}</h2>
            </div>
          <div className="overflow-x-auto">
            <table className="w-full">
              <thead>
                <tr className="border-b border-gray-200 dark:border-gray-700">
                  <th className="text-left py-3 px-4 text-sm font-medium text-gray-600 dark:text-gray-400">{t('health', 'status')}</th>
                  <th className="text-left py-3 px-4 text-sm font-medium text-gray-600 dark:text-gray-400">{t('health', 'node')}</th>
                  <th className="text-left py-3 px-4 text-sm font-medium text-gray-600 dark:text-gray-400">{t('health', 'state')}</th>
                  <th className="text-right py-3 px-4 text-sm font-medium text-gray-600 dark:text-gray-400">{t('health', 'actions')}</th>
                </tr>
              </thead>
              <tbody>
                {nodeStatuses.length > 0 ? (
                  nodeStatuses.map((node) => (
                    <StateMachineRow
                      key={node.name}
                      node={node}
                      onStart={handleStartNode}
                      onStop={handleStopNode}
                      onReset={handleResetNode}
                      t={t}
                    />
                  ))
                ) : (
                  <tr>
                    <td colSpan={4} className="py-8 text-center text-sm text-gray-500 dark:text-gray-400">
                      {connection.online ? 'No state machines available' : 'Connect to robot to view state machines'}
                    </td>
                  </tr>
                )}
              </tbody>
            </table>
          </div>
          </div>

          {/* Robot Diagnostics Section */}
          <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-6 border border-gray-100 dark:border-botbot-darker">
          <div className="flex items-center justify-between mb-4">
            <div className="flex items-center gap-3">
              <AlertCircle className="w-5 h-5 text-violet-600 dark:text-violet-400" />
              <h2 className="text-lg font-semibold text-gray-800 dark:text-white">{t('health', 'robotDiagnostics')}</h2>
            </div>
            <div className="flex items-center gap-4 text-sm">
              {errors.length > 0 && (
                <span className="flex items-center gap-1 px-2 py-1 bg-red-100 dark:bg-red-900/20 text-red-700 dark:text-red-400 rounded-full">
                  <XCircle className="w-3 h-3" />
                  {errors.length} {errors.length === 1 ? 'Error' : 'Errors'}
                </span>
              )}
              {warnings.length > 0 && (
                <span className="flex items-center gap-1 px-2 py-1 bg-yellow-100 dark:bg-yellow-900/20 text-yellow-700 dark:text-yellow-400 rounded-full">
                  <AlertTriangle className="w-3 h-3" />
                  {warnings.length} {warnings.length === 1 ? 'Warning' : 'Warnings'}
                </span>
              )}
              {issues.length === 0 && diagnostics.length > 0 && (
                <span className="flex items-center gap-1 px-2 py-1 bg-green-100 dark:bg-green-900/20 text-green-700 dark:text-green-400 rounded-full">
                  <CheckCircle className="w-3 h-3" />
                  All Systems OK
                </span>
              )}
            </div>
          </div>

          {/* Show issues first if any exist */}
          {issues.length > 0 ? (
            <div className="space-y-3">
              <div className="mb-3">
                <h3 className="text-sm font-medium text-gray-600 dark:text-gray-400 mb-2">Active Issues</h3>
                <div className="grid gap-2">
                  {issues.map((diagnostic, idx) => (
                    <DiagnosticItem key={`${diagnostic.name}-${idx}`} diagnostic={diagnostic} t={t} />
                  ))}
                </div>
              </div>

              {/* Show healthy systems in a collapsed view */}
              {diagnostics.filter(d => d.level === DiagnosticLevel.OK).length > 0 && (
                <details className="group">
                  <summary className="cursor-pointer text-sm text-gray-600 dark:text-gray-400 hover:text-gray-800 dark:hover:text-gray-200 transition-colors">
                    <span className="inline-flex items-center gap-2">
                      <CheckCircle className="w-3 h-3 text-green-500" />
                      {diagnostics.filter(d => d.level === DiagnosticLevel.OK).length} systems operating normally
                    </span>
                  </summary>
                  <div className="mt-2 grid gap-2">
                    {diagnostics.filter(d => d.level === DiagnosticLevel.OK).map((diagnostic, idx) => (
                      <DiagnosticItem key={`${diagnostic.name}-ok-${idx}`} diagnostic={diagnostic} t={t} />
                    ))}
                  </div>
                </details>
              )}
            </div>
          ) : diagnostics.length > 0 ? (
            /* Show all healthy systems when no issues */
            <div className="grid gap-2 max-h-96 overflow-y-auto">
              {diagnostics.map((diagnostic, idx) => (
                <DiagnosticItem key={`${diagnostic.name}-${idx}`} diagnostic={diagnostic} t={t} />
              ))}
            </div>
          ) : (
            /* No diagnostics data */
            <div className="text-center py-8 text-sm text-gray-500 dark:text-gray-400">
              {connection.online ?
                'Waiting for diagnostics data...' :
                'Connect to robot to view diagnostics'
              }
            </div>
          )}
          </div>
        </div>
      </div>

      {/* Reboot Confirmation Modal */}
      {showRebootConfirm && (
        <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/50 backdrop-blur-sm">
          <div className="bg-white dark:bg-botbot-dark rounded-lg shadow-xl p-6 max-w-md w-full mx-4 border border-gray-100 dark:border-botbot-darker">
            <div className="flex items-start gap-4">
              <div className="flex-shrink-0">
                <div className="w-12 h-12 rounded-full bg-red-100 dark:bg-red-900/20 flex items-center justify-center">
                  <Power className="w-6 h-6 text-red-600 dark:text-red-400" />
                </div>
              </div>
              <div className="flex-1">
                <h3 className="text-lg font-semibold text-gray-900 dark:text-white mb-2">
                  Confirm System Reboot
                </h3>
                <p className="text-sm text-gray-600 dark:text-gray-400 mb-4">
                  Are you sure you want to reboot the system? This will temporarily disconnect all services and the robot will be unavailable for a few minutes.
                </p>
                <div className="flex gap-3 justify-end">
                  <button
                    onClick={() => setShowRebootConfirm(false)}
                    disabled={isRebooting}
                    className="px-4 py-2 text-sm font-medium text-gray-700 dark:text-gray-300 bg-gray-50 dark:bg-gray-800 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-700 transition-colors disabled:opacity-50"
                  >
                    Cancel
                  </button>
                  <button
                    onClick={handleSystemReboot}
                    disabled={isRebooting}
                    className="px-4 py-2 text-sm font-medium text-white bg-red-600 rounded-lg hover:bg-red-700 transition-colors disabled:opacity-50 disabled:cursor-not-allowed flex items-center gap-2"
                  >
                    {isRebooting ? (
                      <>
                        <div className="w-4 h-4 border-2 border-white/30 border-t-white rounded-full animate-spin" />
                        <span>Rebooting...</span>
                      </>
                    ) : (
                      <>
                        <Power className="w-4 h-4" />
                        <span>Reboot Now</span>
                      </>
                    )}
                  </button>
                </div>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}