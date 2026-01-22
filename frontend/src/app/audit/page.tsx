'use client';

import { useEffect, useState } from 'react';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { format, subDays, startOfDay, endOfDay, parseISO } from 'date-fns';
import {
  Search,
  Filter,
  Download,
  Activity,
  Bot,
  Command,
  Settings,
  Database as DatabaseIcon,
  RefreshCw,
  Trash2,
  AlertTriangle,
  Navigation,
  Volume2,
  Camera,
  Shield,
  FileDown,
  Map,
  TrendingUp,
  Clock,
  Users,
  BarChart3,
  PieChart,
  Calendar
} from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';
import { auditLogger } from '@/utils/audit-logger';

// Force dynamic rendering
export const dynamic = 'force-dynamic';

interface AuditLog {
  id: string;
  user_id: string;
  event_type: string;
  event_action: string;
  event_details: any;
  robot_id: string | null;
  robot_name: string | null;
  ip_address: string | null;
  user_agent: string | null;
  created_at: string;
}

interface AuditStats {
  totalEvents: number;
  eventsToday: number;
  uniqueRobots: number;
  mostActiveRobot: string | null;
  eventsByType: Record<string, number>;
  eventsByAction: Record<string, number>;
  hourlyDistribution: number[];
  criticalEvents: number;
}

const eventTypeColors: Record<string, string> = {
  auth: 'bg-blue-100 text-blue-800 dark:bg-blue-900/20 dark:text-blue-400',
  robot: 'bg-green-100 text-green-800 dark:bg-green-900/20 dark:text-green-400',
  command: 'bg-purple-100 text-purple-800 dark:bg-purple-900/20 dark:text-purple-400',
  system: 'bg-gray-100 text-gray-800 dark:bg-gray-900/20 dark:text-gray-400',
  data: 'bg-yellow-100 text-yellow-800 dark:bg-yellow-900/20 dark:text-yellow-400',
  mission: 'bg-indigo-100 text-indigo-800 dark:bg-indigo-900/20 dark:text-indigo-400',
  navigation: 'bg-cyan-100 text-cyan-800 dark:bg-cyan-900/20 dark:text-cyan-400',
  audio: 'bg-pink-100 text-pink-800 dark:bg-pink-900/20 dark:text-pink-400',
  camera: 'bg-teal-100 text-teal-800 dark:bg-teal-900/20 dark:text-teal-400',
  safety: 'bg-red-100 text-red-800 dark:bg-red-900/20 dark:text-red-400',
  export: 'bg-orange-100 text-orange-800 dark:bg-orange-900/20 dark:text-orange-400',
};

const eventTypeIcons: Record<string, React.ComponentType<any>> = {
  auth: Activity,
  robot: Bot,
  command: Command,
  system: Settings,
  data: DatabaseIcon,
  mission: Map,
  navigation: Navigation,
  audio: Volume2,
  camera: Camera,
  safety: Shield,
  export: FileDown,
};

const criticalActions = [
  'emergency_stop',
  'collision_avoided',
  'safety_override',
  'robot_connection_failed',
  'navigation_failed',
  'mission_failed',
  'robot_battery_critical'
];

export default function AuditPage() {
  const { user, supabase } = useSupabase();
  const [logs, setLogs] = useState<AuditLog[]>([]);
  const [filteredLogs, setFilteredLogs] = useState<AuditLog[]>([]);
  const [loading, setLoading] = useState(true);
  const [searchTerm, setSearchTerm] = useState('');
  const [selectedEventType, setSelectedEventType] = useState<string>('all');
  const [selectedDateRange, setSelectedDateRange] = useState('7');
  const [isRefreshing, setIsRefreshing] = useState(false);
  const [showDeleteModal, setShowDeleteModal] = useState(false);
  const [isDeleting, setIsDeleting] = useState(false);
  const [stats, setStats] = useState<AuditStats>({
    totalEvents: 0,
    eventsToday: 0,
    uniqueRobots: 0,
    mostActiveRobot: null,
    eventsByType: {},
    eventsByAction: {},
    hourlyDistribution: Array(24).fill(0),
    criticalEvents: 0
  });
  const [viewMode, setViewMode] = useState<'timeline' | 'analytics'>('analytics');

  useEffect(() => {
    document.title = 'Advanced Audit - BotBot';
  }, []);

  // Fetch audit logs with date filtering
  const fetchLogs = async () => {
    if (!supabase || !user) return;

    try {
      setIsRefreshing(true);

      // Calculate date range
      const fromDate = selectedDateRange === 'all'
        ? new Date('2020-01-01')
        : subDays(new Date(), parseInt(selectedDateRange));

      const { data, error } = await supabase
        .from('audit_logs')
        .select('*')
        .eq('user_id', user.id)
        .gte('created_at', fromDate.toISOString())
        .order('created_at', { ascending: false })
        .limit(1000);

      if (error) throw error;

      setLogs(data || []);
      setFilteredLogs(data || []);
      calculateStats(data || []);
    } catch (error) {
      console.error('Error fetching audit logs:', error);
    } finally {
      setLoading(false);
      setIsRefreshing(false);
    }
  };

  // Calculate statistics from logs
  const calculateStats = (logData: AuditLog[]) => {
    const today = startOfDay(new Date());
    const stats: AuditStats = {
      totalEvents: logData.length,
      eventsToday: 0,
      uniqueRobots: 0,
      mostActiveRobot: null,
      eventsByType: {},
      eventsByAction: {},
      hourlyDistribution: Array(24).fill(0),
      criticalEvents: 0
    };

    const robotCounts: Record<string, number> = {};
    const uniqueRobotIds = new Set<string>();

    logData.forEach(log => {
      // Count events today
      if (new Date(log.created_at) >= today) {
        stats.eventsToday++;
      }

      // Count by type
      stats.eventsByType[log.event_type] = (stats.eventsByType[log.event_type] || 0) + 1;

      // Count by action
      stats.eventsByAction[log.event_action] = (stats.eventsByAction[log.event_action] || 0) + 1;

      // Count critical events
      if (criticalActions.includes(log.event_action)) {
        stats.criticalEvents++;
      }

      // Track unique robots
      if (log.robot_id) {
        uniqueRobotIds.add(log.robot_id);
        robotCounts[log.robot_name || log.robot_id] = (robotCounts[log.robot_name || log.robot_id] || 0) + 1;
      }

      // Calculate hourly distribution
      const hour = new Date(log.created_at).getHours();
      stats.hourlyDistribution[hour]++;
    });

    stats.uniqueRobots = uniqueRobotIds.size;

    // Find most active robot
    if (Object.keys(robotCounts).length > 0) {
      const maxCount = Math.max(...Object.values(robotCounts));
      stats.mostActiveRobot = Object.keys(robotCounts).find(key => robotCounts[key] === maxCount) || null;
    }

    setStats(stats);
  };

  useEffect(() => {
    fetchLogs();
  }, [supabase, user, selectedDateRange]);

  // Filter logs based on search and event type
  useEffect(() => {
    let filtered = logs;

    // Filter by event type
    if (selectedEventType !== 'all') {
      filtered = filtered.filter(log => log.event_type === selectedEventType);
    }

    // Filter by search term
    if (searchTerm) {
      filtered = filtered.filter(log =>
        log.event_action.toLowerCase().includes(searchTerm.toLowerCase()) ||
        log.robot_name?.toLowerCase().includes(searchTerm.toLowerCase()) ||
        JSON.stringify(log.event_details).toLowerCase().includes(searchTerm.toLowerCase())
      );
    }

    setFilteredLogs(filtered);
  }, [logs, searchTerm, selectedEventType]);

  // Export logs with enhanced format
  const exportLogs = () => {
    const csv = [
      ['Date', 'Time', 'Event Type', 'Action', 'Robot', 'Robot ID', 'IP Address', 'User Agent', 'Details'],
      ...filteredLogs.map(log => [
        format(new Date(log.created_at), 'yyyy-MM-dd'),
        format(new Date(log.created_at), 'HH:mm:ss'),
        log.event_type,
        log.event_action,
        log.robot_name || '-',
        log.robot_id || '-',
        log.ip_address || '-',
        log.user_agent || '-',
        JSON.stringify(log.event_details || {})
      ])
    ].map(row => row.map(cell => `"${cell}"`).join(',')).join('\n');

    const blob = new Blob([csv], { type: 'text/csv' });
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `audit-logs-${format(new Date(), 'yyyy-MM-dd-HHmmss')}.csv`;
    a.click();

    // Log the export action
    auditLogger.logDataExported('audit_logs', 'csv', filteredLogs.length);
  };

  // Delete all logs
  const deleteAllLogs = async () => {
    if (!supabase || !user) return;

    setIsDeleting(true);
    try {
      const { error } = await supabase
        .from('audit_logs')
        .delete()
        .eq('user_id', user.id);

      if (error) throw error;

      // Log the deletion action
      await auditLogger.log({
        event_type: 'system',
        event_action: 'settings_updated',
        event_details: {
          action: 'deleted_all_audit_logs',
          count: logs.length
        }
      });

      setLogs([]);
      setFilteredLogs([]);
      setShowDeleteModal(false);
      calculateStats([]);
    } catch (error) {
      console.error('Error deleting audit logs:', error);
    } finally {
      setIsDeleting(false);
    }
  };

  // Format action names for display
  const formatAction = (action: string) => {
    return action.split('_').map(word =>
      word.charAt(0).toUpperCase() + word.slice(1)
    ).join(' ');
  };

  if (loading) {
    return (
      <div className="w-full h-full flex items-center justify-center">
        <div className="text-center">
          <div className="w-12 h-12 border-4 border-purple-500 border-t-transparent rounded-full animate-spin mx-auto mb-4"></div>
          <p className="text-gray-600 dark:text-gray-400">Loading audit data...</p>
        </div>
      </div>
    );
  }

  return (
    <div className="w-full h-full overflow-hidden">
      <div className="h-full overflow-y-auto p-6">
        <div className="max-w-7xl mx-auto">
          {/* Header */}
          <div className="mb-6">
            <h1 className="text-3xl font-bold text-gray-800 dark:text-white mb-2">
              Advanced Audit System
            </h1>
            <p className="text-gray-600 dark:text-gray-400">
              Comprehensive tracking and analytics for all operator activities
            </p>
          </div>

          {/* Quick Stats */}
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-6">
            <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-md p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-sm text-gray-600 dark:text-gray-400">Total Events</p>
                  <p className="text-2xl font-bold text-gray-900 dark:text-white">
                    {stats.totalEvents.toLocaleString()}
                  </p>
                  <p className="text-xs text-gray-500 dark:text-gray-500 mt-1">
                    Last {selectedDateRange === 'all' ? 'all time' : `${selectedDateRange} days`}
                  </p>
                </div>
                <BarChart3 className="w-8 h-8 text-purple-500" />
              </div>
            </div>

            <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-md p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-sm text-gray-600 dark:text-gray-400">Events Today</p>
                  <p className="text-2xl font-bold text-gray-900 dark:text-white">
                    {stats.eventsToday.toLocaleString()}
                  </p>
                  <p className="text-xs text-gray-500 dark:text-gray-500 mt-1">
                    Since midnight
                  </p>
                </div>
                <TrendingUp className="w-8 h-8 text-green-500" />
              </div>
            </div>

            <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-md p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-sm text-gray-600 dark:text-gray-400">Active Robots</p>
                  <p className="text-2xl font-bold text-gray-900 dark:text-white">
                    {stats.uniqueRobots}
                  </p>
                  <p className="text-xs text-gray-500 dark:text-gray-500 mt-1">
                    {stats.mostActiveRobot ? `Most active: ${stats.mostActiveRobot}` : 'No robots active'}
                  </p>
                </div>
                <Bot className="w-8 h-8 text-blue-500" />
              </div>
            </div>

            <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-md p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-sm text-gray-600 dark:text-gray-400">Critical Events</p>
                  <p className="text-2xl font-bold text-gray-900 dark:text-white">
                    {stats.criticalEvents}
                  </p>
                  <p className="text-xs text-gray-500 dark:text-gray-500 mt-1">
                    Safety & failures
                  </p>
                </div>
                <Shield className="w-8 h-8 text-red-500" />
              </div>
            </div>
          </div>

          {/* Controls */}
          <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-md p-4 mb-6">
            <div className="flex flex-col lg:flex-row gap-4">
              {/* View Mode Toggle */}
              <div className="flex gap-2">
                <button
                  onClick={() => setViewMode('analytics')}
                  className={`px-4 py-2 rounded-lg transition-colors flex items-center gap-2 ${
                    viewMode === 'analytics'
                      ? 'bg-purple-600 text-white'
                      : 'bg-gray-200 dark:bg-botbot-dark text-gray-700 dark:text-gray-300'
                  }`}
                >
                  <PieChart className="w-4 h-4" />
                  Analytics
                </button>
                <button
                  onClick={() => setViewMode('timeline')}
                  className={`px-4 py-2 rounded-lg transition-colors flex items-center gap-2 ${
                    viewMode === 'timeline'
                      ? 'bg-purple-600 text-white'
                      : 'bg-gray-200 dark:bg-botbot-dark text-gray-700 dark:text-gray-300'
                  }`}
                >
                  <Clock className="w-4 h-4" />
                  Timeline
                </button>
              </div>

              {/* Search */}
              <div className="flex-1">
                <div className="relative">
                  <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 text-gray-400 w-5 h-5" />
                  <input
                    type="text"
                    placeholder="Search logs..."
                    value={searchTerm}
                    onChange={(e) => setSearchTerm(e.target.value)}
                    className="w-full pl-10 pr-4 py-2 bg-gray-50 dark:bg-botbot-dark border border-gray-200 dark:border-gray-700 rounded-lg focus:outline-none focus:ring-2 focus:ring-purple-500"
                  />
                </div>
              </div>

              {/* Filters */}
              <div className="flex gap-2">
                {/* Date Range */}
                <select
                  value={selectedDateRange}
                  onChange={(e) => setSelectedDateRange(e.target.value)}
                  className="px-4 py-2 bg-gray-50 dark:bg-botbot-dark border border-gray-200 dark:border-gray-700 rounded-lg focus:outline-none focus:ring-2 focus:ring-purple-500"
                >
                  <option value="1">Last 24 hours</option>
                  <option value="7">Last 7 days</option>
                  <option value="30">Last 30 days</option>
                  <option value="90">Last 90 days</option>
                  <option value="all">All time</option>
                </select>

                {/* Event Type Filter */}
                <select
                  value={selectedEventType}
                  onChange={(e) => setSelectedEventType(e.target.value)}
                  className="px-4 py-2 bg-gray-50 dark:bg-botbot-dark border border-gray-200 dark:border-gray-700 rounded-lg focus:outline-none focus:ring-2 focus:ring-purple-500"
                >
                  <option value="all">All Events</option>
                  <option value="auth">Authentication</option>
                  <option value="robot">Robot</option>
                  <option value="command">Commands</option>
                  <option value="system">System</option>
                  <option value="data">Data</option>
                  <option value="mission">Mission</option>
                  <option value="navigation">Navigation</option>
                  <option value="audio">Audio</option>
                  <option value="camera">Camera</option>
                  <option value="safety">Safety</option>
                  <option value="export">Export</option>
                </select>
              </div>

              {/* Actions */}
              <div className="flex gap-2">
                <button
                  onClick={fetchLogs}
                  className={`px-4 py-2 bg-purple-600 text-white rounded-lg hover:bg-purple-700 transition-colors flex items-center gap-2 ${
                    isRefreshing ? 'opacity-75 cursor-not-allowed' : ''
                  }`}
                  disabled={isRefreshing}
                >
                  <RefreshCw className={`w-4 h-4 ${isRefreshing ? 'animate-spin' : ''}`} />
                  Refresh
                </button>
                <button
                  onClick={exportLogs}
                  className="px-4 py-2 bg-gray-600 text-white rounded-lg hover:bg-gray-700 transition-colors flex items-center gap-2"
                >
                  <Download className="w-4 h-4" />
                  Export
                </button>
                <button
                  onClick={() => setShowDeleteModal(true)}
                  disabled={filteredLogs.length === 0}
                  className={`px-4 py-2 rounded-lg transition-colors flex items-center gap-2 ${
                    filteredLogs.length === 0
                      ? 'bg-gray-400 text-gray-200 cursor-not-allowed'
                      : 'bg-red-600 text-white hover:bg-red-700'
                  }`}
                >
                  <Trash2 className="w-4 h-4" />
                  Clear
                </button>
              </div>
            </div>
          </div>

          {/* Content Area */}
          {viewMode === 'analytics' ? (
            <>
              {/* Event Distribution */}
              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
                {/* Event Types */}
                <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-md p-4">
                  <h3 className="text-lg font-semibold text-gray-900 dark:text-white mb-4">
                    Event Distribution by Type
                  </h3>
                  <div className="space-y-3">
                    {Object.entries(stats.eventsByType).map(([type, count]) => {
                      const Icon = eventTypeIcons[type] || Activity;
                      const percentage = (count / stats.totalEvents * 100).toFixed(1);
                      return (
                        <div key={type} className="flex items-center gap-3">
                          <div className={`p-2 rounded-lg ${eventTypeColors[type]}`}>
                            <Icon className="w-4 h-4" />
                          </div>
                          <div className="flex-1">
                            <div className="flex justify-between items-center mb-1">
                              <span className="text-sm font-medium text-gray-700 dark:text-gray-300 capitalize">
                                {type}
                              </span>
                              <span className="text-sm text-gray-500">
                                {count} ({percentage}%)
                              </span>
                            </div>
                            <div className="w-full bg-gray-200 dark:bg-gray-700 rounded-full h-2">
                              <div
                                className="bg-purple-600 h-2 rounded-full transition-all"
                                style={{ width: `${percentage}%` }}
                              />
                            </div>
                          </div>
                        </div>
                      );
                    })}
                  </div>
                </div>

                {/* Hourly Activity */}
                <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-md p-4">
                  <h3 className="text-lg font-semibold text-gray-900 dark:text-white mb-4">
                    24-Hour Activity Pattern
                  </h3>
                  <div className="flex items-end justify-between h-32 gap-1">
                    {stats.hourlyDistribution.map((count, hour) => {
                      const maxCount = Math.max(...stats.hourlyDistribution);
                      const height = maxCount > 0 ? (count / maxCount * 100) : 0;
                      return (
                        <div
                          key={hour}
                          className="flex-1 bg-purple-600 dark:bg-purple-500 rounded-t hover:bg-purple-700 dark:hover:bg-purple-400 transition-colors relative group"
                          style={{ height: `${height}%` }}
                          title={`${hour}:00 - ${count} events`}
                        >
                          <span className="absolute -bottom-6 left-1/2 transform -translate-x-1/2 text-xs text-gray-500 opacity-0 group-hover:opacity-100 transition-opacity">
                            {count}
                          </span>
                        </div>
                      );
                    })}
                  </div>
                  <div className="flex justify-between mt-2 text-xs text-gray-500">
                    <span>00:00</span>
                    <span>06:00</span>
                    <span>12:00</span>
                    <span>18:00</span>
                    <span>23:00</span>
                  </div>
                </div>
              </div>

              {/* Top Actions */}
              <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-md p-4 mb-6">
                <h3 className="text-lg font-semibold text-gray-900 dark:text-white mb-4">
                  Most Frequent Actions
                </h3>
                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-3">
                  {Object.entries(stats.eventsByAction)
                    .sort((a, b) => b[1] - a[1])
                    .slice(0, 9)
                    .map(([action, count]) => (
                      <div
                        key={action}
                        className={`flex items-center justify-between p-3 rounded-lg ${
                          criticalActions.includes(action)
                            ? 'bg-red-50 dark:bg-red-900/20 border border-red-200 dark:border-red-800'
                            : 'bg-gray-50 dark:bg-botbot-dark'
                        }`}
                      >
                        <span className="text-sm font-medium text-gray-700 dark:text-gray-300">
                          {formatAction(action)}
                        </span>
                        <span className={`text-sm font-bold ${
                          criticalActions.includes(action)
                            ? 'text-red-600 dark:text-red-400'
                            : 'text-gray-600 dark:text-gray-400'
                        }`}>
                          {count}
                        </span>
                      </div>
                    ))}
                </div>
              </div>
            </>
          ) : (
            /* Timeline View */
            <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-md overflow-hidden">
              <div className="overflow-x-auto">
                <table className="min-w-full divide-y divide-gray-200 dark:divide-gray-700">
                  <thead className="bg-gray-50 dark:bg-botbot-dark">
                    <tr>
                      <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                        Time
                      </th>
                      <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                        Type
                      </th>
                      <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                        Action
                      </th>
                      <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                        Robot
                      </th>
                      <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                        Details
                      </th>
                    </tr>
                  </thead>
                  <tbody className="bg-white dark:bg-transparent divide-y divide-gray-200 dark:divide-gray-700">
                    <AnimatePresence>
                      {filteredLogs.length === 0 ? (
                        <tr>
                          <td colSpan={5} className="px-6 py-12 text-center">
                            <div className="text-gray-500 dark:text-gray-400">
                              <Activity className="w-12 h-12 mx-auto mb-4 opacity-50" />
                              <p>No audit logs found</p>
                            </div>
                          </td>
                        </tr>
                      ) : (
                        filteredLogs.slice(0, 100).map((log, index) => {
                          const Icon = eventTypeIcons[log.event_type] || Activity;
                          const isCritical = criticalActions.includes(log.event_action);

                          return (
                            <motion.tr
                              key={log.id}
                              initial={{ opacity: 0, x: -20 }}
                              animate={{ opacity: 1, x: 0 }}
                              transition={{ delay: index * 0.02, duration: 0.2 }}
                              className={`hover:bg-gray-50 dark:hover:bg-botbot-dark/50 transition-colors ${
                                isCritical ? 'bg-red-50/50 dark:bg-red-900/10' : ''
                              }`}
                            >
                              <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-white">
                                <div>
                                  <div className="font-medium">
                                    {format(new Date(log.created_at), 'MMM dd, yyyy')}
                                  </div>
                                  <div className="text-xs text-gray-500 dark:text-gray-400">
                                    {format(new Date(log.created_at), 'HH:mm:ss')}
                                  </div>
                                </div>
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap">
                                <div className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${eventTypeColors[log.event_type]}`}>
                                  <Icon className="w-3 h-3 mr-1" />
                                  {log.event_type}
                                </div>
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900 dark:text-white">
                                <div className="flex items-center gap-2">
                                  {isCritical && (
                                    <AlertTriangle className="w-4 h-4 text-red-500" />
                                  )}
                                  {formatAction(log.event_action)}
                                </div>
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500 dark:text-gray-400">
                                {log.robot_name || '-'}
                              </td>
                              <td className="px-6 py-4 text-sm text-gray-500 dark:text-gray-400">
                                {log.event_details && Object.keys(log.event_details).length > 0 ? (
                                  <details className="cursor-pointer">
                                    <summary className="hover:text-purple-600 dark:hover:text-purple-400">
                                      View details
                                    </summary>
                                    <pre className="mt-2 text-xs bg-gray-50 dark:bg-botbot-dark p-2 rounded overflow-x-auto max-w-md">
                                      {JSON.stringify(log.event_details, null, 2)}
                                    </pre>
                                  </details>
                                ) : (
                                  '-'
                                )}
                              </td>
                            </motion.tr>
                          );
                        })
                      )}
                    </AnimatePresence>
                  </tbody>
                </table>
              </div>
            </div>
          )}
        </div>
      </div>

      {/* Delete Confirmation Modal */}
      {showDeleteModal && (
        <>
          <div
            className="fixed inset-0 bg-black/50 backdrop-blur-sm z-50"
            onClick={() => setShowDeleteModal(false)}
          />
          <div className="fixed top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 bg-white dark:bg-botbot-darker rounded-lg shadow-xl z-50 p-6 max-w-md w-full mx-4">
            <div className="flex items-center mb-4">
              <AlertTriangle className="w-6 h-6 text-red-500 mr-3" />
              <h3 className="text-lg font-semibold text-gray-900 dark:text-white">
                Clear All Audit Logs
              </h3>
            </div>

            <p className="text-gray-600 dark:text-gray-300 mb-6">
              Are you sure you want to clear all {logs.length} audit logs? This action cannot be undone.
            </p>

            <div className="flex justify-end gap-3">
              <button
                onClick={() => setShowDeleteModal(false)}
                className="px-4 py-2 bg-gray-200 dark:bg-botbot-dark text-gray-700 dark:text-gray-300 rounded-lg hover:bg-gray-300 dark:hover:bg-botbot-dark/80 transition-colors"
              >
                Cancel
              </button>
              <button
                onClick={deleteAllLogs}
                disabled={isDeleting}
                className={`px-4 py-2 bg-red-600 text-white rounded-lg transition-colors flex items-center gap-2 ${
                  isDeleting ? 'opacity-75 cursor-not-allowed' : 'hover:bg-red-700'
                }`}
              >
                {isDeleting ? (
                  <>
                    <div className="w-4 h-4 border-2 border-white border-t-transparent rounded-full animate-spin" />
                    Deleting...
                  </>
                ) : (
                  <>
                    <Trash2 className="w-4 h-4" />
                    Clear All
                  </>
                )}
              </button>
            </div>
          </div>
        </>
      )}
    </div>
  );
}