'use client';

import { useEffect, useState } from 'react';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { format } from 'date-fns';
import { Search, Filter, Download, Activity, Bot, Command, Settings, Database as DatabaseIcon, RefreshCw, TestTube, Trash2, AlertTriangle, Navigation, Volume2, Camera, Shield, FileDown, Map } from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';
import { auditLogger } from '@/utils/audit-logger';

// Force dynamic rendering to prevent static generation issues
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

export default function LogPage() {
  const { user, supabase } = useSupabase();
  const [logs, setLogs] = useState<AuditLog[]>([]);
  const [filteredLogs, setFilteredLogs] = useState<AuditLog[]>([]);
  const [loading, setLoading] = useState(true);
  const [searchTerm, setSearchTerm] = useState('');
  const [selectedEventType, setSelectedEventType] = useState<string>('all');
  const [isRefreshing, setIsRefreshing] = useState(false);
  const [showDeleteModal, setShowDeleteModal] = useState(false);

  useEffect(() => {
    // Set page title
    document.title = 'Audit - BotBot';
  }, []);
  const [isDeleting, setIsDeleting] = useState(false);

  // Fetch audit logs
  const fetchLogs = async () => {
    if (!supabase || !user) return;
    
    try {
      setIsRefreshing(true);
      const { data, error } = await supabase
        .from('audit_logs')
        .select('*')
        .eq('user_id', user.id)
        .order('created_at', { ascending: false })
        .limit(500);

      if (error) throw error;
      
      setLogs(data || []);
      setFilteredLogs(data || []);
    } catch (error) {
      console.error('Error fetching audit logs:', error);
    } finally {
      setLoading(false);
      setIsRefreshing(false);
    }
  };

  useEffect(() => {
    fetchLogs();
  }, [supabase, user]);

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

  // Export logs as CSV
  const exportLogs = () => {
    const csv = [
      ['Date', 'Time', 'Event Type', 'Action', 'Robot', 'Details'],
      ...filteredLogs.map(log => [
        format(new Date(log.created_at), 'yyyy-MM-dd'),
        format(new Date(log.created_at), 'HH:mm:ss'),
        log.event_type,
        log.event_action,
        log.robot_name || '-',
        JSON.stringify(log.event_details || {})
      ])
    ].map(row => row.map(cell => `"${cell}"`).join(',')).join('\n');

    const blob = new Blob([csv], { type: 'text/csv' });
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `audit-logs-${format(new Date(), 'yyyy-MM-dd')}.csv`;
    a.click();

    // Log the export action
    auditLogger.logLogsDownloaded('audit_logs', {
      from: filteredLogs[filteredLogs.length - 1]?.created_at || new Date().toISOString(),
      to: filteredLogs[0]?.created_at || new Date().toISOString()
    });
  };
  
  // Test audit logging
  const testAuditLog = async () => {
    console.log('[Test] Creating test audit log entry...');
    await auditLogger.log({
      event_type: 'system',
      event_action: 'settings_updated',
      event_details: {
        test: true,
        timestamp: new Date().toISOString(),
        message: 'Test audit log entry'
      }
    });
    console.log('[Test] Test audit log created, refreshing...');
    setTimeout(() => fetchLogs(), 500);
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
          <p className="text-gray-600 dark:text-gray-400">Loading audit logs...</p>
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
              Audit Log
            </h1>
            <p className="text-gray-600 dark:text-gray-400">
              Track all activities and events in your application
            </p>
          </div>

          {/* Controls */}
          <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-md p-4 mb-6">
            <div className="flex flex-col md:flex-row gap-4">
              {/* Search */}
              <div className="flex-1">
                <div className="relative">
                  <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 text-gray-400 w-5 h-5" />
                  <input
                    type="text"
                    placeholder="Search logs..."
                    value={searchTerm}
                    onChange={(e) => setSearchTerm(e.target.value)}
                    className="w-full pl-10 pr-4 py-2 bg-gray-50 dark:bg-botbot-dark border border-gray-200 dark:border-gray-700 rounded-lg focus:outline-none focus:ring-2 focus:ring-purple-500 dark:focus:ring-purple-400"
                  />
                </div>
              </div>

              {/* Event Type Filter */}
              <div className="flex items-center gap-2">
                <Filter className="text-gray-400 w-5 h-5" />
                <select
                  value={selectedEventType}
                  onChange={(e) => setSelectedEventType(e.target.value)}
                  className="px-4 py-2 bg-gray-50 dark:bg-botbot-dark border border-gray-200 dark:border-gray-700 rounded-lg focus:outline-none focus:ring-2 focus:ring-purple-500 dark:focus:ring-purple-400"
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
                  Delete All
                </button>
                {process.env.NODE_ENV === 'development' && (
                  <button
                    onClick={testAuditLog}
                    className="px-4 py-2 bg-green-600 text-white rounded-lg hover:bg-green-700 transition-colors flex items-center gap-2"
                    title="Create a test audit log entry"
                  >
                    <TestTube className="w-4 h-4" />
                    Test Log
                  </button>
                )}
              </div>
            </div>
          </div>

          {/* Logs Table */}
          <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-md overflow-hidden">
            <div className="overflow-x-auto">
              <table className="min-w-full divide-y divide-gray-200 dark:divide-gray-700">
                <thead className="bg-gray-50 dark:bg-botbot-dark">
                  <tr>
                    <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                      Date & Time
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
                      filteredLogs.map((log, index) => {
                        const Icon = eventTypeIcons[log.event_type] || Activity;
                        return (
                          <motion.tr
                            key={log.id}
                            initial={{ opacity: 0, y: 20 }}
                            animate={{ opacity: 1, y: 0 }}
                            transition={{ delay: index * 0.05, duration: 0.3 }}
                            className="hover:bg-gray-50 dark:hover:bg-botbot-dark/50 transition-colors"
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
                              <div className="flex items-center">
                                <div className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${eventTypeColors[log.event_type]}`}>
                                  <Icon className="w-3 h-3 mr-1" />
                                  {log.event_type}
                                </div>
                              </div>
                            </td>
                            <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900 dark:text-white">
                              {formatAction(log.event_action)}
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

          {/* Summary Stats */}
          <div className="mt-6 grid grid-cols-1 md:grid-cols-4 gap-4">
            {Object.entries(
              filteredLogs.reduce((acc, log) => {
                acc[log.event_type] = (acc[log.event_type] || 0) + 1;
                return acc;
              }, {} as Record<string, number>)
            ).map(([type, count]) => {
              const Icon = eventTypeIcons[type] || Activity;
              return (
                <div key={type} className="bg-white dark:bg-botbot-darker rounded-lg shadow-md p-4">
                  <div className="flex items-center justify-between">
                    <div className="flex items-center">
                      <div className={`p-2 rounded-lg ${eventTypeColors[type]}`}>
                        <Icon className="w-5 h-5" />
                      </div>
                      <div className="ml-3">
                        <p className="text-sm font-medium text-gray-600 dark:text-gray-400 capitalize">
                          {type} Events
                        </p>
                        <p className="text-2xl font-semibold text-gray-900 dark:text-white">
                          {count}
                        </p>
                      </div>
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
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
                Delete All Audit Logs
              </h3>
            </div>
            
            <p className="text-gray-600 dark:text-gray-300 mb-6">
              Are you sure you want to delete all {filteredLogs.length} audit logs? This action cannot be undone.
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
                    Delete All
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