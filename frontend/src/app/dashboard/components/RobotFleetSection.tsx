'use client';

import { useEffect, useState } from 'react';
import { useRouter } from 'next/navigation';
import { Bot, Wifi, WifiOff, Star, Plus, ArrowRight, Loader2 } from 'lucide-react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { Database } from '@/types/database.types';
import Button from '@/components/ui/button';

type Robot = Database['public']['Tables']['robots']['Row'];

interface RobotCardProps {
  robot: Robot;
  isConnected: boolean;
  isConnecting: boolean;
  onConnect: () => void;
}

function RobotCard({ robot, isConnected, isConnecting, onConnect }: RobotCardProps) {
  const statusGradient = isConnected
    ? 'from-green-400 to-emerald-500'
    : 'from-gray-400 to-gray-500';

  const cardVariant = robot.is_favorite ? 'premium' : 'default';

  return (
    <div className={`relative group ${
      cardVariant === 'premium'
        ? 'bg-gradient-to-br from-violet-50/50 to-purple-50/50 dark:from-violet-900/20 dark:to-purple-900/20'
        : 'bg-white dark:bg-botbot-dark'
    } rounded-2xl p-5 shadow-lg border ${
      cardVariant === 'premium'
        ? 'border-violet-200 dark:border-violet-800/50'
        : 'border-gray-100 dark:border-botbot-darker'
    } hover:shadow-2xl hover:scale-[1.02] transition-all duration-500 overflow-hidden`}>

      {/* Animated background gradient */}
      <div className={`absolute inset-0 bg-gradient-to-br ${statusGradient} opacity-0 group-hover:opacity-5 transition-opacity duration-500`} />

      {/* Favorite indicator with glow */}
      {robot.is_favorite && (
        <div className="absolute top-3 right-3">
          <div className="relative">
            <div className="absolute inset-0 bg-yellow-400 rounded-full blur-md opacity-50 animate-pulse" />
            <Star className="relative h-4 w-4 text-yellow-500 fill-yellow-500" />
          </div>
        </div>
      )}

      <div className="relative flex flex-col h-full">
        {/* Robot Info */}
        <div className="flex items-center gap-3 mb-4">
          <div className="relative">
            <div className={`absolute -inset-1 bg-gradient-to-br ${statusGradient} rounded-xl blur opacity-25 group-hover:opacity-50 transition-opacity duration-500`} />
            <div className={`relative rounded-xl p-2.5 bg-gradient-to-br ${
              isConnected
                ? 'from-green-100 to-emerald-100 dark:from-green-900/30 dark:to-emerald-900/30'
                : 'from-gray-100 to-gray-200 dark:from-gray-800 dark:to-gray-700'
            } shadow-lg`}>
              <Bot className={`h-5 w-5 ${
                isConnected
                  ? 'text-green-600 dark:text-green-400'
                  : 'text-gray-600 dark:text-gray-400'
              }`} />
            </div>
          </div>
          <div className="flex-1 min-w-0">
            <h3 className="text-sm font-bold text-gray-900 dark:text-white truncate">
              {robot.name}
            </h3>
            <p className="text-xs text-gray-500 dark:text-gray-400 truncate">
              {robot.type || 'Robot'} • ID: {robot.id.slice(0, 8)}
            </p>
          </div>
        </div>

        {/* Status and Stats */}
        <div className="flex items-center gap-2 mb-3">
          <div className={`flex items-center gap-1.5 px-2 py-1 rounded-full ${
            isConnected
              ? 'bg-green-100 dark:bg-green-900/30'
              : 'bg-gray-100 dark:bg-gray-800'
          }`}>
            {isConnected ? (
              <>
                <div className="relative">
                  <div className="absolute inset-0 bg-green-500 rounded-full animate-ping opacity-75" />
                  <div className="relative w-2 h-2 bg-green-500 rounded-full" />
                </div>
                <span className="text-[10px] font-semibold text-green-600 dark:text-green-400 uppercase">Live</span>
              </>
            ) : (
              <>
                <div className="w-2 h-2 bg-gray-400 rounded-full" />
                <span className="text-[10px] font-semibold text-gray-500 dark:text-gray-400 uppercase">Offline</span>
              </>
            )}
          </div>

          {isConnected && (
            <div className="flex items-center gap-1 px-2 py-1 rounded-full bg-blue-100 dark:bg-blue-900/30">
              <div className="w-1.5 h-1.5 bg-blue-500 rounded-full animate-pulse" />
              <span className="text-[10px] font-semibold text-blue-600 dark:text-blue-400">Active</span>
            </div>
          )}
        </div>

        {/* Action Button */}
        <div className="mt-auto">
          {!isConnected ? (
            <button
              onClick={onConnect}
              disabled={isConnecting}
              className="w-full px-3 py-2 text-xs font-semibold text-white bg-gradient-to-r from-violet-600 to-purple-600 hover:from-violet-700 hover:to-purple-700 rounded-xl shadow-lg hover:shadow-xl transition-all duration-300 disabled:opacity-50 disabled:cursor-not-allowed flex items-center justify-center gap-2 group"
            >
              {isConnecting ? (
                <>
                  <Loader2 className="h-3.5 w-3.5 animate-spin" />
                  <span>Connecting...</span>
                </>
              ) : (
                <>
                  <Wifi className="h-3.5 w-3.5 group-hover:animate-pulse" />
                  <span>Connect</span>
                </>
              )}
            </button>
          ) : (
            <div className="flex items-center justify-center gap-1 px-3 py-2 bg-green-50 dark:bg-green-900/20 rounded-xl">
              <div className="w-2 h-2 bg-green-500 rounded-full animate-pulse" />
              <span className="text-xs font-semibold text-green-700 dark:text-green-400">Connected</span>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

export default function RobotFleetSection() {
  const router = useRouter();
  const { connection, connectionStatus, connectToRobotWithInfo } = useRobotConnection();
  const { supabase, user } = useSupabase();
  const [robots, setRobots] = useState<Robot[]>([]);
  const [loading, setLoading] = useState(true);
  const [connectingRobotId, setConnectingRobotId] = useState<string | null>(null);

  useEffect(() => {
    const fetchRobots = async () => {
      if (!user || !supabase) return;

      try {
        setLoading(true);
        const { data } = await supabase
          .from('robots')
          .select('*')
          .eq('user_id', user.id)
          .order('is_favorite', { ascending: false })
          .order('created_at', { ascending: false })
          .limit(8);

        if (data) {
          setRobots(data);
        }
      } catch (error) {
        console.error('Error fetching robots:', error);
      } finally {
        setLoading(false);
      }
    };

    fetchRobots();
  }, [user, supabase]);

  const handleConnect = async (robot: Robot) => {
    try {
      setConnectingRobotId(robot.id);
      await connectToRobotWithInfo(robot);
    } catch (error) {
      console.error('Failed to connect:', error);
    } finally {
      setConnectingRobotId(null);
    }
  };

  if (loading) {
    return (
      <div className="space-y-4">
        <div className="flex items-center justify-between">
          <h2 className="text-lg font-semibold text-gray-900 dark:text-white">Your Robots</h2>
        </div>
        <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-4 xl:grid-cols-5 2xl:grid-cols-6 gap-3">
          {[1, 2, 3, 4, 5, 6].map((i) => (
            <div key={i} className="bg-white dark:bg-botbot-dark rounded-xl p-4 shadow-sm border border-gray-100 dark:border-botbot-darker">
              <div className="animate-pulse">
                <div className="flex items-center gap-3 mb-3">
                  <div className="h-9 w-9 bg-gray-200 dark:bg-gray-700 rounded-lg" />
                  <div className="flex-1">
                    <div className="h-4 bg-gray-200 dark:bg-gray-700 rounded w-3/4 mb-1" />
                    <div className="h-3 bg-gray-200 dark:bg-gray-700 rounded w-1/2" />
                  </div>
                </div>
                <div className="h-3 bg-gray-200 dark:bg-gray-700 rounded w-1/3" />
              </div>
            </div>
          ))}
        </div>
      </div>
    );
  }

  if (robots.length === 0) {
    return (
      <div className="space-y-4">
        <div className="flex items-center justify-between">
          <h2 className="text-lg font-semibold text-gray-900 dark:text-white">Your Robots</h2>
        </div>
        <div className="bg-white dark:bg-botbot-dark rounded-xl p-8 shadow-sm border border-gray-100 dark:border-botbot-darker text-center">
          <div className="max-w-md mx-auto">
            <div className="rounded-full p-3 bg-gradient-to-br from-violet-100 to-purple-100 dark:from-violet-900/30 dark:to-purple-900/30 inline-flex mb-3">
              <Bot className="h-8 w-8 text-violet-600 dark:text-violet-400" />
            </div>
            <h3 className="text-lg font-semibold text-gray-900 dark:text-white mb-1">
              No Robots Yet
            </h3>
            <p className="text-sm text-gray-600 dark:text-gray-400 mb-4">
              Get started by adding your first robot
            </p>
            <Button
              label="Add Your First Robot"
              colorPalette="default"
              onClick={() => router.push('/fleet')}
              customClasses="inline-flex items-center gap-2"
            />
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-5">
      <div className="flex items-center justify-between">
        <div>
          <h2 className="text-lg font-bold text-gray-900 dark:text-white">Your Robot Fleet</h2>
          <p className="text-xs text-gray-600 dark:text-gray-400 mt-1">Manage and monitor your connected robots</p>
        </div>
        <button
          onClick={() => router.push('/fleet')}
          className="flex items-center gap-2 px-4 py-2 text-sm font-semibold text-violet-600 dark:text-violet-400 bg-violet-50 dark:bg-violet-900/20 hover:bg-violet-100 dark:hover:bg-violet-900/30 rounded-xl transition-all duration-300 group"
        >
          <span>Manage Fleet</span>
          <ArrowRight className="h-3.5 w-3.5 transform group-hover:translate-x-1 transition-transform" />
        </button>
      </div>

      <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4 xl:grid-cols-5 2xl:grid-cols-6 gap-4">
        {robots.map((robot) => (
          <RobotCard
            key={robot.id}
            robot={robot}
            isConnected={connection.connectedRobot?.id === robot.id && connection.online}
            isConnecting={connectingRobotId === robot.id && connectionStatus === 'connecting'}
            onConnect={() => handleConnect(robot)}
          />
        ))}

        {/* Add robot card with enhanced design */}
        <div
          onClick={() => router.push('/fleet')}
          className="relative bg-gradient-to-br from-gray-50 to-gray-100 dark:from-gray-800/50 dark:to-gray-900/50 rounded-2xl p-5 shadow-lg border-2 border-dashed border-gray-300 dark:border-gray-700 hover:border-violet-400 dark:hover:border-violet-600 hover:shadow-xl hover:scale-[1.02] transition-all duration-500 cursor-pointer group min-h-[180px] flex items-center justify-center overflow-hidden"
        >
          {/* Animated background effect */}
          <div className="absolute inset-0 bg-gradient-to-br from-violet-500/0 to-purple-500/0 group-hover:from-violet-500/5 group-hover:to-purple-500/5 transition-all duration-500" />

          <div className="relative text-center">
            <div className="relative inline-flex mb-3">
              <div className="absolute -inset-2 bg-violet-400/20 rounded-2xl blur-xl opacity-0 group-hover:opacity-100 transition-opacity duration-500" />
              <div className="relative rounded-xl p-3 bg-gradient-to-br from-gray-100 to-gray-200 dark:from-gray-700 dark:to-gray-800 group-hover:from-violet-100 group-hover:to-purple-100 dark:group-hover:from-violet-900/50 dark:group-hover:to-purple-900/50 shadow-lg transition-all duration-300">
                <Plus className="h-6 w-6 text-gray-500 dark:text-gray-400 group-hover:text-violet-600 dark:group-hover:text-violet-400 transition-colors group-hover:rotate-90 duration-500" />
              </div>
            </div>
            <h3 className="text-sm font-bold text-gray-700 dark:text-gray-300 group-hover:text-violet-700 dark:group-hover:text-violet-400 transition-colors">
              Add Robot
            </h3>
            <p className="text-[10px] text-gray-500 dark:text-gray-500 mt-1 group-hover:text-violet-600/70 dark:group-hover:text-violet-400/70 transition-colors">
              Connect new device
            </p>
          </div>
        </div>
      </div>
    </div>
  );
}