'use client';

import { useEffect, useState } from 'react';
import { Bot, Zap, Star, TrendingUp, Clock, BarChart3 } from 'lucide-react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { Database } from '@/types/database.types';

type Robot = Database['public']['Tables']['robots']['Row'];

interface StatCardProps {
  icon: React.ReactNode;
  label: string;
  value: string | number;
  subValue?: string;
  trend?: 'up' | 'down' | 'neutral';
  trendValue?: string;
  gradient: string;
  variant?: 'default' | 'glass' | 'neon' | 'gradient';
  size?: 'small' | 'medium' | 'large';
}

function StatCard({ icon, label, value, subValue, trend, trendValue, gradient, variant = 'default', size = 'medium' }: StatCardProps) {
  const sizeClasses = {
    small: 'p-3',
    medium: 'p-4',
    large: 'p-6'
  };

  const variantClasses = {
    default: 'bg-white dark:bg-botbot-dark border border-gray-100 dark:border-botbot-darker',
    glass: 'bg-white/70 dark:bg-botbot-dark/70 backdrop-blur-md border border-white/20 dark:border-white/10',
    neon: 'bg-gradient-to-br from-gray-900 to-black border border-purple-500/30',
    gradient: `bg-gradient-to-br ${gradient} text-white border-0`
  };

  return (
    <div className={`relative overflow-hidden rounded-2xl ${variantClasses[variant]} ${sizeClasses[size]} shadow-lg hover:shadow-2xl transition-all duration-500 group hover:scale-[1.02]`}>
      {variant !== 'gradient' && (
        <div className={`absolute inset-0 opacity-[0.03] bg-gradient-to-br ${gradient}`} />
      )}

      {/* Animated background effect */}
      {variant === 'neon' && (
        <div className="absolute inset-0 opacity-20">
          <div className={`absolute inset-0 bg-gradient-to-br ${gradient} blur-2xl animate-pulse`} />
        </div>
      )}

      <div className="relative z-10">
        <div className="flex items-start justify-between gap-3">
          <div className="flex-1">
            <p className={`text-xs font-medium ${variant === 'gradient' || variant === 'neon' ? 'text-white/80' : 'text-gray-600 dark:text-gray-400'} uppercase tracking-wider`}>
              {label}
            </p>
            <div className="flex items-baseline gap-2 mt-2">
              <p className={`text-2xl font-bold ${variant === 'gradient' || variant === 'neon' ? 'text-white' : 'text-gray-900 dark:text-white'}`}>
                {value}
              </p>
              {trend && (
                <span className={`text-xs font-semibold ${
                  trend === 'up' ? 'text-green-500' :
                  trend === 'down' ? 'text-red-500' :
                  'text-gray-500'
                }`}>
                  {trend === 'up' ? '↑' : trend === 'down' ? '↓' : '→'} {trendValue}
                </span>
              )}
            </div>
            {subValue && (
              <p className={`text-xs mt-1 ${variant === 'gradient' || variant === 'neon' ? 'text-white/70' : 'text-gray-500 dark:text-gray-500'}`}>
                {subValue}
              </p>
            )}
          </div>
          <div className={`relative ${variant === 'gradient' ? 'bg-white/20' : `bg-gradient-to-br ${gradient}`} rounded-xl p-2.5 shadow-lg group-hover:scale-110 transition-transform duration-300`}>
            {variant === 'neon' && (
              <div className={`absolute inset-0 bg-gradient-to-br ${gradient} rounded-xl blur animate-pulse`} />
            )}
            <div className="relative">
              {icon}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

export default function QuickStats() {
  const { connection, connectionStatus } = useRobotConnection();
  const { supabase, user } = useSupabase();
  const [robots, setRobots] = useState<Robot[]>([]);
  const [totalActions, setTotalActions] = useState<number>(0);
  const [todayActions, setTodayActions] = useState<number>(0);
  const [onlineRobots, setOnlineRobots] = useState<number>(0);
  const [avgSessionTime, setAvgSessionTime] = useState<string>('0m');

  useEffect(() => {
    const fetchRobots = async () => {
      if (!user || !supabase) return;

      try {
        const { data } = await supabase
          .from('robots')
          .select('*')
          .eq('user_id', user.id);

        if (data) {
          setRobots(data);
        }
      } catch (error) {
        console.error('Error fetching robots:', error);
      }
    };

    fetchRobots();
  }, [user, supabase]);

  useEffect(() => {
    const fetchStats = async () => {
      if (!user || !supabase) return;

      try {
        // Fetch total actions count
        const { count } = await supabase
          .from('audit_logs')
          .select('*', { count: 'exact', head: true })
          .eq('user_id', user.id);

        if (count) {
          setTotalActions(count);
        }

        // Fetch today's actions
        const today = new Date();
        today.setHours(0, 0, 0, 0);

        const { count: todayCount } = await supabase
          .from('audit_logs')
          .select('*', { count: 'exact', head: true })
          .eq('user_id', user.id)
          .gte('created_at', today.toISOString());

        if (todayCount) {
          setTodayActions(todayCount);
        }

        // Calculate average session time (mock data for demo)
        const avgMinutes = Math.floor(Math.random() * 120) + 30;
        setAvgSessionTime(`${avgMinutes}m`);
      } catch (error) {
        console.error('Error fetching stats:', error);
      }
    };

    fetchStats();
  }, [user, supabase]);

  // Simulate online robots (in real app, this would come from actual connection status)
  useEffect(() => {
    setOnlineRobots(connection.online ? 1 : 0);
  }, [connection.online]);

  return (
    <div className="space-y-4">
      {/* Main stats grid with varied styles */}
      <div className="grid grid-cols-2 lg:grid-cols-4 gap-4">
        <StatCard
          icon={<Bot className="h-5 w-5 text-white" />}
          label="Fleet Size"
          value={robots.length}
          subValue={robots.length === 1 ? 'robot' : 'robots'}
          trend={robots.length > 0 ? 'up' : 'neutral'}
          trendValue={robots.length > 0 ? '+1 this month' : ''}
          gradient="from-violet-500 to-purple-600"
          variant="gradient"
        />

        <StatCard
          icon={
            <div className="relative">
              <Zap className="h-5 w-5 text-white" />
              {onlineRobots > 0 && (
                <div className="absolute -top-1 -right-1">
                  <div className="h-2 w-2 bg-green-400 rounded-full animate-pulse" />
                </div>
              )}
            </div>
          }
          label="Online Now"
          value={`${onlineRobots}/${robots.length}`}
          subValue={onlineRobots > 0 ? 'active' : 'all offline'}
          gradient={onlineRobots > 0 ? "from-green-500 to-emerald-600" : "from-gray-500 to-gray-600"}
          variant="glass"
        />

        <StatCard
          icon={<Star className="h-5 w-5 text-white" />}
          label="Favorites"
          value={robots.filter(r => r.is_favorite).length}
          subValue={robots.filter(r => r.is_favorite).length === 1 ? 'robot' : 'robots'}
          gradient="from-yellow-500 to-orange-600"
          variant="default"
        />

        <StatCard
          icon={<TrendingUp className="h-5 w-5 text-white" />}
          label="Total Actions"
          value={totalActions > 999 ? `${(totalActions / 1000).toFixed(1)}k` : totalActions}
          subValue="all time"
          trend="up"
          trendValue="+12%"
          gradient="from-blue-500 to-cyan-600"
          variant="neon"
        />
      </div>

      {/* Secondary stats with different style */}
      <div className="grid grid-cols-2 gap-4">
        <StatCard
          icon={<Clock className="h-4 w-4 text-white" />}
          label="Avg Session"
          value={avgSessionTime}
          subValue="per session"
          gradient="from-indigo-500 to-blue-600"
          variant="glass"
          size="small"
        />

        <StatCard
          icon={<BarChart3 className="h-4 w-4 text-white" />}
          label="Today's Actions"
          value={todayActions}
          subValue="since midnight"
          trend={todayActions > 10 ? 'up' : 'down'}
          trendValue={todayActions > 10 ? 'Active day' : 'Slow day'}
          gradient="from-pink-500 to-rose-600"
          variant="glass"
          size="small"
        />
      </div>
    </div>
  );
}