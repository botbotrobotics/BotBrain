'use client';

import { useEffect, useState } from 'react';
import {
  Bot, Zap, Star, TrendingUp, Clock, BarChart3,
  Activity, Brain, Target, Database as DatabaseIcon, Music, Calendar,
  Cpu, Gauge
} from 'lucide-react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useSupabase } from '@/contexts/SupabaseProvider';
import type { Database } from '@/types/database.types';

type Robot = Database['public']['Tables']['robots']['Row'];
type UserProfile = Database['public']['Tables']['user_profiles']['Row'];

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
  loading?: boolean;
}

function StatCard({
  icon,
  label,
  value,
  subValue,
  trend,
  trendValue,
  gradient,
  variant = 'default',
  size = 'medium',
  loading = false
}: StatCardProps) {
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
              {loading ? (
                <div className="h-8 w-20 bg-gray-200 dark:bg-gray-700 rounded animate-pulse" />
              ) : (
                <>
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
                </>
              )}
            </div>
            {subValue && !loading && (
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

export default function EnhancedQuickStats() {
  const { connection } = useRobotConnection();
  const { supabase, user } = useSupabase();
  const [loading, setLoading] = useState(true);

  // Basic stats
  const [robots, setRobots] = useState<Robot[]>([]);
  const [userProfile, setUserProfile] = useState<UserProfile | null>(null);
  const [totalActions, setTotalActions] = useState<number>(0);
  const [todayActions, setTodayActions] = useState<number>(0);
  const [weeklyActions, setWeeklyActions] = useState<number>(0);
  const [monthlyActions, setMonthlyActions] = useState<number>(0);

  // Enhanced stats
  const [memberDuration, setMemberDuration] = useState<string>('New');
  const [peakHour, setPeakHour] = useState<string>('--');
  const [mostUsedRobot, setMostUsedRobot] = useState<string>('None');
  const [activeDays, setActiveDays] = useState<number>(0);
  const [missionsTotal, setMissionsTotal] = useState<number>(0);
  const [detectionsCount, setDetectionsCount] = useState<number>(0);
  const [avgConfidence, setAvgConfidence] = useState<number>(0);
  const [soundsCount, setSoundsCount] = useState<number>(0);
  const [storageUsed, setStorageUsed] = useState<string>('0 MB');

  useEffect(() => {
    const fetchAllStats = async () => {
      if (!user || !supabase) return;

      setLoading(true);

      try {
        // Fetch user profile
        const { data: profile } = await supabase
          .from('user_profiles')
          .select('*')
          .eq('user_id', user.id)
          .single();

        if (profile) {
          setUserProfile(profile);

          // Calculate member duration from profile created_at
          const createdDate = new Date(profile.created_at);
          const now = new Date();
          const diffMs = now.getTime() - createdDate.getTime();
          const diffDays = Math.floor(diffMs / (1000 * 60 * 60 * 24));
          const diffMonths = Math.floor(diffDays / 30);
          const diffYears = Math.floor(diffDays / 365);

          if (diffYears > 0) {
            setMemberDuration(`${diffYears}+ year${diffYears > 1 ? 's' : ''}`);
          } else if (diffMonths > 0) {
            setMemberDuration(`${diffMonths} month${diffMonths > 1 ? 's' : ''}`);
          } else if (diffDays > 0) {
            setMemberDuration(`${diffDays} day${diffDays > 1 ? 's' : ''}`);
          } else {
            setMemberDuration('New member');
          }
        }

        // Fetch robots
        const { data: robotsData } = await supabase
          .from('robots')
          .select('*')
          .eq('user_id', user.id);

        if (robotsData) {
          setRobots(robotsData);
        }

        // Fetch comprehensive stats with individual queries
        // Total actions
        const { count: totalCount } = await supabase
          .from('audit_logs')
          .select('*', { count: 'exact', head: true })
          .eq('user_id', user.id);

        setTotalActions(totalCount || 0);

        // Today's actions
        const today = new Date();
        today.setHours(0, 0, 0, 0);

        const { count: todayCount } = await supabase
          .from('audit_logs')
          .select('*', { count: 'exact', head: true })
          .eq('user_id', user.id)
          .gte('created_at', today.toISOString());

        setTodayActions(todayCount || 0);

        // Weekly actions
        const oneWeekAgo = new Date();
        oneWeekAgo.setDate(oneWeekAgo.getDate() - 7);

        const { count: weekCount } = await supabase
          .from('audit_logs')
          .select('*', { count: 'exact', head: true })
          .eq('user_id', user.id)
          .gte('created_at', oneWeekAgo.toISOString());

        setWeeklyActions(weekCount || 0);

        // Monthly actions
        const oneMonthAgo = new Date();
        oneMonthAgo.setMonth(oneMonthAgo.getMonth() - 1);

        const { count: monthCount } = await supabase
          .from('audit_logs')
          .select('*', { count: 'exact', head: true })
          .eq('user_id', user.id)
          .gte('created_at', oneMonthAgo.toISOString());

        setMonthlyActions(monthCount || 0);

        // Get active days count
        const { data: activityDays } = await supabase
          .from('audit_logs')
          .select('created_at')
          .eq('user_id', user.id);

        if (activityDays) {
          const uniqueDays = new Set(
            activityDays.map(log => new Date(log.created_at).toDateString())
          );
          setActiveDays(uniqueDays.size);
        }

        // Calculate peak hour
        const { data: hourlyActivity } = await supabase
          .from('audit_logs')
          .select('created_at')
          .eq('user_id', user.id)
          .gte('created_at', oneMonthAgo.toISOString());

        if (hourlyActivity && hourlyActivity.length > 0) {
          const hourCounts: { [key: number]: number } = {};

          hourlyActivity.forEach(log => {
            const hour = new Date(log.created_at).getHours();
            hourCounts[hour] = (hourCounts[hour] || 0) + 1;
          });

          const peakHourData = Object.entries(hourCounts)
            .sort((a, b) => b[1] - a[1])[0];

          if (peakHourData) {
            const hour = parseInt(peakHourData[0]);
            const period = hour < 12 ? 'AM' : 'PM';
            const displayHour = hour === 0 ? 12 : hour > 12 ? hour - 12 : hour;
            setPeakHour(`${displayHour}:00 ${period}`);
          }
        }

        // Get most used robot
        if (robotsData && robotsData.length > 0) {
          const robotUsagePromises = robotsData.map(async (robot) => {
            const { count } = await supabase
              .from('audit_logs')
              .select('*', { count: 'exact', head: true })
              .eq('robot_id', robot.id);
            return { name: robot.name, count: count || 0 };
          });

          const robotUsages = await Promise.all(robotUsagePromises);
          const topRobot = robotUsages.sort((a, b) => b.count - a.count)[0];

          if (topRobot && topRobot.count > 0) {
            setMostUsedRobot(topRobot.name);
          }
        }

        // Get mission count
        const { count: missionCount } = await supabase
          .from('missions')
          .select('*', { count: 'exact', head: true })
          .eq('user_id', user.id);

        setMissionsTotal(missionCount || 0);

        // Get YOLO detection stats
        const { data: yoloData } = await supabase
          .from('yolo_data')
          .select('confidence_score')
          .eq('user_id', user.id);

        if (yoloData && yoloData.length > 0) {
          setDetectionsCount(yoloData.length);
          const avgConf = yoloData.reduce((sum, d) => sum + (d.confidence_score || 0), 0) / yoloData.length;
          setAvgConfidence(avgConf);
        }

        // Get sound library stats
        const { data: soundData } = await supabase
          .from('sound_clips')
          .select('file_size')
          .eq('user_id', user.id);

        if (soundData) {
          setSoundsCount(soundData.length);
          const totalBytes = soundData.reduce((sum, s) => sum + (s.file_size || 0), 0);

          if (totalBytes < 1024 * 1024) {
            setStorageUsed(`${(totalBytes / 1024).toFixed(1)} KB`);
          } else {
            setStorageUsed(`${(totalBytes / (1024 * 1024)).toFixed(1)} MB`);
          }
        }
      } catch (error) {
        console.error('Error fetching enhanced stats:', error);
      } finally {
        setLoading(false);
      }
    };

    fetchAllStats();
  }, [user, supabase]);

  const onlineRobots = connection.online ? 1 : 0;

  return (
    <div className="space-y-6">
      {/* Primary Stats Grid - 4 columns */}
      <div className="grid grid-cols-2 lg:grid-cols-4 gap-4">
        <StatCard
          icon={<Bot className="h-5 w-5 text-white" />}
          label="Fleet Size"
          value={robots.length}
          subValue={`${onlineRobots} online now`}
          trend={robots.length > 0 ? 'up' : 'neutral'}
          trendValue={onlineRobots > 0 ? 'Active' : 'Offline'}
          gradient="from-violet-500 to-purple-600"
          variant="gradient"
          loading={loading}
        />

        <StatCard
          icon={<Activity className="h-5 w-5 text-white" />}
          label="Total Actions"
          value={totalActions > 9999 ? `${(totalActions / 1000).toFixed(1)}k` : totalActions}
          subValue={`${activeDays} active days`}
          trend={monthlyActions > 500 ? 'up' : 'down'}
          trendValue={`${monthlyActions} this month`}
          gradient="from-blue-500 to-cyan-600"
          variant="glass"
          loading={loading}
        />

        <StatCard
          icon={<Calendar className="h-5 w-5 text-white" />}
          label="User Since"
          value={memberDuration}
          subValue={userProfile ? new Date(userProfile.created_at).toLocaleDateString() : '--'}
          gradient="from-emerald-500 to-teal-600"
          variant="default"
          loading={loading}
        />

        <StatCard
          icon={<Star className="h-5 w-5 text-white" />}
          label="Favorite Robot"
          value={mostUsedRobot}
          subValue={robots.filter(r => r.is_favorite).length > 0 ? 'Most used' : 'No favorite set'}
          gradient="from-yellow-500 to-orange-600"
          variant="neon"
          loading={loading}
        />
      </div>

      {/* Activity Stats - 3 columns */}
      <div className="grid grid-cols-3 gap-4">
        <StatCard
          icon={<Clock className="h-4 w-4 text-white" />}
          label="Today"
          value={todayActions}
          subValue="actions"
          trend={todayActions > 50 ? 'up' : todayActions > 10 ? 'neutral' : 'down'}
          trendValue={todayActions > 50 ? 'Very active' : todayActions > 10 ? 'Active' : 'Quiet'}
          gradient="from-indigo-500 to-blue-600"
          variant="glass"
          size="small"
          loading={loading}
        />

        <StatCard
          icon={<TrendingUp className="h-4 w-4 text-white" />}
          label="This Week"
          value={weeklyActions}
          subValue="actions"
          trend={weeklyActions > 200 ? 'up' : 'neutral'}
          trendValue={`Avg ${Math.round(weeklyActions / 7)}/day`}
          gradient="from-pink-500 to-rose-600"
          variant="glass"
          size="small"
          loading={loading}
        />

        <StatCard
          icon={<Gauge className="h-4 w-4 text-white" />}
          label="Peak Hour"
          value={peakHour}
          subValue="most active"
          gradient="from-purple-500 to-violet-600"
          variant="glass"
          size="small"
          loading={loading}
        />
      </div>

      {/* Advanced Metrics - 4 columns */}
      <div className="grid grid-cols-2 lg:grid-cols-4 gap-4">
        <StatCard
          icon={<Target className="h-4 w-4 text-white" />}
          label="Missions"
          value={missionsTotal}
          subValue="created"
          gradient="from-red-500 to-pink-600"
          variant="default"
          size="small"
          loading={loading}
        />

        <StatCard
          icon={<Brain className="h-4 w-4 text-white" />}
          label="AI Detections"
          value={detectionsCount}
          subValue={avgConfidence > 0 ? `${(avgConfidence * 100).toFixed(0)}% conf` : 'YOLO active'}
          gradient="from-green-500 to-emerald-600"
          variant="default"
          size="small"
          loading={loading}
        />

        <StatCard
          icon={<Music className="h-4 w-4 text-white" />}
          label="Sound Library"
          value={soundsCount}
          subValue="audio clips"
          gradient="from-orange-500 to-red-600"
          variant="default"
          size="small"
          loading={loading}
        />

        <StatCard
          icon={<DatabaseIcon className="h-4 w-4 text-white" />}
          label="Storage"
          value={storageUsed}
          subValue="used"
          gradient="from-gray-600 to-gray-800"
          variant="default"
          size="small"
          loading={loading}
        />
      </div>
    </div>
  );
}