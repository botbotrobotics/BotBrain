'use client';

import { useEffect, useState } from 'react';
import {
  Activity,
  Clock,
  TrendingUp,
  Calendar,
  Command,
  Zap,
  Target,
  Award,
  ChevronRight,
  BarChart2,
  Users,
  Globe,
} from 'lucide-react';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { useRouter } from 'next/navigation';

interface ActivityItem {
  id: string;
  action: string;
  timestamp: string;
  icon: React.ReactNode;
  color: string;
}

interface AchievementBadge {
  id: string;
  title: string;
  description: string;
  icon: React.ReactNode;
  unlocked: boolean;
  progress: number;
  total: number;
}

export default function UserActivitySection() {
  const router = useRouter();
  const { user, supabase } = useSupabase();
  const [recentActivity, setRecentActivity] = useState<ActivityItem[]>([]);
  const [achievements, setAchievements] = useState<AchievementBadge[]>([]);
  const [weeklyProgress, setWeeklyProgress] = useState<number[]>([0, 0, 0, 0, 0, 0, 0]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchUserActivity = async () => {
      if (!user || !supabase) return;

      try {
        // Fetch recent activity logs
        const { data: logs } = await supabase
          .from('audit_logs')
          .select('*')
          .eq('user_id', user.id)
          .order('created_at', { ascending: false })
          .limit(5);

        if (logs) {
          const activities = logs.map((log) => ({
            id: log.id,
            action: log.action || 'Action performed',
            timestamp: new Date(log.created_at).toLocaleTimeString('en-US', {
              hour: '2-digit',
              minute: '2-digit',
            }),
            icon: getActivityIcon(log.action),
            color: getActivityColor(log.action),
          }));
          setRecentActivity(activities);
        }

        // Generate weekly progress data (mock for demo)
        const progress = Array.from({ length: 7 }, () => Math.floor(Math.random() * 100) + 20);
        setWeeklyProgress(progress);

        // Set achievements (mock data)
        setAchievements([
          {
            id: '1',
            title: 'First Connection',
            description: 'Connect your first robot',
            icon: <Zap className="h-4 w-4" />,
            unlocked: true,
            progress: 1,
            total: 1,
          },
          {
            id: '2',
            title: 'Fleet Commander',
            description: 'Manage 5+ robots',
            icon: <Users className="h-4 w-4" />,
            unlocked: false,
            progress: 3,
            total: 5,
          },
          {
            id: '3',
            title: 'Power User',
            description: '1000+ actions performed',
            icon: <Award className="h-4 w-4" />,
            unlocked: false,
            progress: 750,
            total: 1000,
          },
          {
            id: '4',
            title: 'Global Operator',
            description: 'Control robots remotely',
            icon: <Globe className="h-4 w-4" />,
            unlocked: true,
            progress: 1,
            total: 1,
          },
        ]);
      } catch (error) {
        console.error('Error fetching user activity:', error);
      } finally {
        setLoading(false);
      }
    };

    fetchUserActivity();
  }, [user, supabase]);

  const getActivityIcon = (action?: string): React.ReactNode => {
    if (!action) return <Activity className="h-3 w-3" />;
    if (action.includes('connect')) return <Zap className="h-3 w-3" />;
    if (action.includes('command')) return <Command className="h-3 w-3" />;
    if (action.includes('update')) return <TrendingUp className="h-3 w-3" />;
    return <Activity className="h-3 w-3" />;
  };

  const getActivityColor = (action?: string): string => {
    if (!action) return 'from-gray-400 to-gray-500';
    if (action.includes('connect')) return 'from-green-400 to-emerald-500';
    if (action.includes('command')) return 'from-blue-400 to-cyan-500';
    if (action.includes('update')) return 'from-purple-400 to-pink-500';
    return 'from-gray-400 to-gray-500';
  };

  const days = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'];
  const maxProgress = Math.max(...weeklyProgress);

  return (
    <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
      {/* Recent Activity Timeline */}
      <div className="lg:col-span-1 bg-white dark:bg-botbot-dark rounded-2xl p-6 shadow-lg border border-gray-100 dark:border-botbot-darker">
        <div className="flex items-center justify-between mb-5">
          <div>
            <h3 className="text-base font-bold text-gray-900 dark:text-white">Recent Activity</h3>
            <p className="text-xs text-gray-600 dark:text-gray-400 mt-1">Your latest actions</p>
          </div>
          <button
            onClick={() => router.push('/activity')}
            className="text-xs font-medium text-violet-600 dark:text-violet-400 hover:text-violet-700 dark:hover:text-violet-300 flex items-center gap-1 group"
          >
            View all
            <ChevronRight className="h-3 w-3 transform group-hover:translate-x-0.5 transition-transform" />
          </button>
        </div>

        {loading ? (
          <div className="space-y-3">
            {[1, 2, 3, 4, 5].map((i) => (
              <div key={i} className="flex items-center gap-3 animate-pulse">
                <div className="h-8 w-8 bg-gray-200 dark:bg-gray-700 rounded-lg" />
                <div className="flex-1">
                  <div className="h-3 bg-gray-200 dark:bg-gray-700 rounded w-3/4 mb-1" />
                  <div className="h-2 bg-gray-200 dark:bg-gray-700 rounded w-1/2" />
                </div>
              </div>
            ))}
          </div>
        ) : recentActivity.length > 0 ? (
          <div className="relative">
            {/* Timeline line */}
            <div className="absolute left-4 top-0 bottom-0 w-0.5 bg-gray-200 dark:bg-gray-700" />

            <div className="space-y-4">
              {recentActivity.map((activity, index) => (
                <div key={activity.id} className="flex items-start gap-3 relative">
                  {/* Timeline dot */}
                  <div className={`relative z-10 rounded-lg p-2 bg-gradient-to-br ${activity.color} shadow-lg`}>
                    <div className="text-white">{activity.icon}</div>
                  </div>

                  <div className="flex-1 min-w-0">
                    <p className="text-sm font-medium text-gray-900 dark:text-white truncate">
                      {activity.action}
                    </p>
                    <p className="text-xs text-gray-500 dark:text-gray-400 flex items-center gap-1 mt-0.5">
                      <Clock className="h-3 w-3" />
                      {activity.timestamp}
                    </p>
                  </div>
                </div>
              ))}
            </div>
          </div>
        ) : (
          <div className="text-center py-8">
            <Activity className="h-12 w-12 text-gray-300 dark:text-gray-700 mx-auto mb-3" />
            <p className="text-sm text-gray-500 dark:text-gray-400">No recent activity</p>
          </div>
        )}
      </div>

      {/* Weekly Progress Chart */}
      <div className="lg:col-span-1 bg-gradient-to-br from-indigo-50 to-purple-50 dark:from-indigo-900/20 dark:to-purple-900/20 rounded-2xl p-6 shadow-lg border border-indigo-100 dark:border-indigo-900/50">
        <div className="flex items-center justify-between mb-5">
          <div>
            <h3 className="text-base font-bold text-gray-900 dark:text-white">Weekly Progress</h3>
            <p className="text-xs text-gray-600 dark:text-gray-400 mt-1">Activity over the week</p>
          </div>
          <BarChart2 className="h-5 w-5 text-indigo-600 dark:text-indigo-400" />
        </div>

        <div className="flex items-end justify-between gap-2 h-32">
          {weeklyProgress.map((value, index) => (
            <div key={index} className="flex-1 flex flex-col items-center gap-2">
              <div className="relative w-full flex-1 flex items-end">
                <div
                  className="w-full bg-gradient-to-t from-indigo-500 to-purple-500 rounded-t-lg transition-all duration-500 hover:from-indigo-600 hover:to-purple-600 cursor-pointer group"
                  style={{
                    height: `${(value / maxProgress) * 100}%`,
                    minHeight: '4px',
                  }}
                >
                  <div className="absolute -top-6 left-1/2 -translate-x-1/2 bg-gray-900 dark:bg-white text-white dark:text-gray-900 text-xs px-1.5 py-0.5 rounded opacity-0 group-hover:opacity-100 transition-opacity whitespace-nowrap">
                    {value}
                  </div>
                </div>
              </div>
              <span className="text-[10px] text-gray-600 dark:text-gray-400 font-medium">
                {days[index]}
              </span>
            </div>
          ))}
        </div>

        <div className="mt-4 flex items-center justify-between text-xs">
          <div className="flex items-center gap-2">
            <div className="h-2 w-2 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500" />
            <span className="text-gray-600 dark:text-gray-400">Actions performed</span>
          </div>
          <span className="font-semibold text-gray-900 dark:text-white">
            {weeklyProgress.reduce((a, b) => a + b, 0)} total
          </span>
        </div>
      </div>

      {/* Achievements & Badges */}
      <div className="lg:col-span-1 bg-white dark:bg-botbot-dark rounded-2xl p-6 shadow-lg border border-gray-100 dark:border-botbot-darker">
        <div className="flex items-center justify-between mb-5">
          <div>
            <h3 className="text-base font-bold text-gray-900 dark:text-white">Achievements</h3>
            <p className="text-xs text-gray-600 dark:text-gray-400 mt-1">Your milestones</p>
          </div>
          <Target className="h-5 w-5 text-violet-600 dark:text-violet-400" />
        </div>

        <div className="space-y-3">
          {achievements.map((achievement) => (
            <div
              key={achievement.id}
              className={`relative p-3 rounded-xl border transition-all duration-300 ${
                achievement.unlocked
                  ? 'bg-gradient-to-r from-violet-50 to-purple-50 dark:from-violet-900/20 dark:to-purple-900/20 border-violet-200 dark:border-violet-800/50'
                  : 'bg-gray-50 dark:bg-gray-800/50 border-gray-200 dark:border-gray-700'
              }`}
            >
              <div className="flex items-start gap-3">
                <div
                  className={`rounded-lg p-2 ${
                    achievement.unlocked
                      ? 'bg-gradient-to-br from-violet-500 to-purple-600 text-white'
                      : 'bg-gray-200 dark:bg-gray-700 text-gray-400 dark:text-gray-500'
                  } shadow`}
                >
                  {achievement.icon}
                </div>
                <div className="flex-1 min-w-0">
                  <h4
                    className={`text-sm font-semibold ${
                      achievement.unlocked
                        ? 'text-gray-900 dark:text-white'
                        : 'text-gray-500 dark:text-gray-400'
                    }`}
                  >
                    {achievement.title}
                    {achievement.unlocked && (
                      <span className="ml-2 inline-flex items-center px-1.5 py-0.5 rounded-full text-[10px] font-bold bg-green-100 dark:bg-green-900/30 text-green-700 dark:text-green-400">
                        UNLOCKED
                      </span>
                    )}
                  </h4>
                  <p className="text-xs text-gray-600 dark:text-gray-400 mt-0.5">
                    {achievement.description}
                  </p>

                  {/* Progress bar */}
                  {!achievement.unlocked && (
                    <div className="mt-2">
                      <div className="flex items-center justify-between mb-1">
                        <span className="text-[10px] text-gray-500 dark:text-gray-400">
                          Progress
                        </span>
                        <span className="text-[10px] font-semibold text-gray-700 dark:text-gray-300">
                          {achievement.progress}/{achievement.total}
                        </span>
                      </div>
                      <div className="h-1.5 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden">
                        <div
                          className="h-full bg-gradient-to-r from-violet-500 to-purple-600 rounded-full transition-all duration-500"
                          style={{
                            width: `${(achievement.progress / achievement.total) * 100}%`,
                          }}
                        />
                      </div>
                    </div>
                  )}
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}