'use client';

import { useEffect, useState } from 'react';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { Bot, TrendingUp, Star, Clock, Zap, ChevronDown, ChevronUp } from 'lucide-react';
import { Database } from '@/types/database.types';

type Robot = Database['public']['Tables']['robots']['Row'];

interface RobotUsage {
  robot: Robot;
  usageCount: number;
  lastUsed: string | null;
  percentage: number;
  trend: 'up' | 'down' | 'stable';
}

export default function RobotUsageAnalytics() {
  const { supabase, user } = useSupabase();
  const [robotUsage, setRobotUsage] = useState<RobotUsage[]>([]);
  const [loading, setLoading] = useState(true);
  const [totalUsage, setTotalUsage] = useState(0);
  const [mostActiveTime, setMostActiveTime] = useState<string>('');
  const [isExpanded, setIsExpanded] = useState(false);

  useEffect(() => {
    const fetchRobotUsage = async () => {
      if (!user || !supabase) return;

      setLoading(true);

      try {
        // Fetch robots
        const { data: robots } = await supabase
          .from('robots')
          .select('*')
          .eq('user_id', user.id)
          .order('created_at', { ascending: false });

        if (!robots) return;

        // Fetch usage data for each robot
        const usagePromises = robots.map(async (robot) => {
          // Get total usage count
          const { count: totalCount } = await supabase
            .from('audit_logs')
            .select('*', { count: 'exact', head: true })
            .eq('robot_id', robot.id);

          // Get last 7 days usage for trend
          const sevenDaysAgo = new Date();
          sevenDaysAgo.setDate(sevenDaysAgo.getDate() - 7);

          const { count: recentCount } = await supabase
            .from('audit_logs')
            .select('*', { count: 'exact', head: true })
            .eq('robot_id', robot.id)
            .gte('created_at', sevenDaysAgo.toISOString());

          // Get last 14 days usage for comparison
          const fourteenDaysAgo = new Date();
          fourteenDaysAgo.setDate(fourteenDaysAgo.getDate() - 14);

          const { count: previousCount } = await supabase
            .from('audit_logs')
            .select('*', { count: 'exact', head: true })
            .eq('robot_id', robot.id)
            .lt('created_at', sevenDaysAgo.toISOString())
            .gte('created_at', fourteenDaysAgo.toISOString());

          // Get last used timestamp
          const { data: lastLog } = await supabase
            .from('audit_logs')
            .select('created_at')
            .eq('robot_id', robot.id)
            .order('created_at', { ascending: false })
            .limit(1)
            .single();

          // Determine trend
          let trend: 'up' | 'down' | 'stable' = 'stable';
          if (recentCount && previousCount) {
            if (recentCount > previousCount * 1.2) trend = 'up';
            else if (recentCount < previousCount * 0.8) trend = 'down';
          }

          return {
            robot,
            usageCount: totalCount || 0,
            lastUsed: lastLog?.created_at || null,
            percentage: 0, // Will calculate after we have all counts
            trend
          };
        });

        const usageData = await Promise.all(usagePromises);

        // Calculate total usage and percentages
        const total = usageData.reduce((sum, item) => sum + item.usageCount, 0);
        setTotalUsage(total);

        // Update percentages and sort by usage
        const dataWithPercentages = usageData
          .map(item => ({
            ...item,
            percentage: total > 0 ? (item.usageCount / total) * 100 : 0
          }))
          .sort((a, b) => b.usageCount - a.usageCount);

        setRobotUsage(dataWithPercentages);

        // Fetch most active time across all robots
        const { data: peakHour } = await supabase
          .from('audit_logs')
          .select('created_at')
          .eq('user_id', user.id)
          .not('robot_id', 'is', null);

        if (peakHour && peakHour.length > 0) {
          const hourCounts: { [key: number]: number } = {};

          peakHour.forEach(log => {
            const hour = new Date(log.created_at).getHours();
            hourCounts[hour] = (hourCounts[hour] || 0) + 1;
          });

          const maxHour = Object.entries(hourCounts).reduce((a, b) =>
            b[1] > a[1] ? b : a
          )[0];

          const hour = parseInt(maxHour);
          const period = hour < 12 ? 'AM' : 'PM';
          const displayHour = hour === 0 ? 12 : hour > 12 ? hour - 12 : hour;
          setMostActiveTime(`${displayHour}:00 ${period}`);
        }
      } catch (error) {
        console.error('Error fetching robot usage:', error);
      } finally {
        setLoading(false);
      }
    };

    fetchRobotUsage();
  }, [user, supabase]);

  const formatLastUsed = (timestamp: string | null) => {
    if (!timestamp) return 'Never';

    const date = new Date(timestamp);
    const now = new Date();
    const diffMs = now.getTime() - date.getTime();
    const diffMins = Math.floor(diffMs / 60000);
    const diffHours = Math.floor(diffMs / 3600000);
    const diffDays = Math.floor(diffMs / 86400000);

    if (diffMins < 1) return 'Just now';
    if (diffMins < 60) return `${diffMins}m ago`;
    if (diffHours < 24) return `${diffHours}h ago`;
    if (diffDays < 7) return `${diffDays}d ago`;
    return date.toLocaleDateString();
  };

  const getUsageColor = (percentage: number) => {
    if (percentage > 40) return 'from-purple-500 to-violet-600';
    if (percentage > 20) return 'from-blue-500 to-cyan-600';
    if (percentage > 10) return 'from-green-500 to-emerald-600';
    if (percentage > 0) return 'from-yellow-500 to-orange-600';
    return 'from-gray-400 to-gray-500';
  };

  if (loading) {
    return (
      <div className="bg-white dark:bg-botbot-dark rounded-2xl p-4 shadow-lg h-full min-h-[280px]">
        <div className="animate-pulse h-full">
          <div className="h-5 bg-gray-200 dark:bg-gray-700 rounded w-1/3 mb-3"></div>
          <div className="space-y-2">
            {[1, 2, 3].map(i => (
              <div key={i} className="h-14 bg-gray-200 dark:bg-gray-700 rounded"></div>
            ))}
          </div>
        </div>
      </div>
    );
  }

  if (robotUsage.length === 0) {
    return (
      <div className="bg-white dark:bg-botbot-dark rounded-2xl p-4 shadow-lg h-full min-h-[280px] flex items-center justify-center">
        <div className="text-center">
          <Bot className="h-10 w-10 text-gray-400 mx-auto mb-2" />
          <p className="text-sm text-gray-500 dark:text-gray-400">No robots configured yet</p>
        </div>
      </div>
    );
  }

  const displayedRobots = isExpanded ? robotUsage : robotUsage.slice(0, 3);
  const hasMoreRobots = robotUsage.length > 3;

  return (
    <div className="bg-white dark:bg-botbot-dark rounded-2xl p-4 shadow-lg h-full min-h-[280px] flex flex-col">
      <div className="flex items-start justify-between mb-3">
        <div className="flex items-center gap-2.5">
          <div className="p-2 bg-gradient-to-br from-violet-500 to-purple-600 rounded-lg shadow-md">
            <Bot className="h-4 w-4 text-white" />
          </div>
          <div>
            <h3 className="text-sm font-semibold text-gray-900 dark:text-white">
              Robot Fleet Usage
            </h3>
            <p className="text-xs text-gray-500 dark:text-gray-400">
              {robotUsage.length} robot{robotUsage.length !== 1 ? 's' : ''}
            </p>
          </div>
        </div>
        <div className="flex flex-col items-end gap-1">
          <div className="flex items-center gap-1.5 px-2 py-0.5 bg-violet-100 dark:bg-violet-900/30 rounded-full">
            <span className="text-xs font-bold text-violet-700 dark:text-violet-300">
              {totalUsage.toLocaleString()}
            </span>
            <span className="text-xs text-violet-600 dark:text-violet-400">actions</span>
          </div>
          {mostActiveTime && (
            <div className="flex items-center gap-1 text-xs text-gray-500 dark:text-gray-400">
              <Clock className="h-3 w-3" />
              <span>Peak: {mostActiveTime}</span>
            </div>
          )}
        </div>
      </div>

      <div className="space-y-1.5 flex-1">
        {displayedRobots.map((usage, index) => (
          <div
            key={usage.robot.id}
            className="relative bg-gray-50 dark:bg-botbot-darker rounded-lg p-2.5 hover:shadow-md transition-all duration-200"
          >
            <div className="flex items-center justify-between gap-3">
              <div className="flex items-center gap-2 flex-1 min-w-0">
                {/* Rank badge */}
                <div className={`
                  w-7 h-7 rounded-md flex items-center justify-center text-xs font-bold text-white shadow-sm
                  bg-gradient-to-br ${getUsageColor(usage.percentage)}
                `}>
                  {index + 1}
                </div>

                {/* Robot info */}
                <div className="flex-1 min-w-0">
                  <div className="flex items-center gap-2">
                    <h4 className="font-medium text-sm text-gray-900 dark:text-white truncate">
                      {usage.robot.name}
                    </h4>
                    {usage.robot.is_favorite && (
                      <Star className="h-3 w-3 text-yellow-500 fill-yellow-500 flex-shrink-0" />
                    )}
                    <span className="text-[10px] px-1.5 py-0.5 rounded bg-gray-200 dark:bg-gray-700 text-gray-600 dark:text-gray-400">
                      {usage.robot.type}
                    </span>
                  </div>
                  <div className="flex items-center gap-2 mt-0.5">
                    <p className="text-[11px] text-gray-600 dark:text-gray-400">
                      {usage.usageCount.toLocaleString()} actions
                    </p>
                    <span className="text-[11px] text-gray-400">•</span>
                    <p className="text-[11px] text-gray-500 dark:text-gray-500">
                      {formatLastUsed(usage.lastUsed)}
                    </p>
                    {usage.lastUsed && new Date(usage.lastUsed) > new Date(Date.now() - 3600000) && (
                      <>
                        <span className="text-[11px] text-gray-400">•</span>
                        <div className="flex items-center gap-1">
                          <Zap className="h-2.5 w-2.5 text-green-500" />
                          <span className="text-[11px] text-green-600 dark:text-green-400">Active</span>
                        </div>
                      </>
                    )}
                  </div>
                </div>
              </div>

              {/* Percentage */}
              <div className="text-right flex-shrink-0">
                <p className="text-base font-bold text-gray-900 dark:text-white">
                  {usage.percentage.toFixed(0)}%
                </p>
                {usage.trend !== 'stable' && (
                  <div className={`text-[10px] ${
                    usage.trend === 'up' ? 'text-green-600 dark:text-green-400' : 'text-red-600 dark:text-red-400'
                  }`}>
                    <TrendingUp className={`inline h-2.5 w-2.5 ${usage.trend === 'down' ? 'rotate-180' : ''}`} />
                  </div>
                )}
              </div>
            </div>

            {/* Slim usage bar */}
            <div className="relative h-1 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden mt-2">
              <div
                className={`absolute inset-y-0 left-0 bg-gradient-to-r ${getUsageColor(usage.percentage)} transition-all duration-500`}
                style={{ width: `${Math.max(2, usage.percentage)}%` }}
              />
            </div>
          </div>
        ))}
      </div>

      {hasMoreRobots && (
        <button
          onClick={() => setIsExpanded(!isExpanded)}
          className="mt-2 pt-2 border-t border-gray-200 dark:border-gray-700/50 flex items-center justify-center gap-1.5 text-xs text-gray-600 dark:text-gray-400 hover:text-violet-600 dark:hover:text-violet-400 transition-colors"
        >
          {isExpanded ? (
            <>
              <ChevronUp className="h-3 w-3" />
              <span>Show less</span>
            </>
          ) : (
            <>
              <ChevronDown className="h-3 w-3" />
              <span>Show {robotUsage.length - 3} more</span>
            </>
          )}
        </button>
      )}
    </div>
  );
}