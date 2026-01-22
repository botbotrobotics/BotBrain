'use client';

import { useEffect, useState } from 'react';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { Activity, TrendingUp } from 'lucide-react';

interface HourlyActivity {
  hour: number;
  day: number;
  count: number;
}

export default function ActivityHeatmap() {
  const { supabase, user } = useSupabase();
  const [hourlyData, setHourlyData] = useState<HourlyActivity[]>([]);
  const [loading, setLoading] = useState(true);
  const [maxActivity, setMaxActivity] = useState(0);
  const [totalActions, setTotalActions] = useState(0);
  const [weeklyActions, setWeeklyActions] = useState(0);

  const days = ['S', 'M', 'T', 'W', 'T', 'F', 'S'];
  const hours = Array.from({ length: 24 }, (_, i) => i);

  useEffect(() => {
    const fetchActivityData = async () => {
      if (!user || !supabase) return;

      setLoading(true);

      try {
        // Fetch hourly activity pattern for heatmap
        const { data: activityData, error } = await supabase
          .from('audit_logs')
          .select('created_at')
          .eq('user_id', user.id)
          .gte('created_at', new Date(Date.now() - 30 * 24 * 60 * 60 * 1000).toISOString());

        if (!error && activityData) {
          // Process data for heatmap
          const heatmapData: { [key: string]: number } = {};
          let maxCount = 0;
          let total = 0;

          // Calculate weekly actions
          const oneWeekAgo = new Date(Date.now() - 7 * 24 * 60 * 60 * 1000);
          let weeklyCount = 0;

          activityData.forEach(log => {
            const date = new Date(log.created_at);
            const hour = date.getHours();
            const day = date.getDay();
            const key = `${day}-${hour}`;

            heatmapData[key] = (heatmapData[key] || 0) + 1;
            maxCount = Math.max(maxCount, heatmapData[key]);
            total++;

            if (date >= oneWeekAgo) {
              weeklyCount++;
            }
          });

          setMaxActivity(maxCount);
          setTotalActions(total);
          setWeeklyActions(weeklyCount);

          // Convert to array format
          const processedData: HourlyActivity[] = [];
          for (let day = 0; day < 7; day++) {
            for (let hour = 0; hour < 24; hour++) {
              processedData.push({
                hour,
                day,
                count: heatmapData[`${day}-${hour}`] || 0
              });
            }
          }
          setHourlyData(processedData);
        }
      } catch (error) {
        console.error('Error fetching activity data:', error);
      } finally {
        setLoading(false);
      }
    };

    fetchActivityData();
  }, [user, supabase]);

  const getHeatmapColor = (count: number) => {
    if (count === 0) return 'bg-gray-100 dark:bg-gray-800/50';
    const intensity = Math.min((count / maxActivity) * 100, 100);

    if (intensity < 20) return 'bg-purple-200 dark:bg-purple-900/40';
    if (intensity < 40) return 'bg-purple-300 dark:bg-purple-800/50';
    if (intensity < 60) return 'bg-purple-400 dark:bg-purple-700/60';
    if (intensity < 80) return 'bg-purple-500 dark:bg-purple-600/70';
    return 'bg-purple-600 dark:bg-purple-500';
  };

  if (loading) {
    return (
      <div className="bg-white dark:bg-botbot-dark rounded-2xl p-4 shadow-lg h-full min-h-[280px]">
        <div className="animate-pulse h-full">
          <div className="h-5 bg-gray-200 dark:bg-gray-700 rounded w-1/3 mb-3"></div>
          <div className="h-40 bg-gray-200 dark:bg-gray-700 rounded"></div>
        </div>
      </div>
    );
  }

  const avgDailyActions = Math.round(weeklyActions / 7);

  return (
    <div className="bg-white dark:bg-botbot-dark rounded-2xl p-4 shadow-lg h-full min-h-[240px] flex flex-col">
      {/* Compact Header */}
      <div className="flex items-start justify-between mb-3">
        <div className="flex items-center gap-2.5">
          <div className="p-2 bg-gradient-to-br from-purple-500 to-violet-600 rounded-lg shadow-md">
            <Activity className="h-4 w-4 text-white" />
          </div>
          <div>
            <h3 className="text-sm font-semibold text-gray-900 dark:text-white">
              Activity Pattern
            </h3>
            <p className="text-xs text-gray-500 dark:text-gray-400">
              Last 30 days
            </p>
          </div>
        </div>
        {/* Stats badges */}
        <div className="flex flex-col items-end gap-1">
          <div className="flex items-center gap-1.5 px-2 py-0.5 bg-violet-100 dark:bg-violet-900/30 rounded-full">
            <span className="text-xs font-bold text-violet-700 dark:text-violet-300">
              {totalActions.toLocaleString()}
            </span>
            <span className="text-xs text-violet-600 dark:text-violet-400">total</span>
          </div>
          <div className="flex items-center gap-1 text-xs text-gray-500 dark:text-gray-400">
            <TrendingUp className="h-3 w-3" />
            <span>{avgDailyActions}/day avg</span>
          </div>
        </div>
      </div>

      {/* Heatmap Grid */}
      <div className="flex flex-col">
        <div className="w-full overflow-x-auto">
          {/* Hour labels - show every 3 hours */}
          <div className="grid grid-cols-24 gap-[3px] ml-6 mb-1">
            {hours.map(hour => (
              <div key={hour} className="flex justify-center">
                <span className="text-[9px] text-gray-400 dark:text-gray-500">
                  {hour % 3 === 0 ? (hour === 0 ? '12a' : hour === 12 ? '12p' : hour < 12 ? hour : hour - 12) : ''}
                </span>
              </div>
            ))}
          </div>

          {/* Heatmap rows */}
          <div className="space-y-[3px]">
            {days.map((day, dayIndex) => (
              <div key={dayIndex} className="flex items-center gap-0">
                {/* Day label */}
                <div className="w-6 flex-shrink-0">
                  <span className="text-[10px] font-medium text-gray-500 dark:text-gray-400">
                    {day}
                  </span>
                </div>

                {/* Hour cells for this day */}
                <div className="flex-1 grid grid-cols-24 gap-[3px]">
                  {hours.map(hour => {
                    const activity = hourlyData.find(
                      d => d.day === dayIndex && d.hour === hour
                    );
                    const count = activity?.count || 0;

                    return (
                      <div
                        key={`${dayIndex}-${hour}`}
                        className={`
                          aspect-square rounded-sm transition-all duration-200
                          hover:scale-125 hover:z-10 hover:shadow-lg cursor-pointer
                          ${getHeatmapColor(count)}
                        `}
                        title={`${days[dayIndex]} ${hour}:00 - ${count} actions`}
                      />
                    );
                  })}
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Compact Legend */}
        <div className="flex items-center justify-center gap-2 mt-3 pt-2 border-t border-gray-200 dark:border-gray-700/50">
          <span className="text-[10px] text-gray-500 dark:text-gray-400">Less</span>
          <div className="flex gap-[2px]">
            {[0, 20, 40, 60, 80, 100].map((_, i) => (
              <div
                key={i}
                className={`w-2.5 h-2.5 rounded-sm ${
                  i === 0 ? 'bg-gray-100 dark:bg-gray-800/50' :
                  i === 1 ? 'bg-purple-200 dark:bg-purple-900/40' :
                  i === 2 ? 'bg-purple-300 dark:bg-purple-800/50' :
                  i === 3 ? 'bg-purple-400 dark:bg-purple-700/60' :
                  i === 4 ? 'bg-purple-500 dark:bg-purple-600/70' :
                  'bg-purple-600 dark:bg-purple-500'
                }`}
              />
            ))}
          </div>
          <span className="text-[10px] text-gray-500 dark:text-gray-400">More</span>
        </div>
      </div>
    </div>
  );
}