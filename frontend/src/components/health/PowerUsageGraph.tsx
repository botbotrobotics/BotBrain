'use client';

import { useState, useEffect, useRef, useMemo, useCallback } from 'react';
import { AreaChart, Area, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';
import { TrendingUp, TrendingDown, Minus } from 'lucide-react';

interface PowerDataPoint {
  timestamp: number;
  current: number;
  time: string;
}

interface PowerUsageGraphProps {
  currentPower: number; // in milliwatts
  averagePower: number; // in milliwatts
  t: any; // translation function
}

const HISTORY_DURATION = 60; // 60 seconds of history
const MAX_POINTS = 60; // Store one point per second
const UPDATE_THRESHOLD = 50; // Only update if change is > 50mW

export default function PowerUsageGraph({ currentPower, averagePower, t }: PowerUsageGraphProps) {
  const [dataHistory, setDataHistory] = useState<PowerDataPoint[]>([]);
  const previousPowerRef = useRef<number>(currentPower);
  const lastUpdateRef = useRef<number>(0);
  const [trend, setTrend] = useState<'up' | 'down' | 'stable'>('stable');
  const animationFrameRef = useRef<number | null>(null);

  // Convert milliwatts to watts for display
  const currentWatts = currentPower / 1000;
  const averageWatts = averagePower / 1000;

  // Smooth data addition with requestAnimationFrame
  const addDataPoint = useCallback((watts: number, timestamp: number) => {
    setDataHistory(prev => {
      const newPoint: PowerDataPoint = {
        timestamp,
        current: watts,
        time: new Date(timestamp).toLocaleTimeString('en-US', {
          hour12: false,
          second: '2-digit'
        }).slice(-2)
      };

      // Keep only last HISTORY_DURATION seconds of data
      const cutoffTime = timestamp - (HISTORY_DURATION * 1000);
      let filtered = prev.filter(p => p.timestamp > cutoffTime);

      // Interpolate if there's a gap
      if (filtered.length > 0) {
        const lastPoint = filtered[filtered.length - 1];
        const timeDiff = timestamp - lastPoint.timestamp;

        // If gap is larger than 2 seconds, add interpolated points
        if (timeDiff > 2000) {
          const steps = Math.floor(timeDiff / 1000);
          const powerStep = (watts - lastPoint.current) / steps;

          for (let i = 1; i < steps; i++) {
            const interpTime = lastPoint.timestamp + (i * 1000);
            filtered.push({
              timestamp: interpTime,
              current: lastPoint.current + (powerStep * i),
              time: new Date(interpTime).toLocaleTimeString('en-US', {
                hour12: false,
                second: '2-digit'
              }).slice(-2)
            });
          }
        }
      }

      // Limit to MAX_POINTS
      if (filtered.length >= MAX_POINTS) {
        filtered = filtered.slice(filtered.length - MAX_POINTS + 1);
      }

      return [...filtered, newPoint];
    });
  }, []);

  // Update data with throttling
  useEffect(() => {
    const now = Date.now();

    // Throttle updates to prevent too frequent changes
    if (now - lastUpdateRef.current < 250) {
      return; // Skip this update if less than 250ms since last update
    }

    // Only update if change is significant
    const diff = Math.abs(currentPower - previousPowerRef.current);
    if (diff < UPDATE_THRESHOLD && dataHistory.length > 0) {
      return; // Skip minor changes
    }

    lastUpdateRef.current = now;

    // Cancel any pending animation frame
    if (animationFrameRef.current) {
      cancelAnimationFrame(animationFrameRef.current);
    }

    // Use animation frame for smooth update
    animationFrameRef.current = requestAnimationFrame(() => {
      addDataPoint(currentWatts, now);

      // Update trend indicator with smoothing
      const trendDiff = currentPower - previousPowerRef.current;
      if (Math.abs(trendDiff) < 100) { // Less than 100mW change is considered stable
        setTrend('stable');
      } else if (trendDiff > 0) {
        setTrend('up');
      } else {
        setTrend('down');
      }

      previousPowerRef.current = currentPower;
    });

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [currentPower, currentWatts, addDataPoint, dataHistory.length]);

  // Calculate min/max for better Y-axis scaling
  const { yMin, yMax } = useMemo(() => {
    if (dataHistory.length === 0) return { yMin: 0, yMax: 30 };

    const allValues = dataHistory.map(d => d.current);
    const min = Math.min(...allValues);
    const max = Math.max(...allValues);
    const padding = (max - min) * 0.2 || 5; // Add 20% padding or 5W if range is 0

    return {
      yMin: Math.max(0, Math.floor((min - padding) * 10) / 10),
      yMax: Math.ceil((max + padding) * 10) / 10
    };
  }, [dataHistory]);

  // Get trend icon
  const TrendIcon = () => {
    switch (trend) {
      case 'up':
        return <TrendingUp className="w-3 h-3 text-red-500" />;
      case 'down':
        return <TrendingDown className="w-3 h-3 text-green-500" />;
      default:
        return <Minus className="w-3 h-3 text-gray-500" />;
    }
  };

  // Generate smooth chart data
  const chartData = useMemo(() => {
    if (dataHistory.length === 0) {
      // Empty state - show axis with no data
      return Array.from({ length: 13 }, (_, i) => ({
        time: `${60 - i * 5}`,
        current: null
      }));
    }

    const now = Date.now();
    const points: any[] = [];

    // Create a continuous line of data points
    for (let i = 0; i <= 60; i++) {
      const targetTime = now - ((60 - i) * 1000);

      // Find closest data point
      let closestPoint = null;
      let minDiff = Infinity;

      for (const point of dataHistory) {
        const diff = Math.abs(point.timestamp - targetTime);
        if (diff < minDiff) {
          minDiff = diff;
          closestPoint = point;
        }
      }

      // Only show data if we have a point within 2 seconds
      if (closestPoint && minDiff < 2000) {
        points.push({
          time: i.toString(),
          current: closestPoint.current,
          displayTime: `${60 - i}s`
        });
      } else if (points.length > 0 && i < 60) {
        // Use last known value for continuity
        const lastValidPoint = points[points.length - 1];
        points.push({
          time: i.toString(),
          current: lastValidPoint.current,
          displayTime: `${60 - i}s`,
          interpolated: true
        });
      }
    }

    return points;
  }, [dataHistory]);

  // Custom tooltip
  const CustomTooltip = ({ active, payload, label }: any) => {
    if (active && payload && payload.length) {
      const data = payload[0].payload;
      return (
        <div className="bg-white dark:bg-gray-800 p-2 rounded-lg shadow-lg border border-gray-200 dark:border-gray-700">
          <p className="text-xs font-medium text-gray-600 dark:text-gray-400 mb-1">
            {data.displayTime || `${label}s`} ago
          </p>
          {payload.map((entry: any, index: number) => (
            <p key={index} className="text-xs" style={{ color: entry.color }}>
              {entry.name}: {entry.value.toFixed(2)}W
              {data.interpolated && <span className="text-gray-500"> (interpolated)</span>}
            </p>
          ))}
        </div>
      );
    }
    return null;
  };

  return (
    <div className="w-full space-y-3">
      {/* Current values display */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <span className="text-2xl font-bold text-gray-800 dark:text-white">
            {currentWatts.toFixed(1)}
            <span className="text-sm font-normal text-gray-600 dark:text-gray-400 ml-1">W</span>
          </span>
          <TrendIcon />
        </div>
        <div className="text-right">
          <p className="text-xs text-gray-500 dark:text-gray-400">
            {t('health', 'avg')}: <span className="font-medium text-gray-700 dark:text-gray-300">{averageWatts.toFixed(1)}W</span>
          </p>
        </div>
      </div>

      {/* Graph */}
      <div className="h-32 w-full">
        <ResponsiveContainer width="100%" height="100%">
          <AreaChart
            data={chartData}
            margin={{ top: 5, right: 5, bottom: 20, left: 5 }}
          >
            <defs>
              <linearGradient id="currentGradient" x1="0" y1="0" x2="0" y2="1">
                <stop offset="5%" stopColor="#7C3AED" stopOpacity={0.3}/>
                <stop offset="95%" stopColor="#7C3AED" stopOpacity={0.05}/>
              </linearGradient>
            </defs>

            <CartesianGrid
              strokeDasharray="3 3"
              stroke="rgba(156, 163, 175, 0.2)"
              vertical={false}
            />

            <XAxis
              dataKey="time"
              tick={{ fontSize: 10, fill: '#9CA3AF' }}
              tickLine={false}
              axisLine={{ stroke: 'rgba(156, 163, 175, 0.2)' }}
              interval={10}
              label={{
                value: 'Seconds ago',
                position: 'insideBottom',
                offset: -15,
                style: { fontSize: 10, fill: '#9CA3AF' }
              }}
              tickFormatter={(value) => {
                const num = parseInt(value);
                return num % 10 === 0 ? `${60 - num}` : '';
              }}
            />

            <YAxis
              domain={[yMin, yMax]}
              tick={{ fontSize: 10, fill: '#9CA3AF' }}
              tickLine={false}
              axisLine={{ stroke: 'rgba(156, 163, 175, 0.2)' }}
              tickFormatter={(value) => `${value.toFixed(1)}`}
              width={35}
            />

            <Tooltip content={<CustomTooltip />} />

            {/* Current power line */}
            <Area
              name={t('health', 'current')}
              type="linear"
              dataKey="current"
              stroke="#7C3AED"
              strokeWidth={2}
              fill="url(#currentGradient)"
              connectNulls={false}
              dot={false}
              isAnimationActive={false}
            />
          </AreaChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}