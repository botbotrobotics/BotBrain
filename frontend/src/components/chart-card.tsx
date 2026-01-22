'use client';

import { useState, useEffect } from 'react';
import {
  LineChart,
  Line,
  BarChart,
  Bar,
  AreaChart,
  Area,
  ScatterChart,
  Scatter,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  Legend,
  RadarChart,
  Radar,
  PolarGrid,
  PolarAngleAxis,
  PolarRadiusAxis,
} from 'recharts';
import ApexChartWrapper from './apex-chart-wrapper';
import useChartData from '@/hooks/ros/useChartData';
import { Clock, Settings } from 'lucide-react';
import { ApexOptions } from 'apexcharts';
import { topicsMessages } from '@/utils/ros/topics-and-services';
import { isEqual } from 'lodash';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useLanguage } from '@/contexts/LanguageContext';

// Define refresh times
const REFRESH_TIMES = [
  { label: '0.1s', value: 100 },
  { label: '0.5s', value: 500 },
  { label: '1s', value: 1000 },
  { label: '2s', value: 2000 },
  { label: '5s', value: 5000 },
  { label: '10s', value: 10000 },
  { label: '30s', value: 30000 },
  { label: '1m', value: 60000 },
];

// Define available chart types
const CHART_TYPES: { label: string; value: ChartType }[] = [
  { label: 'Line', value: 'line' },
  { label: 'Bar', value: 'bar' },
  { label: 'Area', value: 'area' },
  { label: 'Scatter', value: 'scatter' },
  { label: 'Radar', value: 'radar' },
  { label: 'Gauge', value: 'gauge' },
  { label: 'Radial Bar', value: 'radialBar' },
  { label: 'Pie', value: 'pie' },
  { label: 'Donut', value: 'donut' },
  { label: 'Heatmap', value: 'heatmap' },
];

type ChartType =
  | 'line'
  | 'bar'
  | 'area'
  | 'scatter'
  | 'radar'
  | 'gauge'
  | 'radialBar'
  | 'pie'
  | 'donut'
  | 'heatmap';

// Type for ApexCharts data
interface ApexChartData {
  series: number[] | { name: string; data: any[] }[];
  options: ApexOptions;
}

interface ChartCardProps {
  defaultTopic: keyof typeof topicsMessages;
  defaultChartType: ChartType;
  refreshTime?: number;
  title: string;
  formatLabel?: (data: number) => string;
}

interface TopicKeyValue {
  label: string;
  value: keyof typeof topicsMessages;
}

const ROS_TOPICS: TopicKeyValue[] = [
  // Basic robot state
  { label: 'Battery State', value: 'battery' },
  { label: 'Temperature', value: 'temperature' },
  { label: 'Joint States', value: 'jointStates' },
  // { label: 'CPU Usage', value: '/system/cpu_usage' },
  { label: 'Robot Status', value: 'robotStatus' },
  // { label: 'Environment Temperature', value: 'temperature' },
  { label: 'Sport Mode States', value: 'sportModeState' },

  // Sensors
  { label: 'Odometry', value: 'odometry' },
  { label: 'Laser Scan', value: 'laserScan' },
  { label: 'Camera Front', value: 'camera' },
  // { label: 'Camera Rear', value: 'camera' },
  // { label: 'Camera Left', value: 'camera' },
  // { label: 'Camera Right', value: 'camera' },

  // Motion data
  { label: 'Velocity', value: 'velocity' },
  { label: 'NippleJs Velocity', value: 'velocityNipple' },

  // Additional sensors (based on common Unitree topics)
  // { label: 'IMU Data', value: '/imu/data' },
  // { label: 'IMU Mag', value: '/imu/mag' },
  // { label: 'Robot State', value: '/robot_state' },
  // { label: 'Joint Command', value: '/joint_command' },
  // { label: 'Foot Contact', value: '/foot_contact' },
  // { label: 'Force Torque', value: '/force_torque' },
];

export default function ChartCard({
  defaultTopic,
  defaultChartType,
  refreshTime = 5000, // Default to 5s if not provided
  title,
  formatLabel,
}: ChartCardProps) {
  const [topic, setTopic] = useState<keyof typeof topicsMessages>(defaultTopic);
  const [chartType, setChartType] = useState(defaultChartType);
  const [localRefreshTime, setLocalRefreshTime] = useState(refreshTime);
  const [showControls, setShowControls] = useState(false);
  // Dados vindos de useChartData
  const data = useChartData(topic, localRefreshTime);
  // Cria o estado que guarda os últimos 5 valores que realmente foram renderizados
  const [cachedLastFive, setCachedLastFive] = useState<any[]>(data.slice(-5));
  const { connection } = useRobotConnection();
  const { t } = useLanguage();

  // Use um useEffect para atualizar cachedLastFive apenas se os últimos 5 valores mudaram
  useEffect(() => {
    const newLastFive = data.slice(-5);
    if (!isEqual(cachedLastFive, newLastFive)) {
      setCachedLastFive(newLastFive);
    }
    // Note que como dependências temos data e cachedLastFive
  }, [data, cachedLastFive]);

  // Format timestamp for display
  const formatTimestamp = (timestamp: number) => {
    const date = new Date(timestamp);
    return `${date.getHours().toString().padStart(2, '0')}:${date
      .getMinutes()
      .toString()
      .padStart(2, '0')}:${date.getSeconds().toString().padStart(2, '0')}`;
  };

  // Find the current topic label
  const getCurrentTopicLabel = () => {
    const topicItem = ROS_TOPICS.find((t) => t.value === topic);
    return topicItem ? topicItem.label : topic;
  };

  // Generate chart colors based on the chart type
  const getChartColor = () => {
    switch (chartType) {
      case 'line':
        return '#7C3AED'; // Violet
      case 'bar':
        return '#3B82F6'; // Blue
      case 'area':
        return '#10B981'; // Green
      case 'scatter':
        return '#F59E0B'; // Amber
      case 'radar':
        return '#EC4899'; // Pink
      case 'gauge':
      case 'radialBar':
        return '#8B5CF6'; // Purple
      case 'pie':
      case 'donut':
        return ['#7C3AED', '#3B82F6', '#10B981', '#F59E0B', '#EC4899'];
      case 'heatmap':
        return ['#10B981', '#F59E0B', '#EC4899', '#EF4444'];
      default:
        return '#7C3AED';
    }
  };

  // Format data for different chart types
  const formatDataForApex = (): ApexChartData => {
    // For ApexCharts
    if (['gauge', 'radialBar', 'pie', 'donut', 'heatmap'].includes(chartType)) {
      // Get the latest value
      const latestValue = data.length > 0 ? data[data.length - 1].value : 0;

      // Format for different chart types
      if (chartType === 'gauge' || chartType === 'radialBar') {
        return {
          series: [Math.round(Number(latestValue))],
          options: {
            chart: {
              type: chartType as any, // Type assertion to satisfy TypeScript
              height: 350,
              fontFamily: 'inherit',
              toolbar: {
                show: false,
              },
              animations: {
                enabled: true,
                speed: 300,
              },
              background: 'transparent',
            },
            plotOptions: {
              radialBar: {
                startAngle: -90,
                endAngle: 90,
                hollow: {
                  margin: 0,
                  size: '70%',
                },
                track: {
                  background: '#e7e7e7',
                  strokeWidth: '97%',
                  margin: 5,
                  dropShadow: {
                    enabled: false,
                  },
                },
                dataLabels: {
                  name: {
                    show: true,
                    color: '#888',
                    fontSize: '12px',
                    offsetY: -5,
                  },
                  value: {
                    offsetY: -2,
                    fontSize: '18px',
                    formatter: function (val: number) {
                      return val.toString();
                    },
                  },
                },
              },
            },
            fill: {
              type: 'gradient',
              gradient: {
                shade: 'dark',
                type: 'horizontal',
                shadeIntensity: 0.5,
                gradientToColors: Array.isArray(getChartColor())
                  ? undefined
                  : [getChartColor() as string],
                inverseColors: false,
                opacityFrom: 1,
                opacityTo: 1,
                stops: [0, 100],
              },
            },
            stroke: {
              lineCap: 'round',
            },
            labels: [getCurrentTopicLabel()],
            colors: Array.isArray(getChartColor())
              ? (getChartColor() as string[])
              : [getChartColor() as string],
            theme: {
              mode: 'light',
            },
          },
        };
      } else if (chartType === 'pie' || chartType === 'donut') {
        // For pie/donut charts, create multiple data points from the values history
        // Use the last 5 values to make it more interesting
        // Aqui usamos os últimos 5 valores do cachedLastFive
        const lastValues = cachedLastFive.map((d) =>
          Math.round(Number(d.value))
        );
        const labels = lastValues.map(
          (val, i) => formatLabel?.(val) ?? `Value ${i + 1}`
        );

        return {
          series: lastValues.length > 0 ? lastValues : [0],
          options: {
            chart: {
              type: chartType as any, // Type assertion
              height: 350,
              fontFamily: 'inherit',
              toolbar: {
                show: false,
              },
              animations: {
                enabled: true,
                speed: 300,
              },
              background: 'transparent',
            },
            labels: labels.length > 0 ? labels : ['No Data'],
            colors: getChartColor() as string[],
            legend: {
              position: 'bottom',
            },
            dataLabels: {
              enabled: false,
            },
            responsive: [
              {
                breakpoint: 480,
                options: {
                  chart: {
                    width: 200,
                  },
                  legend: {
                    position: 'bottom',
                  },
                },
              },
            ],
            theme: {
              mode: 'light',
            },
          },
        };
      } else if (chartType === 'heatmap') {
        // Create a heatmap from the data
        const series = Array.from({ length: 5 }, (_, i) => {
          return {
            name: `Series ${i + 1}`,
            data: data.slice(-10).map((d, _j) => {
              // Create a variation of the value for each series
              return {
                x: formatTimestamp(d.timestamp),
                y: Math.round(Number(d.value) * (0.5 + i * 0.2)),
              };
            }),
          };
        });

        return {
          series:
            series.length > 0
              ? series
              : [{ name: 'No Data', data: [{ x: 'No Data', y: 0 }] }],
          options: {
            chart: {
              type: 'heatmap',
              height: 350,
              fontFamily: 'inherit',
              toolbar: {
                show: false,
              },
              animations: {
                enabled: true,
                speed: 300,
              },
              background: 'transparent',
            },
            dataLabels: {
              enabled: false,
            },
            colors: getChartColor() as string[],
            title: {
              text: getCurrentTopicLabel(),
              align: 'center',
              style: {
                fontSize: '14px',
              },
            },
            theme: {
              mode: 'light',
            },
          },
        };
      }
    }

    // Default return for non-Apex charts
    return {
      series: [],
      options: {},
    };
  };

  // Render the appropriate chart based on the selected chart type
  const renderChart = () => {
    // If not connected to robot and no data, show a message
    if (!connection.online && data.length === 0) {
      return (
        <div className="flex flex-col items-center justify-center h-full w-full text-gray-500 dark:text-gray-400 text-center p-4">
          <div className="mb-2">
            <div className="h-3 w-3 rounded-full bg-red-500 mx-auto mb-2" />
            <span className="text-sm font-medium">{t('robotOffline', 'robotDisconnected')}</span>
          </div>
          <p className="text-xs">{t('robotOffline', 'connectToRobotMessage')}</p>
        </div>
      );
    }

    // If connected but no data yet, show loading
    if (connection.online && data.length === 0) {
      return (
        <div className="flex items-center justify-center h-full">
          <span className="text-sm text-gray-500 dark:text-gray-400">Waiting for data...</span>
        </div>
      );
    }

    const chartData = data.map((point) => ({
      ...point,
      formattedTime: formatTimestamp(point.timestamp),
    }));

    const chartColor = getChartColor();

    // Recharts components
    switch (chartType) {
      case 'line':
        return (
          <LineChart data={chartData}>
            <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
            <XAxis
              dataKey="formattedTime"
              tick={{ fontSize: 10, fill: '#666' }}
              tickLine={{ stroke: '#ccc' }}
            />
            <YAxis
              tick={{ fontSize: 10, fill: '#666' }}
              tickLine={{ stroke: '#ccc' }}
            />
            <Tooltip
              contentStyle={{
                backgroundColor: 'rgba(255, 255, 255, 0.9)',
                border: '1px solid #ccc',
                borderRadius: '4px',
                boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
              }}
            />
            <Legend />
            <Line
              name={getCurrentTopicLabel()}
              type="monotone"
              dataKey="value"
              stroke={chartColor as string}
              activeDot={{ r: 6, strokeWidth: 2 }}
              strokeWidth={2}
              dot={{ r: 3 }}
              animationDuration={300}
            />
          </LineChart>
        );
      case 'bar':
        return (
          <BarChart data={chartData}>
            <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
            <XAxis
              dataKey="formattedTime"
              tick={{ fontSize: 10, fill: '#666' }}
              tickLine={{ stroke: '#ccc' }}
            />
            <YAxis
              tick={{ fontSize: 10, fill: '#666' }}
              tickLine={{ stroke: '#ccc' }}
            />
            <Tooltip
              contentStyle={{
                backgroundColor: 'rgba(255, 255, 255, 0.9)',
                border: '1px solid #ccc',
                borderRadius: '4px',
                boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
              }}
            />
            <Legend />
            <Bar
              name={getCurrentTopicLabel()}
              dataKey="value"
              fill={chartColor as string}
              radius={[4, 4, 0, 0]}
              animationDuration={300}
            />
          </BarChart>
        );
      case 'area':
        return (
          <AreaChart data={chartData}>
            <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
            <XAxis
              dataKey="formattedTime"
              tick={{ fontSize: 10, fill: '#666' }}
              tickLine={{ stroke: '#ccc' }}
            />
            <YAxis
              tick={{ fontSize: 10, fill: '#666' }}
              tickLine={{ stroke: '#ccc' }}
            />
            <Tooltip
              contentStyle={{
                backgroundColor: 'rgba(255, 255, 255, 0.9)',
                border: '1px solid #ccc',
                borderRadius: '4px',
                boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
              }}
            />
            <Legend />
            <Area
              name={getCurrentTopicLabel()}
              type="monotone"
              dataKey="value"
              stroke={chartColor as string}
              fill={chartColor as string}
              fillOpacity={0.3}
              activeDot={{ r: 6 }}
              animationDuration={300}
            />
          </AreaChart>
        );
      case 'scatter':
        return (
          <ScatterChart>
            <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
            <XAxis
              type="number"
              dataKey="timestamp"
              domain={['dataMin', 'dataMax']}
              name="Time"
              tickFormatter={formatTimestamp}
              tick={{ fontSize: 10, fill: '#666' }}
              tickLine={{ stroke: '#ccc' }}
            />
            <YAxis
              type="number"
              dataKey="value"
              name="Value"
              tick={{ fontSize: 10, fill: '#666' }}
              tickLine={{ stroke: '#ccc' }}
            />
            <Tooltip
              cursor={{ strokeDasharray: '3 3' }}
              formatter={(value) => [value, 'Value']}
              labelFormatter={formatTimestamp}
              contentStyle={{
                backgroundColor: 'rgba(255, 255, 255, 0.9)',
                border: '1px solid #ccc',
                borderRadius: '4px',
                boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
              }}
            />
            <Legend />
            <Scatter
              name={getCurrentTopicLabel()}
              data={chartData}
              fill={chartColor as string}
              shape="circle"
              animationDuration={300}
            />
          </ScatterChart>
        );
      case 'radar':
        return (
          <RadarChart
            cx="50%"
            cy="50%"
            outerRadius="80%"
            data={chartData.slice(-10)}
          >
            <PolarGrid stroke="#e0e0e0" />
            <PolarAngleAxis
              dataKey="formattedTime"
              tick={{ fontSize: 10, fill: '#666' }}
            />
            <PolarRadiusAxis tick={{ fontSize: 10, fill: '#666' }} />
            <Radar
              name={getCurrentTopicLabel()}
              dataKey="value"
              stroke={chartColor as string}
              fill={chartColor as string}
              fillOpacity={0.6}
              animationDuration={300}
            />
            <Legend />
            <Tooltip
              contentStyle={{
                backgroundColor: 'rgba(255, 255, 255, 0.9)',
                border: '1px solid #ccc',
                borderRadius: '4px',
                boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
              }}
            />
          </RadarChart>
        );
      case 'gauge':
      case 'radialBar':
      case 'pie':
      case 'donut':
      case 'heatmap':
        const { series, options } = formatDataForApex();
        return (
          <ApexChartWrapper
            options={options}
            series={series}
            type={
              chartType === 'donut'
                ? 'donut'
                : chartType === 'gauge'
                ? 'radialBar'
                : (chartType as any)
            }
            height={320}
          />
        );
      default:
        return <div>Unsupported chart type</div>;
    }
  };

  return (
    <div
      className="bg-white dark:bg-botbot-dark rounded-lg shadow-md p-2 flex flex-col transition-all duration-300 hover:shadow-lg border border-gray-100 dark:border-botbot-darker overflow-hidden relative z-10"
      style={{ isolation: 'isolate' }}
    >
      <div className="flex items-center justify-between mb-2 px-2">
        <h2 className="text-base font-medium text-gray-800 dark:text-botbot-accent">
          {title}
        </h2>
        <div className="flex items-center gap-2">
          <div className="flex items-center gap-1 text-xs text-gray-500 dark:text-botbot-accent/70">
            <Clock className="w-3 h-3" />
            <span>
              {REFRESH_TIMES.find((t) => t.value === localRefreshTime)?.label ||
                '5s'}
            </span>
          </div>
          <button
            onClick={() => setShowControls(!showControls)}
            className="p-1 hover:bg-gray-100 dark:hover:bg-botbot-dark rounded-md transition-colors"
            title="Chart settings"
          >
            <Settings className="w-4 h-4 text-violet-600 dark:text-violet-400" />
          </button>
        </div>
      </div>

      {showControls && (
        <div className="flex flex-col px-2 pb-2 gap-2 animate-fadeIn bg-gray-50 dark:bg-botbot-dark rounded-md m-1 p-2 text-xs border border-gray-100 dark:border-botbot-darker">
          <div className="grid grid-cols-2 gap-2">
            <div className="flex flex-col">
              <label className="text-xs text-gray-500 dark:text-botbot-accent mb-1">
                Data Source
              </label>
              <select
                className="bg-white dark:bg-botbot-darker border border-violet-200 dark:border-botbot-dark rounded-md px-2 py-1 text-xs w-full focus:outline-none focus:ring-2 focus:ring-violet-500 dark:text-white"
                value={topic}
                onChange={(e) =>
                  setTopic(e.target.value as keyof typeof topicsMessages)
                }
              >
                {ROS_TOPICS.map((t) => (
                  <option key={t.value} value={t.value}>
                    {t.label}
                  </option>
                ))}
              </select>
            </div>
            <div className="flex flex-col">
              <label className="text-xs text-gray-500 dark:text-botbot-accent mb-1">
                Chart Type
              </label>
              <select
                className="bg-white dark:bg-botbot-darker border border-violet-200 dark:border-botbot-dark rounded-md px-2 py-1 text-xs w-full focus:outline-none focus:ring-2 focus:ring-violet-500 dark:text-white"
                value={chartType}
                onChange={(e) => setChartType(e.target.value as ChartType)}
              >
                {CHART_TYPES.map((type) => (
                  <option key={type.value} value={type.value}>
                    {type.label}
                  </option>
                ))}
              </select>
            </div>
          </div>

          <div className="flex flex-col w-full">
            <label className="text-xs text-gray-500 dark:text-botbot-accent mb-1">
              Refresh Rate
            </label>
            <select
              className="bg-white dark:bg-botbot-darker border border-violet-200 dark:border-botbot-dark rounded-md px-2 py-1 text-xs w-full focus:outline-none focus:ring-2 focus:ring-violet-500 dark:text-white"
              value={localRefreshTime}
              onChange={(e) => setLocalRefreshTime(Number(e.target.value))}
            >
              {REFRESH_TIMES.map((time) => (
                <option key={time.value} value={time.value}>
                  {time.label}
                </option>
              ))}
            </select>
          </div>
        </div>
      )}

      <div className="h-64 w-full flex-grow px-2 pb-2">
        <ResponsiveContainer width="100%" height="100%">
          {renderChart()}
        </ResponsiveContainer>
      </div>
    </div>
  );
}
