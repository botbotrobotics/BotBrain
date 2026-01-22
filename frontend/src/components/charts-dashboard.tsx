'use client';

import ChartCard from '@/components/chart-card';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import Container from './ui/container';
import { modeStates } from '@/utils/ros/modeStates';
import { useLanguage } from '@/contexts/LanguageContext';
import { NestedTranslationKey } from '@/utils/translations';

export default function ChartsDashboard() {
  const { connection } = useRobotConnection();
  const { t } = useLanguage();

  const headerContent = (
    <div className="flex items-center justify-between w-full p-4">
      <h1 className="text-xl font-bold text-gray-800 dark:text-gray-100">
        Robot Telemetry Dashboard
      </h1>
      <div className="flex items-center gap-3">
        <div className="flex items-center px-3 py-1 rounded-full border border-gray-200 dark:border-gray-600">
          <div
            className={`h-3 w-3 rounded-full mr-2 ${
              connection.online ? 'bg-green-500 animate-pulse' : 'bg-red-500'
            }`}
          />
          <span className="text-sm text-gray-700 dark:text-gray-300">
            {connection.online ? 'Connected' : 'Disconnected'}
          </span>
        </div>
      </div>
    </div>
  );

  return (
    <div className="p-6 w-full h-full overflow-auto relative z-0">
      <Container
        headerContent={headerContent}
        className="shadow-md dark:shadow-gray-900 relative z-10"
      >
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6 relative z-10">
          <ChartCard
            defaultTopic="battery"
            defaultChartType="gauge"
            refreshTime={5000}
            title="Battery Status"
          />
          <ChartCard
            defaultTopic="temperature"
            defaultChartType="area"
            refreshTime={5000}
            title="Motor Temperature"
          />
          <ChartCard
            defaultTopic="jointStates"
            defaultChartType="radar"
            refreshTime={2000}
            title="Joint Positions"
          />
          <ChartCard
            defaultTopic="velocityNipple"
            defaultChartType="line"
            refreshTime={2000}
            title="Velocity"
          />
          <ChartCard
            defaultTopic="sportModeState"
            defaultChartType="donut"
            refreshTime={1000}
            title="Sport Mode States"
            formatLabel={(data) =>
              t(
                'robotModes',
                modeStates[
                  data
                ]?.toString() as NestedTranslationKey<'robotModes'>
              )
            }
          />
          <ChartCard
            defaultTopic="sportModeState"
            defaultChartType="pie"
            refreshTime={1000}
            title="Sport Mode States"
            formatLabel={(data) =>
              t(
                'robotModes',
                modeStates[
                  data
                ]?.toString() as NestedTranslationKey<'robotModes'>
              )
            }
          />
        </div>
      </Container>
    </div>
  );
}
