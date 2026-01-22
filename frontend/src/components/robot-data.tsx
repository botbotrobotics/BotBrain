'use client';

import Container from '@/ui/container';
import RingGauge from './ring-gauge';
import { MenuActionType } from '../types/RobotActionTypes';
import useMenuActions from '@/hooks/useMenuActions';
import useRobotBatteryState from '@/hooks/ros/useRobotBatteryState';
import useRobotSpeed from '@/hooks/ros/useRobotSpeed';
import { useEffect } from 'react';
import { useLanguage } from '@/contexts/LanguageContext';
import { Info } from 'lucide-react';

export default function RobotData() {
  const menuActions = useMenuActions();
  const { t } = useLanguage();

  const actions: MenuActionType[] = [
    // Removed fullscreen and menu buttons
  ];

  const battery = useRobotBatteryState();
  const { speed, maxSpeed } = useRobotSpeed();

  const gaugeContainerClasses =
    'flex-1 min-w-[45%] max-w-full p-3 md:p-4 outline outline-1 outline-gray-200 dark:outline-black rounded-default-border flex flex-col items-center justify-center';

  useEffect(() => {
    // console.log('render robot-data.tsx');
  }, [battery, speed]);

  return (
    <Container
      title={
        <div className="flex items-center">
          <Info className="mr-2 w-5 h-5" />
          <span>{t('robotData', 'title')}</span>
        </div>
      }
      actions={actions}
      className="h-full"
      customContentClasses="w-full h-min flex flex-col justify-between overflow-auto hide-scrollbar pt-1"
    >
      <div className="w-full flex flex-col items-center justify-center">
        <div className="w-full flex flex-row flex-wrap justify-between gap-2 mt-2">
          <div className={gaugeContainerClasses}>
            <RingGauge
              value={battery.percentage * 100}
              unit="%"
              label={t('robotData', 'battery')}
              variant="battery"
            />
          </div>

          <div className={gaugeContainerClasses}>
            <RingGauge
              value={speed}
              maxValue={maxSpeed}
              unit="m/s"
              label={t('robotData', 'speed')}
              variant="speed"
              decimalPlaces={1}
              gradientColors={['#22c55e', '#facc15', '#f97316', '#ef4444']}
            />
          </div>
        </div>
      </div>
    </Container>
  );
}
