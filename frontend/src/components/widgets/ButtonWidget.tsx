'use client';

import { Widget } from './Widget';
import { useState } from 'react';
import RobotActionButton from '@/components/ui/robot-action-button';
import { useDashboard } from '@/contexts/DashboardContext';
import { RobotActionTypeName } from '@/types/RobotActionTypes';
import { useLanguage } from '@/contexts/LanguageContext';

// Available button actions
const BUTTON_ACTIONS = [
  { label: 'Get Up', value: 'getUp' },
  { label: 'Get Down', value: 'getDown' },
  { label: 'Balance Stand', value: 'balanceStand' },
  { label: 'Joint Lock', value: 'jointLock' },
  { label: 'Pose On', value: 'poseOn' },
  { label: 'Pose Off', value: 'poseOff' },
  { label: 'Hello', value: 'hello' },
  { label: 'Stop On', value: 'stopOn' },
  { label: 'Stop Off', value: 'stopOff' },
  { label: 'Light', value: 'light' },
  { label: 'Anti Collision', value: 'antiCollision' },
  { label: 'Stop Move', value: 'stopMove' },
];

interface ButtonWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  title?: string;
  props?: {
    title?: string;
    buttonAction?: RobotActionTypeName;
  };
}

export function ButtonWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 200, height: 200 },
  title = 'Button',
  props,
}: ButtonWidgetProps) {
  const { updateWidgetProps } = useDashboard();

  // Use props from dashboard context if available, otherwise use defaults
  const [currentTitle, setCurrentTitle] = useState(props?.title || title);
  const [currentButtonAction, setCurrentButtonAction] =
    useState<RobotActionTypeName>(
      (props?.buttonAction as RobotActionTypeName) || 'getUp'
    );
  const [showSettings, setShowSettings] = useState(false);
  const { t } = useLanguage();

  // Update dashboard context when settings change
  const updateSettings = (updates: any) => {
    const newProps = {
      ...props,
      ...updates,
    };
    updateWidgetProps(id, newProps);
    return newProps;
  };

  const handleTitleChange = (newTitle: string) => {
    setCurrentTitle(newTitle);
    updateSettings({ title: newTitle });
  };

  const handleButtonActionChange = (newAction: string) => {
    setCurrentButtonAction(newAction as RobotActionTypeName);
    updateSettings({ buttonAction: newAction });
  };

  return (
    <Widget
      id={id}
      title={currentTitle}
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      initialPosition={initialPosition}
      initialSize={initialSize}
      minWidth={150}
      minHeight={150}
      onSettingsClick={() => setShowSettings(!showSettings)}
    >
      <div className="h-full flex flex-col items-center justify-center p-2">
        {showSettings && (
          <div className="w-full bg-gray-100 dark:bg-botbot-darker rounded-md p-3 mb-4 text-sm">
            <div className="mb-3">
              <label className="block text-gray-700 dark:text-gray-300 mb-1">
                {t('myUI', 'widgetName')}
              </label>
              <input
                type="text"
                value={currentTitle}
                onChange={(e) => handleTitleChange(e.target.value)}
                className="w-full p-2 border rounded-md dark:bg-botbot-dark dark:border-botbot-darker"
                placeholder="Enter widget name"
              />
            </div>

            <div className="mb-3">
              <label className="block text-gray-700 dark:text-gray-300 mb-1">
                {t('myUI', 'buttonAction')}
              </label>
              <div className="relative">
                <select
                  value={currentButtonAction}
                  onChange={(e) => handleButtonActionChange(e.target.value)}
                  className="w-full p-2 border rounded-md dark:bg-botbot-dark dark:border-botbot-darker appearance-none text-sm"
                >
                  {BUTTON_ACTIONS.map((action) => (
                    <option key={action.value} value={action.value}>
                      {action.label}
                    </option>
                  ))}
                </select>
                <div className="pointer-events-none absolute inset-y-0 right-0 flex items-center px-2 text-gray-700 dark:text-gray-300">
                  <svg
                    className="fill-current h-4 w-4"
                    xmlns="http://www.w3.org/2000/svg"
                    viewBox="0 0 20 20"
                  >
                    <path d="M9.293 12.95l.707.707L15.657 8l-1.414-1.414L10 10.828 5.757 6.586 4.343 8z" />
                  </svg>
                </div>
              </div>
            </div>
          </div>
        )}

        <div className="flex-1 w-full flex items-center justify-center">
          <RobotActionButton
            action={currentButtonAction}
            className="w-full h-full max-w-64 max-h-64"
          />
        </div>
      </div>
    </Widget>
  );
}
