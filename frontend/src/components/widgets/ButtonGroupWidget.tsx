'use client';

import { Widget } from './Widget';
import { useState, useMemo } from 'react';
import RobotActionButton from '@/components/ui/robot-action-button';
import { useDashboard } from '@/contexts/DashboardContext';
import { RobotActionTypeName } from '@/types/RobotActionTypes';
import { useRobotCustomModeContext } from '@/contexts/RobotCustomModesContext';
import useRobotActionsTransitions from '@/hooks/ros/useRobotActionsTransitions';
import useRobotStatus from '@/hooks/ros/useRobotStatus';
import { useLanguage } from '@/contexts/LanguageContext';
import { useRobotProfile } from '@/contexts/RobotProfileContext';

interface ButtonGroupWidgetProps {
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

export function ButtonGroupWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 200, height: 200 },
  title = 'Button',
  props,
}: ButtonGroupWidgetProps) {
  const { updateWidgetProps } = useDashboard();

  // Use props from dashboard context if available, otherwise use defaults
  const [currentTitle, setCurrentTitle] = useState(props?.title || title);
  const [currentButtonAction, setCurrentButtonAction] =
    useState<RobotActionTypeName>(
      (props?.buttonAction as RobotActionTypeName) || 'getUp'
    );
  const [showSettings, setShowSettings] = useState(false);

  const [isInEmergencyMode, setEmergencyMode] = useState(false);
  const { light, statusBeforeEmergency, antiCollision } =
    useRobotCustomModeContext();
  const { robotStatus } = useRobotStatus();
  const robotActions = useRobotActionsTransitions(robotStatus);
  const robotActionsBeforeEmergency = useRobotActionsTransitions(
    statusBeforeEmergency ?? robotStatus
  );
  const { t } = useLanguage();
  const { currentProfile } = useRobotProfile();

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

  const getToggleActionButtons = (
    action: RobotActionTypeName
  ): RobotActionTypeName => {
    if (action.includes('light')) return light ? 'lightOff' : 'lightOn';
    else if (action.includes('Collision'))
      return antiCollision ? 'antiCollisionOff' : 'antiCollisionOn';
    return action;
  };

  // Filter out hidden actions based on robot profile
  const filteredActions = useMemo(() => {
    const actions = !isInEmergencyMode ? robotActions : robotActionsBeforeEmergency;
    if (!currentProfile?.hiddenActions || currentProfile.hiddenActions.length === 0) {
      return actions;
    }
    return actions.filter(action => !currentProfile.hiddenActions?.includes(action));
  }, [robotActions, robotActionsBeforeEmergency, isInEmergencyMode, currentProfile]);

  const btnCustomClasses =
    'hover:dark:bg-botbot-dark/80 dark:bg-botbot-dark focus:dark:bg-botbot-darker';

  const emergencyOnBtnClasses = `!aspect-auto font-bold py-3 bg-red-500
    text-white text-sm
    hover:bg-red-600
    focus:bg-red-700`;

  const emergencyOffBtnClasses = `!aspect-auto font-bold py-3 bg-green-500
    text-white text-sm
    hover:bg-green-600
    focus:bg-green-700`;

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
      <div className="h-full flex flex-col items-center justify-center p-2 relative">
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
          </div>
        )}

        <div className="flex flex-col gap-2 overflow-auto flex-1 w-full">
          {/* Emergency button at top */}
          <div className="w-full px-2">
            {isInEmergencyMode ? (
              <RobotActionButton
                className={emergencyOffBtnClasses}
                action="emergencyOff"
              />
            ) : (
              <RobotActionButton
                className={emergencyOnBtnClasses}
                action="emergencyOn"
              />
            )}
          </div>

          {/* Button grid with uniform sizes */}
          <div className="grid grid-cols-1 gap-2 px-2 pb-2 justify-items-center">
            {filteredActions.map((action, index) => {
              return (
                <div key={index} className="w-full max-w-[120px]">
                  <RobotActionButton
                    className={btnCustomClasses}
                    action={getToggleActionButtons(action)}
                  />
                </div>
              );
            })}
          </div>
        </div>
      </div>
    </Widget>
  );
}
