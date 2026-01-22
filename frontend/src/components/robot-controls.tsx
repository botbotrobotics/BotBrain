'use client';

import Container from '@/ui/container';
import RobotActionButton from '@/ui/robot-action-button';
import { useEffect, useRef, useState, useMemo } from 'react';
import { Gamepad } from 'lucide-react';
import { useLanguage } from '@/contexts/LanguageContext';
import useRobotStatus from '@/hooks/ros/useRobotStatus';
import useRobotActionsTransitions from '@/hooks/ros/useRobotActionsTransitions';
import { useRobotCustomModeContext } from '@/contexts/RobotCustomModesContext';
import { RobotActionTypeName } from '@/types/RobotActionTypes';
import { useRobotProfile } from '@/contexts/RobotProfileContext';

export default function RobotControls() {
  const containerRef = useRef<HTMLDivElement>(null);
  const [_isWrapped, setIsWrapped] = useState(false);
  const [_itemsPerRow, setItemsPerRow] = useState(0);
  const { t } = useLanguage();
  const { light, statusBeforeEmergency, antiCollision } =
    useRobotCustomModeContext();
  const { robotStatus } = useRobotStatus();
  const robotActions = useRobotActionsTransitions(robotStatus);
  const robotActionsBeforeEmergency = useRobotActionsTransitions(
    statusBeforeEmergency ?? robotStatus
  );
  const [isInEmergencyMode, setEmergencyMode] = useState(false);
  const { currentProfile } = useRobotProfile();

  const btnCustomClasses =
    'hover:dark:bg-botbot-dark/80 dark:bg-botbot-dark focus:dark:bg-botbot-darker';

  const emergencyOnBtnClasses = `!aspect-auto font-bold py-3 xl:py-4 bg-red-500
    text-white text-sm xl:text-base 2xl:text-lg
    hover:bg-red-600
    focus:bg-red-700 mb-2 rounded-default-border`;

  const emergencyOffBtnClasses = `!aspect-auto font-bold py-3 xl:py-4 bg-green-500
    text-white text-sm xl:text-base 2xl:text-lg
    hover:bg-green-600
    focus:bg-green-700 mb-2 rounded-default-border`;

  useEffect(() => {
    const checkWrapAndCount = () => {
      if (!containerRef.current) return;

      const container = containerRef.current;
      const children = Array.from(container.children) as HTMLElement[];

      if (children.length === 0) return;

      let lastOffsetTop = children[0].offsetTop;
      let itemsInRow = 0;
      let maxItemsInRow = 0;

      children.forEach((child) => {
        if (child.offsetTop !== lastOffsetTop) {
          // New line detected
          maxItemsInRow = Math.max(maxItemsInRow, itemsInRow);
          itemsInRow = 1; // First item in new line
          lastOffsetTop = child.offsetTop;
        } else {
          itemsInRow++; // Count elements in same line
        }
      });

      maxItemsInRow = Math.max(maxItemsInRow, itemsInRow); // Check last line
      setItemsPerRow(maxItemsInRow);

      // If container height is greater than height of a single item, there was a line break
      const hasWrapped = container.clientHeight > children[0].clientHeight;
      setIsWrapped(hasWrapped);
    };

    const observer = new ResizeObserver(checkWrapAndCount);
    if (containerRef.current) {
      observer.observe(containerRef.current);
    }

    checkWrapAndCount(); // Run on first render

    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    setEmergencyMode(robotStatus === 'emergency');
  }, [robotStatus]);

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

  return (
    <Container
      title={
        <div className="flex items-center">
          <Gamepad className="mr-2 w-5 h-5" />
          <span>{t('robotControls', 'title')}</span>
        </div>
      }
      className="w-full h-full flex flex-col"
      customContentClasses="p-0 h-full flex flex-col overflow-hidden"
    >
      <div className="h-full flex flex-col overflow-y-auto overflow-x-hidden px-2 py-3 scrollbar-thin scrollbar-thumb-gray-300 scrollbar-track-transparent">
        {/* Emergency button first */}
        <div className="w-full">
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
        
        {/* Button grid container */}
        <div 
          ref={containerRef}
          className="w-full grid grid-cols-2 xl:grid-cols-1 gap-2 pb-2"
        >
          {/* Other action buttons */}
          {filteredActions.map((action, index) => (
            <RobotActionButton
              key={index}
              className={btnCustomClasses}
              action={getToggleActionButtons(action)}
            />
          ))}
        </div>
      </div>
    </Container>
  );
}
