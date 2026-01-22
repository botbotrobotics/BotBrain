'use client';

import { cn } from '@/utils/cn';
import useRobotActions from '@/hooks/ros/useRobotActions';
import Image from 'next/image';
import React, { useRef, useState } from 'react';
import { RobotActionType, RobotActionTypeName } from '@/types/RobotActionTypes';
import useRobotStatus from '@/hooks/ros/useRobotStatus';
import { auditLogger } from '@/utils/audit-logger';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useConfirmationDialog } from '@/contexts/ConfirmationDialogContext';

export default function RobotActionButton({
  action,
  className = '',
}: {
  action: RobotActionTypeName;
  className?: string;
}) {
  const [isClicked, setIsClicked] = useState(false);
  const [showFlash, setShowFlash] = useState(false);
  const [showSuccessFeedback, setShowSuccessFeedback] = useState(false);
  const [showErrorFeedback, setShowErrorFeedback] = useState(false);
  const robotActions = useRobotActions();
  const btnRef = useRef<HTMLButtonElement>(null);
  const { robotStatus } = useRobotStatus();
  const { connection } = useRobotConnection();
  const { requestConfirmation, requiresConfirmation } = useConfirmationDialog();

  const isEmergencyAction = action.includes('emergency');
  const feedbackAnimationDuration = 750;

  // Execute the action (extracted from handleClick for reuse with confirmation)
  const executeAction = (btn: RobotActionType) => {
    setIsClicked(true);
    setTimeout(() => setIsClicked(false), 400);

    // Show flash animation when stop button is clicked
    if (action === 'emergencyOn') {
      setShowFlash(true);
      setTimeout(() => setShowFlash(false), 1500);
    }

    // Log button press
    const robotId = connection.connectedRobot?.id;
    const robotName = connection.connectedRobot?.name;

    if (action === 'emergencyOn') {
      auditLogger.logEmergencyStop(robotId, robotName);
    } else {
      auditLogger.logButtonPress(action, robotId, robotName, {
        button_label: btn.label || action,
        robot_status: robotStatus,
      });
    }

    btn.action &&
      btn.action({
        callback: () => {
          console.log(`Ação ${action} executada com sucesso.`);
          setShowSuccessFeedback(true);
          setTimeout(() => {
            setShowSuccessFeedback(false);
          }, feedbackAnimationDuration);
        },
        failedCallback: (error?: string) => {
          console.error(error);
          setShowErrorFeedback(true);

          if (action === 'emergencyOn') {
            setShowFlash(false);
          }

          setTimeout(() => {
            setShowErrorFeedback(false);
          }, feedbackAnimationDuration);
        },
      });
  };

  const handleClick = (btn: RobotActionType) => {
    // Check if action requires confirmation
    if (requiresConfirmation(action)) {
      requestConfirmation({
        actionName: action,
        actionLabel: btn.label || action,
        onConfirm: () => {
          executeAction(btn);
        },
      });
      return;
    }

    executeAction(btn);
  };

  const btn = robotActions[action];
  const defaultHoverClass = isEmergencyAction ? '' : 'hover:bg-secondary';
  const defaultRingHoverClass = isEmergencyAction
    ? 'ring-0'
    : 'hover:ring-1 hover:ring-primary focus:ring-1 focus:ring-primary dark:ring-black disabled:ring-0';

  return (
    <>
      {showFlash && (
        <div className="fixed inset-0 pointer-events-none z-50 bg-red-600 animate-redFlash" />
      )}

      <div className="relative">
        {/* {showSuccessFeedback && (
          <CircleCheck
            color="#108a00"
            className="absolute w-10 h-10 right-[90%] mt-8 z-50 animate-slide-up-fade-out"
          />
        )}

        {showErrorFeedback && (
          <CircleX
            color="#cd0404"
            className="absolute w-10 h-10 right-[90%] mt-8 z-50 animate-slide-up-fade-out"
          />
        )} */}

        <button
          ref={btnRef}
          disabled={isEmergencyAction ? false : robotStatus === 'emergency'}
          className={cn(
            `w-full aspect-square bg-pink-lighter text-black rounded-default-border p-2 xl:p-3 2xl:p-4 flex flex-col items-center justify-center
              hover:scale-105 transition-transform focus:bg-action-btn-focus text-xs xl:text-sm
              ${
                isClicked ? ' animate-shrinkBounce' : ''
              } focus:outline-none disabled:opacity-50 disabled:cursor-not-allowed disabled:hover:scale-100`,
            className,
            defaultRingHoverClass,
            defaultHoverClass
          )}
          onClick={() => {
            handleClick(btn);
          }}
        >
          {showSuccessFeedback && (
            <div className="absolute rounded-default-border inset-0 pointer-events-none z-50 bg-green-500 animate-successActionFeedback" />
          )}

          {showErrorFeedback && (
            <div className="absolute rounded-default-border inset-0 pointer-events-none z-50 bg-red-500 animate-failActionFeedback" />
          )}
          <div className="relative w-6 h-6 xl:w-8 xl:h-8 2xl:w-9 2xl:h-9 flex-shrink-0">
            {btn?.icon && (
              <Image
                src={`/robot-controls/${btn.icon}`}
                alt={btn.name}
                fill
                className="object-contain"
              />
            )}
          </div>
          <label className="mt-1 text-center leading-tight line-clamp-2">
            {btn?.label}
          </label>
        </button>
      </div>
    </>
  );
}
