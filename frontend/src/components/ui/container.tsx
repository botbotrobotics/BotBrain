'use client';

import React, { ReactNode, useRef } from 'react';
import SeparatorLine from './separator-line';
import Image from 'next/image';
import { MenuActionType } from '../../types/RobotActionTypes';
import useElementWidth from '@/hooks/useElementWidth';
import { cn } from '@/utils/cn';

export default function Container({
  children,
  title = '',
  actions,
  menuButtonSize = 20,
  className = '',
  customContentClasses = '',
  headerContent = undefined,
  customActions = undefined,
}: Readonly<{
  children?: React.ReactNode;
  title?: string | React.ReactNode;
  actions?: MenuActionType[];
  menuButtonSize?: number;
  className?: string;
  customContentClasses?: string;
  headerContent?: ReactNode;
  customActions?: ReactNode;
}>) {
  const showSeparator =
    (typeof title === 'string' ? title.length > 0 : !!title) || actions || customActions;
  const contentClasses = `${customContentClasses} p-4`;

  const containerRef = useRef<HTMLDivElement>(null);
  const width = useElementWidth(containerRef);
  const actionMenuBtnBreakPointWidth = 300;

  const actionBtnBreakClasses = `${
    width < actionMenuBtnBreakPointWidth ? ' flex-col' : ' flex-row'
  }`;

  return (
    <div
      className={cn(
        'rounded-default-border border-[1px] border-gray-200 dark:border-black relative bg-white dark:bg-botbot-dark w-auto h-fit z-10',
        className
      )}
      ref={containerRef}
      style={{ isolation: 'isolate' }}
    >
      {showSeparator && (
        <div className="w-full inset-y-3">
          <div className="flex flex-row items-center justify-between">
            {!headerContent && (
              <div className="mb-2">
                <h1 className="p-4 pb-0 text-center text-base font-bold">
                  {title}
                </h1>
              </div>
            )}

            {headerContent && <div className="h-full p-0">{headerContent}</div>}

            {(actions || customActions) && (
              <div
                className={cn(`w-auto flex flex-row items-center gap-1 pr-4  +
                ${actionBtnBreakClasses} +
                ${headerContent === undefined ? 'pt-2' : 'pt-0'}`)}
              >
                {customActions && (
                  <div className="mr-1">
                    {customActions}
                  </div>
                )}
                {actions && actions.map((action, index) => (
                  <button
                    key={index}
                    className="border-none bg-none h-fit hover:bg-purple-100 dark:hover:bg-purple-700 focus:bg-purple-200 dark:focus:bg-purple-800 rounded-menu-btn-border transition-colors duration-200"
                    onClick={(_e) => action?.action && action.action()}
                  >
                    <div
                      className="min-h-7 flex flex-row items-center 
                      p-1"
                    >
                      <Image
                        src={`icons/${action?.icon}.svg`}
                        className="min-w-6 self-center"
                        alt="Avatar"
                        width={action?.iconDefaultSize ?? menuButtonSize}
                        height={action?.iconDefaultSize ?? menuButtonSize}
                        style={{
                          objectFit: 'fill',
                          width: '100%',
                          height: 'auto',
                        }}
                      />
                    </div>
                  </button>
                ))}
              </div>
            )}
          </div>

          <div className="min-w-full">
            <SeparatorLine />
          </div>
        </div>
      )}

      <div className={contentClasses}>
        {children}
        {/* Width: {width}px */}
      </div>
    </div>
  );
}
