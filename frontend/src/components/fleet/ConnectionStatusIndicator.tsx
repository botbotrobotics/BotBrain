'use client';

import React from 'react';
import { RobotStatus } from '@/hooks/useRobotAvailability';

interface ConnectionStatusIndicatorProps {
  status: RobotStatus;
  lastChecked?: Date | null;
  size?: 'sm' | 'md' | 'lg';
  showTooltip?: boolean;
}

export default function ConnectionStatusIndicator({
  status,
  lastChecked,
  size = 'sm',
  showTooltip = true
}: ConnectionStatusIndicatorProps) {
  const sizeClasses = {
    sm: 'w-2 h-2',
    md: 'w-3 h-3',
    lg: 'w-4 h-4'
  };

  const statusClasses = {
    available: 'bg-green-500',
    unavailable: 'bg-red-500',
    checking: 'bg-gray-400'
  };

  const getTooltipText = () => {
    switch (status) {
      case 'available':
        return 'Robot is online';
      case 'unavailable':
        return 'Robot is offline';
      case 'checking':
        return 'Checking connection...';
    }
  };

  return (
    <div className="relative inline-flex items-center justify-center group">
      <div
        className={`
          ${sizeClasses[size]}
          ${statusClasses[status]}
          rounded-full
          ${status === 'checking' ? 'animate-pulse' : ''}
          transition-colors duration-300
        `}
      />

      {status === 'available' && (
        <div
          className={`
            absolute
            ${sizeClasses[size]}
            ${statusClasses[status]}
            rounded-full
            animate-ping
          `}
        />
      )}

      {showTooltip && (
        <div className="absolute bottom-full left-1/2 transform -translate-x-1/2 mb-2 px-2 py-1
                      bg-gray-800 dark:bg-gray-900 text-white text-xs rounded
                      opacity-0 group-hover:opacity-100 transition-opacity pointer-events-none
                      whitespace-nowrap z-10">
          {getTooltipText()}
          <div className="absolute top-full left-1/2 transform -translate-x-1/2 -mt-1">
            <div className="w-0 h-0 border-l-4 border-l-transparent border-r-4
                         border-r-transparent border-t-4 border-t-gray-800 dark:border-t-gray-900" />
          </div>
        </div>
      )}
    </div>
  );
}