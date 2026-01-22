'use client';

import React from 'react';
import { cn } from '@/utils/cn';
import clamp from '@/utils/clamp';

interface RingGaugeProps {
  /** Current value to display */
  value: number;
  /** Minimum value (default: 0) */
  minValue?: number;
  /** Maximum value (default: 100) */
  maxValue?: number;
  /** Unit to display after value (e.g., "%", "m/s") */
  unit?: string;
  /** Label shown below the gauge */
  label?: string;
  /** Color variant - determines fill color scheme */
  variant?: 'battery' | 'speed' | 'default';
  /** Custom gradient colors (for speed variant) */
  gradientColors?: string[];
  /** Number of decimal places to show (default: 0) */
  decimalPlaces?: number;
  /** Custom class name for the container */
  className?: string;
}

const RingGauge: React.FC<RingGaugeProps> = ({
  value,
  minValue = 0,
  maxValue = 100,
  unit = '',
  label,
  variant = 'default',
  gradientColors = ['#22c55e', '#facc15', '#f97316', '#ef4444'],
  decimalPlaces = 0,
  className,
}) => {
  // Calculate percentage and clamp
  const clampedValue = clamp(value, minValue, maxValue);
  const percentage = (clampedValue - minValue) / (maxValue - minValue);

  // SVG constants
  const radius = 40;
  const circumference = 2 * Math.PI * radius;
  const strokeOffset = circumference * (1 - percentage);

  // Get battery status color based on percentage
  const getBatteryColor = (): string => {
    if (percentage <= 0.15) return '#ef4444'; // red-500 - critical
    if (percentage <= 0.30) return '#eab308'; // yellow-500 - warning
    return '#22c55e'; // green-500 - good
  };

  // Get speed color based on percentage (interpolate through gradient)
  const getSpeedColor = (): string => {
    if (!gradientColors || gradientColors.length === 0) return '#8A2BE2';
    if (gradientColors.length === 1) return gradientColors[0];

    const segmentCount = gradientColors.length - 1;
    const segmentIndex = Math.min(
      Math.floor(percentage * segmentCount),
      segmentCount - 1
    );
    return gradientColors[segmentIndex + 1] || gradientColors[segmentCount];
  };

  // Determine stroke color based on variant
  const getStrokeColor = (): string => {
    switch (variant) {
      case 'battery':
        return getBatteryColor();
      case 'speed':
        return getSpeedColor();
      default:
        return '#8A2BE2'; // botbot purple
    }
  };

  // Format value for display
  const displayValue =
    decimalPlaces > 0
      ? clampedValue.toFixed(decimalPlaces)
      : Math.round(clampedValue).toString();

  // Generate unique ID for gradient
  const gradientId = `speedGradient-${React.useId()}`;

  return (
    <div
      className={cn(
        'flex flex-col items-center justify-center gap-1',
        className
      )}
    >
      {/* Label above the gauge */}
      {label && (
        <span className="text-sm font-semibold text-gray-700 dark:text-gray-300">
          {label}
        </span>
      )}

      {/* Ring gauge container - responsive square */}
      <div className="relative w-full max-w-[100px] aspect-square">
        <svg viewBox="0 0 100 100" className="w-full h-full">
          {/* Gradient definition for speed variant */}
          {variant === 'speed' && gradientColors && (
            <defs>
              <linearGradient
                id={gradientId}
                x1="0%"
                y1="0%"
                x2="100%"
                y2="0%"
              >
                {gradientColors.map((color, i) => (
                  <stop
                    key={i}
                    offset={`${(i / (gradientColors.length - 1)) * 100}%`}
                    stopColor={color}
                  />
                ))}
              </linearGradient>
            </defs>
          )}

          {/* Background track */}
          <circle
            cx="50"
            cy="50"
            r={radius}
            fill="none"
            strokeWidth="8"
            className="stroke-gray-200 dark:stroke-gray-700"
          />

          {/* Progress ring */}
          <circle
            cx="50"
            cy="50"
            r={radius}
            fill="none"
            strokeWidth="8"
            strokeLinecap="round"
            strokeDasharray={circumference}
            strokeDashoffset={strokeOffset}
            stroke={
              variant === 'speed' ? `url(#${gradientId})` : getStrokeColor()
            }
            className="transition-all duration-500 ease-out origin-center"
            style={{ transform: 'rotate(-90deg)', transformOrigin: '50% 50%' }}
          />
        </svg>

        {/* Center content - value display */}
        <div className="absolute inset-0 flex flex-col items-center justify-center">
          <span
            className={cn(
              'text-xl font-bold',
              variant === 'battery' && percentage <= 0.15
                ? 'text-red-500 dark:text-red-400'
                : variant === 'battery' && percentage <= 0.3
                  ? 'text-yellow-600 dark:text-yellow-400'
                  : 'text-gray-900 dark:text-white'
            )}
          >
            {displayValue}
          </span>
          {unit && (
            <span className="text-xs text-gray-500 dark:text-gray-400 -mt-0.5">
              {unit}
            </span>
          )}
        </div>
      </div>
    </div>
  );
};

export default RingGauge;
