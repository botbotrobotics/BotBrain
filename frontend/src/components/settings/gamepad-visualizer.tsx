'use client';

import React from 'react';
import { cn } from '@/utils/cn';

interface GamepadVisualizerProps {
  pressedButtons: Set<number>;
  mappedButtons: Record<number, string>;
  onButtonClick?: (buttonIndex: number) => void;
  selectedButton?: number | null;
}

export default function GamepadVisualizer({
  pressedButtons,
  mappedButtons,
  onButtonClick,
  selectedButton,
}: GamepadVisualizerProps) {
  const isButtonPressed = (index: number) => pressedButtons.has(index);
  const isButtonSelected = (index: number) => selectedButton === index;

  const getButtonClass = (index: number) => {
    if (isButtonPressed(index)) {
      return 'fill-green-500 stroke-green-600';
    }
    if (isButtonSelected(index)) {
      return 'fill-blue-500 stroke-blue-600 animate-pulse';
    }
    if (mappedButtons[index]) {
      return 'fill-purple-500 stroke-purple-600 hover:fill-purple-400';
    }
    return 'fill-gray-600 dark:fill-gray-400 stroke-gray-700 dark:stroke-gray-300 hover:fill-gray-500 dark:hover:fill-gray-300';
  };

  const getStickClass = (pressed: boolean) => {
    return pressed
      ? 'fill-green-500 stroke-green-600'
      : 'fill-gray-700 dark:fill-gray-500 stroke-gray-800 dark:stroke-gray-400';
  };

  return (
    <div className="w-full max-w-2xl mx-auto p-4">
      <svg
        viewBox="0 0 600 400"
        className="w-full h-auto"
        style={{ maxHeight: '400px' }}
      >
        {/* Controller Body */}
        <path
          d="M 150 200
             Q 150 150, 200 150
             L 250 150
             Q 300 150, 300 100
             Q 300 150, 350 150
             L 400 150
             Q 450 150, 450 200
             L 450 250
             Q 450 300, 400 300
             L 350 300
             Q 300 300, 300 350
             Q 300 300, 250 300
             L 200 300
             Q 150 300, 150 250
             Z"
          className="fill-gray-800 dark:fill-gray-700 stroke-gray-900 dark:stroke-gray-600"
          strokeWidth="2"
        />

        {/* Left Analog Stick */}
        <g>
          <circle
            cx="200"
            cy="200"
            r="30"
            className="fill-gray-900 dark:fill-gray-800 stroke-gray-700 dark:stroke-gray-600"
            strokeWidth="2"
          />
          <circle
            cx="200"
            cy="200"
            r="20"
            className={getStickClass(false)}
            strokeWidth="2"
          />
        </g>

        {/* Right Analog Stick */}
        <g>
          <circle
            cx="350"
            cy="250"
            r="30"
            className="fill-gray-900 dark:fill-gray-800 stroke-gray-700 dark:stroke-gray-600"
            strokeWidth="2"
          />
          <circle
            cx="350"
            cy="250"
            r="20"
            className={getStickClass(false)}
            strokeWidth="2"
          />
        </g>

        {/* D-Pad */}
        <g>
          {/* Up */}
          <rect
            x="190"
            y="240"
            width="20"
            height="30"
            rx="2"
            className={cn('cursor-pointer transition-colors', getButtonClass(12))}
            onClick={() => onButtonClick?.(12)}
          />
          {/* Down */}
          <rect
            x="190"
            y="280"
            width="20"
            height="30"
            rx="2"
            className={cn('cursor-pointer transition-colors', getButtonClass(13))}
            onClick={() => onButtonClick?.(13)}
          />
          {/* Left */}
          <rect
            x="160"
            y="270"
            width="30"
            height="20"
            rx="2"
            className={cn('cursor-pointer transition-colors', getButtonClass(14))}
            onClick={() => onButtonClick?.(14)}
          />
          {/* Right */}
          <rect
            x="210"
            y="270"
            width="30"
            height="20"
            rx="2"
            className={cn('cursor-pointer transition-colors', getButtonClass(15))}
            onClick={() => onButtonClick?.(15)}
          />
        </g>

        {/* Action Buttons (A, B, X, Y) */}
        <g>
          {/* A (bottom) - Button 0 */}
          <circle
            cx="400"
            cy="220"
            r="15"
            className={cn('cursor-pointer transition-colors', getButtonClass(0))}
            onClick={() => onButtonClick?.(0)}
            strokeWidth="2"
          />
          <text x="400" y="225" textAnchor="middle" className="fill-white text-xs font-bold pointer-events-none">
            A
          </text>

          {/* B (right) - Button 1 */}
          <circle
            cx="430"
            cy="190"
            r="15"
            className={cn('cursor-pointer transition-colors', getButtonClass(1))}
            onClick={() => onButtonClick?.(1)}
            strokeWidth="2"
          />
          <text x="430" y="195" textAnchor="middle" className="fill-white text-xs font-bold pointer-events-none">
            B
          </text>

          {/* X (left) - Button 2 */}
          <circle
            cx="370"
            cy="190"
            r="15"
            className={cn('cursor-pointer transition-colors', getButtonClass(2))}
            onClick={() => onButtonClick?.(2)}
            strokeWidth="2"
          />
          <text x="370" y="195" textAnchor="middle" className="fill-white text-xs font-bold pointer-events-none">
            X
          </text>

          {/* Y (top) - Button 3 */}
          <circle
            cx="400"
            cy="160"
            r="15"
            className={cn('cursor-pointer transition-colors', getButtonClass(3))}
            onClick={() => onButtonClick?.(3)}
            strokeWidth="2"
          />
          <text x="400" y="165" textAnchor="middle" className="fill-white text-xs font-bold pointer-events-none">
            Y
          </text>
        </g>

        {/* Shoulder Buttons */}
        <g>
          {/* LB - Button 4 */}
          <rect
            x="170"
            y="120"
            width="60"
            height="20"
            rx="4"
            className={cn('cursor-pointer transition-colors', getButtonClass(4))}
            onClick={() => onButtonClick?.(4)}
            strokeWidth="2"
          />
          <text x="200" y="133" textAnchor="middle" className="fill-white text-xs font-bold pointer-events-none">
            LB
          </text>

          {/* RB - Button 5 */}
          <rect
            x="370"
            y="120"
            width="60"
            height="20"
            rx="4"
            className={cn('cursor-pointer transition-colors', getButtonClass(5))}
            onClick={() => onButtonClick?.(5)}
            strokeWidth="2"
          />
          <text x="400" y="133" textAnchor="middle" className="fill-white text-xs font-bold pointer-events-none">
            RB
          </text>
        </g>

        {/* Triggers */}
        <g>
          {/* LT - Button 6 */}
          <rect
            x="180"
            y="100"
            width="40"
            height="15"
            rx="4"
            className={cn('cursor-pointer transition-colors', getButtonClass(6))}
            onClick={() => onButtonClick?.(6)}
            strokeWidth="2"
          />
          <text x="200" y="110" textAnchor="middle" className="fill-white text-[10px] font-bold pointer-events-none">
            LT
          </text>

          {/* RT - Button 7 */}
          <rect
            x="380"
            y="100"
            width="40"
            height="15"
            rx="4"
            className={cn('cursor-pointer transition-colors', getButtonClass(7))}
            onClick={() => onButtonClick?.(7)}
            strokeWidth="2"
          />
          <text x="400" y="110" textAnchor="middle" className="fill-white text-[10px] font-bold pointer-events-none">
            RT
          </text>
        </g>

        {/* Center Buttons */}
        <g>
          {/* Select/Back - Button 8 */}
          <rect
            x="255"
            y="180"
            width="30"
            height="15"
            rx="4"
            className={cn('cursor-pointer transition-colors', getButtonClass(8))}
            onClick={() => onButtonClick?.(8)}
            strokeWidth="2"
          />
          <text x="270" y="190" textAnchor="middle" className="fill-white text-[8px] font-bold pointer-events-none">
            BACK
          </text>

          {/* Start - Button 9 */}
          <rect
            x="315"
            y="180"
            width="30"
            height="15"
            rx="4"
            className={cn('cursor-pointer transition-colors', getButtonClass(9))}
            onClick={() => onButtonClick?.(9)}
            strokeWidth="2"
          />
          <text x="330" y="190" textAnchor="middle" className="fill-white text-[8px] font-bold pointer-events-none">
            START
          </text>
        </g>

        {/* Stick Buttons */}
        <g>
          {/* L3 - Button 10 */}
          <circle
            cx="200"
            cy="200"
            r="12"
            className={cn('cursor-pointer transition-colors opacity-50', getButtonClass(10))}
            onClick={() => onButtonClick?.(10)}
            strokeWidth="2"
            fill="none"
          />

          {/* R3 - Button 11 */}
          <circle
            cx="350"
            cy="250"
            r="12"
            className={cn('cursor-pointer transition-colors opacity-50', getButtonClass(11))}
            onClick={() => onButtonClick?.(11)}
            strokeWidth="2"
            fill="none"
          />
        </g>
      </svg>

      {/* Legend */}
      <div className="mt-4 flex flex-wrap justify-center gap-4 text-sm">
        <div className="flex items-center gap-2">
          <div className="w-4 h-4 bg-gray-600 dark:bg-gray-400 rounded"></div>
          <span className="text-gray-700 dark:text-gray-300">Not Mapped</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="w-4 h-4 bg-purple-500 rounded"></div>
          <span className="text-gray-700 dark:text-gray-300">Mapped</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="w-4 h-4 bg-green-500 rounded"></div>
          <span className="text-gray-700 dark:text-gray-300">Pressed</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="w-4 h-4 bg-blue-500 rounded animate-pulse"></div>
          <span className="text-gray-700 dark:text-gray-300">Selected</span>
        </div>
      </div>
    </div>
  );
}