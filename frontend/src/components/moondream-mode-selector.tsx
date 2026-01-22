'use client';

import { useState } from 'react';

interface MoondreamModeSelectorProps {
  mode: 'yolo' | 'moondream';
  onModeChange: (mode: 'yolo' | 'moondream') => void;
}

export default function MoondreamModeSelector({ mode, onModeChange }: MoondreamModeSelectorProps) {
  const [isAnimating, setIsAnimating] = useState(false);

  const handleToggle = () => {
    setIsAnimating(true);
    const newMode = mode === 'yolo' ? 'moondream' : 'yolo';
    onModeChange(newMode);
    setTimeout(() => setIsAnimating(false), 300);
  };

  return (
    <div className="flex items-center gap-4">
      <span className={`text-sm font-medium transition-colors ${
        mode === 'yolo' ? 'text-gray-900 dark:text-white' : 'text-gray-500 dark:text-gray-400'
      }`}>
        YOLO
      </span>

      <button
        onClick={handleToggle}
        className={`
          relative inline-flex h-7 w-14 items-center rounded-full
          transition-colors duration-300 ease-in-out focus:outline-none
          focus-visible:ring-2 focus-visible:ring-blue-500 focus-visible:ring-offset-2
          ${mode === 'moondream'
            ? 'bg-gradient-to-r from-purple-500 to-blue-500'
            : 'bg-gray-300 dark:bg-gray-700'
          }
          ${isAnimating ? 'scale-95' : 'scale-100'}
        `}
        style={{ transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)' }}
        aria-label={`Switch to ${mode === 'yolo' ? 'MoonDream' : 'YOLO'} mode`}
      >
        <span
          className={`
            ${mode === 'moondream' ? 'translate-x-7' : 'translate-x-1'}
            inline-block h-5 w-5 transform rounded-full
            bg-white shadow-lg ring-0 transition duration-300 ease-in-out
            ${isAnimating ? 'scale-110' : 'scale-100'}
          `}
        >
          {/* Optional: Add icons inside the toggle */}
          <span className="flex h-full w-full items-center justify-center text-xs">
            {mode === 'moondream' ? '🌙' : '👁️'}
          </span>
        </span>
      </button>

      <span className={`text-sm font-medium transition-colors ${
        mode === 'moondream' ? 'text-gray-900 dark:text-white' : 'text-gray-500 dark:text-gray-400'
      }`}>
        MoonDream
      </span>
    </div>
  );
}