'use client';

import { useEffect, useState } from 'react';
import { Clock, Calendar, MapPin } from 'lucide-react';

export default function ClockWidget() {
  const [currentTime, setCurrentTime] = useState(new Date());
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
    const timer = setInterval(() => {
      setCurrentTime(new Date());
    }, 1000);

    return () => clearInterval(timer);
  }, []);

  if (!mounted) {
    return null; // Prevent SSR mismatch
  }

  const formatTime = () => {
    return currentTime.toLocaleTimeString('en-US', {
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit',
      hour12: false,
    });
  };

  const formatDate = () => {
    return currentTime.toLocaleDateString('en-US', {
      weekday: 'short',
      month: 'short',
      day: 'numeric',
      year: 'numeric',
    });
  };

  const getTimezone = () => {
    return Intl.DateTimeFormat().resolvedOptions().timeZone;
  };

  const getAnalogClockAngles = () => {
    const hours = currentTime.getHours() % 12;
    const minutes = currentTime.getMinutes();
    const seconds = currentTime.getSeconds();

    return {
      hour: (hours * 30) + (minutes * 0.5), // 30 degrees per hour + minute adjustment
      minute: minutes * 6, // 6 degrees per minute
      second: seconds * 6, // 6 degrees per second
    };
  };

  const angles = getAnalogClockAngles();

  return (
    <div className="bg-white dark:bg-botbot-dark rounded-2xl p-6 shadow-md border border-gray-100 dark:border-botbot-darker">
      <div className="flex items-start justify-between mb-4">
        <div>
          <h3 className="text-lg font-semibold text-gray-900 dark:text-white flex items-center gap-2">
            <Clock className="h-5 w-5 text-violet-600 dark:text-violet-400" />
            System Time
          </h3>
          <div className="flex items-center gap-4 mt-2 text-sm text-gray-600 dark:text-gray-400">
            <div className="flex items-center gap-1">
              <Calendar className="h-3 w-3" />
              <span>{formatDate()}</span>
            </div>
            <div className="flex items-center gap-1">
              <MapPin className="h-3 w-3" />
              <span>{getTimezone()}</span>
            </div>
          </div>
        </div>
      </div>

      <div className="flex items-center justify-between gap-6">
        {/* Digital Clock */}
        <div className="flex-1">
          <div className="text-4xl font-bold text-gray-900 dark:text-white font-mono tracking-wider">
            {formatTime()}
          </div>
          <div className="mt-2 flex gap-2">
            {['H', 'M', 'S'].map((label, index) => (
              <div key={label} className="text-xs text-gray-500 dark:text-gray-500">
                <span className="inline-block w-[4.5ch] text-center">{label}</span>
              </div>
            ))}
          </div>
        </div>

        {/* Analog Clock */}
        <div className="relative w-24 h-24">
          <svg
            className="w-full h-full transform -rotate-90"
            viewBox="0 0 100 100"
          >
            {/* Clock face */}
            <circle
              cx="50"
              cy="50"
              r="48"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              className="text-gray-200 dark:text-gray-700"
            />

            {/* Hour markers */}
            {[...Array(12)].map((_, i) => {
              const angle = i * 30;
              const isMainHour = i % 3 === 0;
              const length = isMainHour ? 8 : 4;
              const x1 = 50 + 40 * Math.cos(angle * Math.PI / 180);
              const y1 = 50 + 40 * Math.sin(angle * Math.PI / 180);
              const x2 = 50 + (48 - length) * Math.cos(angle * Math.PI / 180);
              const y2 = 50 + (48 - length) * Math.sin(angle * Math.PI / 180);

              return (
                <line
                  key={i}
                  x1={x1}
                  y1={y1}
                  x2={x2}
                  y2={y2}
                  stroke="currentColor"
                  strokeWidth={isMainHour ? "2" : "1"}
                  className="text-gray-400 dark:text-gray-600"
                />
              );
            })}

            {/* Hour hand */}
            <line
              x1="50"
              y1="50"
              x2="50"
              y2="28"
              stroke="currentColor"
              strokeWidth="4"
              strokeLinecap="round"
              className="text-violet-600 dark:text-violet-400 transition-transform duration-500"
              style={{ transform: `rotate(${angles.hour}deg)`, transformOrigin: '50px 50px' }}
            />

            {/* Minute hand */}
            <line
              x1="50"
              y1="50"
              x2="50"
              y2="15"
              stroke="currentColor"
              strokeWidth="3"
              strokeLinecap="round"
              className="text-purple-600 dark:text-purple-400 transition-transform duration-500"
              style={{ transform: `rotate(${angles.minute}deg)`, transformOrigin: '50px 50px' }}
            />

            {/* Second hand */}
            <line
              x1="50"
              y1="50"
              x2="50"
              y2="10"
              stroke="currentColor"
              strokeWidth="1"
              strokeLinecap="round"
              className="text-red-500 dark:text-red-400 transition-transform duration-100"
              style={{ transform: `rotate(${angles.second}deg)`, transformOrigin: '50px 50px' }}
            />

            {/* Center dot */}
            <circle
              cx="50"
              cy="50"
              r="3"
              fill="currentColor"
              className="text-gray-800 dark:text-gray-200"
            />
          </svg>
        </div>
      </div>
    </div>
  );
}