'use client';

import { Widget } from './Widget';
import { useState, useRef, useEffect } from 'react';
import nipplejs, { JoystickManagerOptions, JoystickOutputData } from 'nipplejs';
import useJoystickMove from '@/hooks/useJoystickMove';
import { Vector2 } from 'three';
import { useDashboard } from '@/contexts/DashboardContext';
import { StickType } from '@/types/StickType';
import { Vector3 } from 'roslib';
import { useLanguage } from '@/contexts/LanguageContext';

interface JoystickWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  title?: string;
  props?: {
    title?: string;
    joystickSide?: StickType;
  };
}

export function JoystickWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 220, height: 220 },
  title = 'Joystick',
  props = {},
}: JoystickWidgetProps) {
  const [isDarkMode, setIsDarkMode] = useState(false);
  const { joystickMove } = useJoystickMove();
  const { updateWidgetProps } = useDashboard();
  const isLightTheme = () => !isDarkMode;
  const { t } = useLanguage();

  // State for settings
  const [showSettings, setShowSettings] = useState(false);
  const [currentTitle, setCurrentTitle] = useState(props.title || title);
  const [currentJoystickSide, setCurrentJoystickSide] = useState<StickType>(
    props.joystickSide || 'left'
  );

  // Refs for nipplejs
  const joystickRef = useRef<HTMLDivElement>(null);
  const joystickManagerRef = useRef<nipplejs.JoystickManager | null>(null);
  const joystickOutputDataRef = useRef<JoystickOutputData | undefined>(
    undefined
  );
  const useJoystickRef = useRef(false);

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

  const handleJoystickSideChange = (newSide: StickType) => {
    setCurrentJoystickSide(newSide);
    updateSettings({ joystickSide: newSide });
    // Recreate the joystick when changing sides
    recreateJoystick();
  };

  const createJoystickStyles = (zone: HTMLElement) => {
    const front = zone.querySelector('.front') as HTMLElement;
    const back = zone.querySelector('.back') as HTMLElement;
    if (front && back) {
      front.style.background = isLightTheme() ? '#B388FF' : '#6b21a8';
      back.style.background = isLightTheme() ? '#2D1A45' : 'white';
      back.style.boxShadow = isLightTheme()
        ? '0 0 15px rgba(138, 43, 226, 0.3)'
        : '0 0 15px rgba(179, 136, 255, 0.3)';
      front.style.border = '2px solid rgba(255, 255, 255, 0.1)';
    }
  };

  const normalizeAngle = (theta: number): number => {
    if (theta >= -Math.PI / 4 && theta <= Math.PI / 4) {
      return 1;
    } else if (theta >= (3 * Math.PI) / 4 || theta <= -(3 * Math.PI) / 4) {
      return -1;
    } else if (theta > Math.PI / 4 && theta < (3 * Math.PI) / 4) {
      return 1 - 2 * ((theta - Math.PI / 4) / (Math.PI / 2));
    } else {
      return -1 - 2 * ((theta + Math.PI / 4) / (Math.PI / 2));
    }
  };

  // Create or recreate the joystick
  const recreateJoystick = () => {
    if (!joystickRef.current) return;

    // Clean up existing joystick
    if (joystickManagerRef.current) {
      joystickManagerRef.current.destroy();
      joystickManagerRef.current = null;
    }

    // Create joystick with options
    const options: JoystickManagerOptions = {
      mode: 'static',
      size: 120,
      position: { bottom: '50%', left: '50%' },
      color: '#8A2BE2',
      fadeTime: 100,
      lockX: false,
      lockY: false,
      dynamicPage: true,
      restJoystick: true,
      catchDistance: 150,
      restOpacity: 0.6,
    };

    const joystick = nipplejs.create({
      ...options,
      zone: joystickRef.current,
    });

    joystickManagerRef.current = joystick;

    // Apply custom styles
    if (joystickRef.current) createJoystickStyles(joystickRef.current);

    const scale = 50;
    const interval = 50; // milliseconds
    let intervalId: NodeJS.Timeout;

    // Set up event listeners
    joystick.on('start', (evt, data) => {
      useJoystickRef.current = true;
      joystickOutputDataRef.current = data;

      // Start interval for continuous updates
      intervalId = setInterval(() => {
        if (useJoystickRef.current && joystickOutputDataRef.current) {
          const data = joystickOutputDataRef.current;

          if (currentJoystickSide === 'left') {
            joystickMove?.({
              stick: 'left',
              angle: data.angle.radian,
              force: Math.min(data.force ?? 1 / scale, 1),
              // vector: new Vector2(data?.vector.y, -(data?.vector.x ?? 0)),
              linear: new Vector2(data?.vector.y, -(data?.vector.x ?? 0)),
              angular: new Vector3(),
            });
          } else {
            const angle = Math.atan2(data.vector.y, data.vector.x);
            const normalizedAngle = normalizeAngle(angle);

            joystickMove?.({
              stick: 'right',
              angle: data.angle.radian,
              force: Math.min(data.force / scale, 1),
              angular: new Vector3({ x: 0, y: 0, z: -normalizedAngle }),
            });
          }
        }
      }, interval);
    });

    joystick.on('move', (evt, data) => {
      if (useJoystickRef.current) {
        joystickOutputDataRef.current = data;
      }
    });

    joystick.on('end', () => {
      useJoystickRef.current = false;
      clearInterval(intervalId);

      // Reset joystick values
      joystickMove?.({
        stick: currentJoystickSide,
        angle: 0,
        force: 0,
        linear: new Vector2(),
        angular: new Vector3(),
      });
    });
  };

  // Check for dark mode from DOM
  useEffect(() => {
    const checkDarkMode = () => {
      setIsDarkMode(document.documentElement.classList.contains('dark'));
    };

    // Initial check
    checkDarkMode();

    // Observe changes to the dark class
    const observer = new MutationObserver(checkDarkMode);
    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['class'],
    });

    return () => observer.disconnect();
  }, []);

  // Initialize joystick on mount and when side changes
  useEffect(() => {
    recreateJoystick();

    return () => {
      // Clean up on unmount
      if (joystickManagerRef.current) {
        joystickManagerRef.current.destroy();
        joystickManagerRef.current = null;
      }
    };
  }, [isDarkMode, joystickMove]);

  // Apply styles when theme changes
  useEffect(() => {
    if (joystickRef.current) createJoystickStyles(joystickRef.current);
  }, [isDarkMode]);

  return (
    <Widget
      id={id}
      title={currentTitle}
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      initialPosition={initialPosition}
      initialSize={initialSize}
      onSettingsClick={() => setShowSettings(!showSettings)}
    >
      <div className="h-full flex flex-col">
        {showSettings ? (
          <div className="mb-4 flex flex-col gap-2">
            <div>
              <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                {t('myUI', 'widgetTitle')}
              </label>
              <input
                type="text"
                value={currentTitle}
                onChange={(e) => handleTitleChange(e.target.value)}
                className="w-full px-3 py-2 bg-white dark:bg-botbot-darker border border-gray-300 dark:border-gray-600 rounded-md shadow-sm focus:outline-none focus:ring-1 focus:ring-purple-500 focus:border-purple-500"
              />
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                {t('myUI', 'joystickType')}
              </label>
              <select
                value={currentJoystickSide}
                onChange={(e) =>
                  handleJoystickSideChange(e.target.value as StickType)
                }
                className="w-full px-3 py-2 bg-white dark:bg-botbot-darker border border-gray-300 dark:border-gray-600 rounded-md shadow-sm focus:outline-none focus:ring-1 focus:ring-purple-500 focus:border-purple-500"
              >
                <option value="left">{t('myUI', 'leftJoystick')}</option>
                <option value="right">{t('myUI', 'rightJoystick')}</option>
              </select>
            </div>
          </div>
        ) : (
          <div
            ref={joystickRef}
            className="flex-1 w-full h-full pointer-events-auto"
            style={{
              background: isLightTheme()
                ? 'rgba(45, 26, 69, 0.3)'
                : 'rgba(106, 33, 168, 0.3)',
              borderRadius: '50%',
              backdropFilter: 'blur(8px)',
            }}
          />
        )}
      </div>
    </Widget>
  );
}
