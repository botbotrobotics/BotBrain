'use client';

import { useState, useEffect } from 'react';
import {
  Moon,
  Sun,
  ChevronLeft,
  ChevronRight,
  Bell,
  Gamepad2,
  User2,
  Maximize,
} from 'lucide-react';
import { useNotifications } from '@/contexts/NotificationsContext';
import { useDemoNotifications } from '@/hooks/useDemoNotifications';
import { UserProfilePopup } from './user-profile-popup';
import { JoysticksWrapper } from './ui/JoysticksWrapper';
import { NotificationsPanel } from './ui/notifications';
import { useHeader } from '@/contexts/HeaderContext';
import { useTheme } from '@/contexts/ThemeContext';
import { useLanguage } from '@/contexts/LanguageContext';
import { languageNames, LanguageCode } from '@/utils/translations';
import toggleFullscreen from '@/utils/toggle-fullscreen';

export function Sidebar() {
  const [darkMode, setDarkMode] = useState(false);
  const [isCollapsed, setIsCollapsed] = useState(true);
  const [showNotifications, setShowNotifications] = useState(false);
  const [showLanguageSelector, setShowLanguageSelector] = useState(false);
  const [gamepadConnected, setGamepadConnected] = useState(false);
  const { joystickEnabled, setJoystickEnabled } = useHeader();
  const { state: notificationsState } = useNotifications();
  const { themeColor, setThemeColor } = useTheme();
  const { language, setLanguage, t } = useLanguage();

  // Initialize demo notifications
  useDemoNotifications();

  // Initialize dark mode from DOM
  useEffect(() => {
    const isDark = document.documentElement.classList.contains('dark');
    setDarkMode(isDark);
  }, []);

  // Check for gamepad connectivity
  useEffect(() => {
    const checkGamepads = () => {
      const gamepads = navigator.getGamepads();
      setGamepadConnected(gamepads.some((gamepad) => gamepad !== null));
    };

    // Initial check
    checkGamepads();

    // Event listeners for gamepad connections/disconnections
    const handleGamepadConnected = () => {
      setGamepadConnected(true);
    };

    const handleGamepadDisconnected = () => {
      // We need to check if any gamepads are still connected
      setTimeout(checkGamepads, 100);
    };

    window.addEventListener('gamepadconnected', handleGamepadConnected);
    window.addEventListener('gamepaddisconnected', handleGamepadDisconnected);

    // Regular check for gamepad status
    const interval = setInterval(checkGamepads, 1000);

    return () => {
      window.removeEventListener('gamepadconnected', handleGamepadConnected);
      window.removeEventListener(
        'gamepaddisconnected',
        handleGamepadDisconnected
      );
      clearInterval(interval);
    };
  }, []);

  const unreadCount = notificationsState.notifications.filter(
    (n) => !n.read
  ).length;

  // Get language emoji based on current language
  const getLanguageEmoji = () => {
    return language === 'en' ? '🇬🇧' : '🇧🇷';
  };

  // Handle escape key to close panels
  useEffect(() => {
    const handleEscape = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        setShowNotifications(false);
        setShowLanguageSelector(false);
      }
    };

    document.addEventListener('keydown', handleEscape);
    return () => document.removeEventListener('keydown', handleEscape);
  }, []);

  // Handle click outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      const target = event.target as HTMLElement;
      if (
        !target.closest('[data-notifications-panel]') &&
        !target.closest('[data-notifications-trigger]') &&
        !target.closest('[data-language-selector]') &&
        !target.closest('[data-language-trigger]')
      ) {
        setShowNotifications(false);
        setShowLanguageSelector(false);
      }
    };

    if (showNotifications || showLanguageSelector) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [showNotifications, showLanguageSelector]);

  // Add overlay when panels are shown
  useEffect(() => {
    if (showNotifications || showLanguageSelector) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = 'auto';
    }
    return () => {
      document.body.style.overflow = 'auto';
    };
  }, [showNotifications, showLanguageSelector]);

  const toggleDarkMode = () => {
    setDarkMode(!darkMode);
    if (darkMode) {
      document.documentElement.classList.remove('dark');
    } else {
      document.documentElement.classList.add('dark');
    }
  };

  return (
    <>
      <div
        className={`flex flex-col ${
          isCollapsed ? 'w-3 md:w-3' : 'w-16'
        } h-full bg-clear-pink dark:bg-botbot-darker border-l border-l-clear-gray dark:border-l-black transition-all duration-200 ease-in-out`}
      >
        {/* Collapse toggle */}
        <button
          className={`flex items-center justify-center w-full h-12 text-gray-500 hover:text-primary dark:text-white dark:hover:text-gray-400 transition-colors ${
            isCollapsed ? 'pl-0' : ''
          }`}
          onClick={() => setIsCollapsed(!isCollapsed)}
        >
          {isCollapsed ? (
            <ChevronLeft className="w-3 h-3" />
          ) : (
            <ChevronRight className="w-5 h-5" />
          )}
        </button>

        {/* Sidebar content - only visible when expanded */}
        <div
          className={`flex flex-col h-[70%] items-center py-4 space-y-6 ${
            isCollapsed ? 'hidden' : 'block'
          }`}
        >
          {/* Notifications */}
          <div className="relative">
            <button
              data-notifications-trigger
              onClick={() => {
                setShowLanguageSelector(false);
                setShowNotifications(!showNotifications);
              }}
              className="w-8 h-8 flex items-center justify-center rounded-full bg-gray-200 dark:bg-botbot-dark text-gray-700 dark:text-white hover:bg-primary hover:dark:bg-botbot-purple hover:text-white relative"
            >
              <Bell className="w-5 h-5" />
              {unreadCount > 0 && (
                <span className="absolute -top-1 -right-1 min-w-4 h-4 bg-[#821db7] dark:bg-red-700 text-white text-xs flex items-center justify-center rounded-full animate-pulse">
                  {unreadCount}
                </span>
              )}
            </button>
          </div>

          {/* Joysticks toggle */}
          <div className="flex flex-col items-center space-y-1">
            <button
              onClick={() => setJoystickEnabled(!joystickEnabled)}
              className={`w-8 h-8 flex items-center justify-center rounded-full ${
                joystickEnabled
                  ? 'bg-primary dark:bg-botbot-darkest text-white'
                  : 'bg-gray-200 dark:bg-botbot-dark text-gray-700 dark:text-white hover:bg-primary hover:dark:bg-botbot-purple hover:text-white'
              }`}
            >
              <Gamepad2 className="w-5 h-5" />
            </button>
            <div
              className={`w-3 h-3 rounded-full ${
                gamepadConnected ? 'bg-green-500 animate-pulse' : 'bg-red-500'
              }`}
            ></div>
          </div>
        </div>

        {/* Bottom actions - only visible when expanded */}
        <div
          className={`flex flex-col items-center py-4 space-y-4 ${
            isCollapsed ? 'hidden' : 'block'
          }`}
        >
          {/* Full screen toggle */}
          <button
            className="flex flex-col items-center justify-center space-y-1 text-gray-500 hover:text-primary dark:text-white dark:hover:text-botbot-accent"
            onClick={toggleFullscreen}
          >
            <Maximize className="w-5 h-5" />
            <span className="text-xs">{t('sidebar', 'fullScreen')}</span>
          </button>

          {/* Dark mode toggle */}
          <button
            className="flex flex-col items-center justify-center space-y-1 text-gray-500 hover:text-primary dark:text-white dark:hover:text-botbot-accent"
            onClick={toggleDarkMode}
          >
            {darkMode ? (
              <>
                <Sun className="w-5 h-5" />
                <span className="text-xs">{t('sidebar', 'lightMode')}</span>
              </>
            ) : (
              <>
                <Moon className="w-5 h-5" />
                <span className="text-xs">{t('sidebar', 'darkMode')}</span>
              </>
            )}
          </button>

          {/* Language toggle */}
          <button
            data-language-trigger
            className="flex flex-col items-center justify-center space-y-1 text-gray-500 hover:text-primary dark:text-white dark:hover:text-botbot-accent"
            onClick={() => {
              setShowNotifications(false);
              setShowLanguageSelector(!showLanguageSelector);
            }}
          >
            <span className="text-xl">{getLanguageEmoji()}</span>
            <span className="text-xs">{t('sidebar', 'language')}</span>
          </button>

          {/* User profile */}
          <UserProfilePopup>
            <button
              className="relative w-8 h-8 flex items-center justify-center rounded-full bg-gray-200 dark:bg-botbot-dark text-gray-700 dark:text-white hover:bg-primary hover:text-white z-50"
              aria-label="Open user menu"
              type="button"
            >
              <User2 className="w-5 h-5" />
            </button>
          </UserProfilePopup>
        </div>
      </div>

      {/* Notifications overlay and panel */}
      {showNotifications && (
        <>
          <div className="fixed inset-0 bg-black/20 dark:bg-black/40 backdrop-blur-sm z-[90]" />
          <div data-notifications-panel>
            <NotificationsPanel />
          </div>
        </>
      )}

      {/* Language selector overlay and panel */}
      {showLanguageSelector && (
        <>
          <div className="fixed inset-0 bg-black/20 dark:bg-black/40 backdrop-blur-sm z-[90]" />
          <div
            data-language-selector
            className="fixed top-[30%] left-1/2 -translate-x-1/2 -translate-y-1/2 bg-white dark:bg-botbot-dark shadow-lg rounded-lg p-4 z-[100] min-w-[250px]"
          >
            <h2 className="text-lg font-semibold mb-3 text-center dark:text-white">
              {t('sidebar', 'language')}
            </h2>
            <div className="space-y-2">
              {(Object.keys(languageNames) as LanguageCode[]).map((code) => (
                <button
                  key={code}
                  className={`w-full py-2 px-4 text-left rounded-md transition flex items-center gap-2 ${
                    language === code
                      ? 'bg-primary/10 text-primary dark:bg-botbot-purple/20 dark:text-botbot-accent font-medium'
                      : 'hover:bg-gray-100 dark:hover:bg-botbot-darker'
                  }`}
                  onClick={() => {
                    setLanguage(code);
                    setShowLanguageSelector(false);
                  }}
                >
                  <span className="text-xl">{code === 'en' ? '🇬🇧' : '🇧🇷'}</span>
                  {code === 'en' ? 'English' : 'Português'}
                </button>
              ))}
            </div>
          </div>
        </>
      )}

      {/* Joysticks overlay */}
      <JoysticksWrapper enabled={joystickEnabled} />
    </>
  );
} 