'use client';

import { useState, useEffect } from 'react';
import {
  Moon,
  Sun,
  Bell,
  Gamepad2,
  User2,
  Maximize,
} from 'lucide-react';
import { useNotifications } from '@/contexts/NotificationsContext';
import { useDemoNotifications } from '@/hooks/useDemoNotifications';
import { JoysticksWrapper } from './ui/JoysticksWrapper';
import { NotificationsPanel } from './ui/notifications';
import { useHeader } from '@/contexts/HeaderContext';
import { useTheme } from '@/contexts/ThemeContext';
import { useLanguage } from '@/contexts/LanguageContext';
import { languageNames, LanguageCode } from '@/utils/translations';
import toggleFullscreen from '@/utils/toggle-fullscreen';
import Link from 'next/link';
import { UserProfilePopup } from './user-profile-popup';

export function ExtrasBar() {
  const [darkMode, setDarkMode] = useState(false);
  const [showNotifications, setShowNotifications] = useState(false);
  const [showLanguageSelector, setShowLanguageSelector] = useState(false);
  const [gamepadConnected, setGamepadConnected] = useState(false);
  const { joystickEnabled, setJoystickEnabled, extrasBarVisible } = useHeader();
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

  // If the extras bar is not visible, render a hidden version for animation
  return (
    <>
      <div 
        className={`w-full h-12 bg-clear-pink dark:bg-botbot-darker border-b border-b-clear-gray dark:border-b-black transition-all duration-300 ease-in-out overflow-hidden ${
          extrasBarVisible ? 'opacity-100 max-h-12' : 'opacity-0 max-h-0 border-b-0'
        }`}
      >
        <div className="max-w-screen-xl mx-auto h-full py-1 px-4">
          {/* Single flex container with evenly spaced items */}
          <div className="flex items-center justify-center h-full">
            <div className="w-full flex justify-evenly items-center">
              {/* Notifications */}
              <div className="flex items-center justify-center">
                <button
                  data-notifications-trigger
                  onClick={() => {
                    setShowLanguageSelector(false);
                    setShowNotifications(!showNotifications);
                  }}
                  className="flex items-center justify-center space-x-1 text-gray-500 hover:text-primary dark:text-white dark:hover:text-botbot-accent relative"
                >
                  <Bell className="w-6 h-6" />
                  {unreadCount > 0 && (
                    <span className="absolute -top-1 -right-1 min-w-5 h-5 bg-[#821db7] dark:bg-red-700 text-white text-xs flex items-center justify-center rounded-full animate-pulse">
                      {unreadCount}
                    </span>
                  )}
                  <span className="text-sm hidden sm:inline ml-1">{t('sidebar', 'notifications')}</span>
                </button>
              </div>

              {/* Joysticks toggle */}
              <div className="flex items-center justify-center">
                <button
                  onClick={() => setJoystickEnabled(!joystickEnabled)}
                  className={`flex items-center justify-center space-x-1 ${
                    joystickEnabled
                      ? 'text-primary dark:text-botbot-accent'
                      : 'text-gray-500 hover:text-primary dark:text-white dark:hover:text-botbot-accent'
                  }`}
                >
                  <div className="relative">
                    <Gamepad2 className="w-6 h-6" />
                    <div
                      className={`absolute -top-1 -right-1 w-2.5 h-2.5 rounded-full border border-clear-pink dark:border-botbot-darker ${
                        gamepadConnected ? 'bg-green-500 animate-pulse' : 'bg-red-500'
                      }`}
                    ></div>
                  </div>
                  <span className="text-sm hidden sm:inline ml-1">{t('sidebar', 'joystick')}</span>
                </button>
              </div>

              {/* Full screen toggle */}
              <div className="flex items-center justify-center">
                <button
                  className="flex items-center justify-center space-x-1 text-gray-500 hover:text-primary dark:text-white dark:hover:text-botbot-accent"
                  onClick={toggleFullscreen}
                >
                  <Maximize className="w-6 h-6" />
                  <span className="text-sm hidden sm:inline ml-1">{t('sidebar', 'fullScreen')}</span>
                </button>
              </div>

              {/* Dark mode toggle */}
              <div className="flex items-center justify-center">
                <button
                  className="flex items-center justify-center space-x-1 text-gray-500 hover:text-primary dark:text-white dark:hover:text-botbot-accent"
                  onClick={toggleDarkMode}
                >
                  {darkMode ? (
                    <>
                      <Sun className="w-6 h-6" />
                      <span className="text-sm hidden sm:inline ml-1">{t('sidebar', 'lightMode')}</span>
                    </>
                  ) : (
                    <>
                      <Moon className="w-6 h-6" />
                      <span className="text-sm hidden sm:inline ml-1">{t('sidebar', 'darkMode')}</span>
                    </>
                  )}
                </button>
              </div>

              {/* Language toggle */}
              <div className="flex items-center justify-center">
                <button
                  data-language-trigger
                  className="flex items-center justify-center space-x-1 text-gray-500 hover:text-primary dark:text-white dark:hover:text-botbot-accent"
                  onClick={() => {
                    setShowNotifications(false);
                    setShowLanguageSelector(!showLanguageSelector);
                  }}
                >
                  <span className="text-lg">{getLanguageEmoji()}</span>
                  <span className="text-sm hidden sm:inline ml-1">{t('sidebar', 'language')}</span>
                </button>
              </div>
            </div>
          </div>
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