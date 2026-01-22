'use client';

import toggleFullscreen from '@/utils/toggle-fullscreen';
import { useHeader } from '../contexts/HeaderContext';
import { MenuActionType, MenuActionTypeName } from '../types/RobotActionTypes';
import { useSafeNavigation } from './useSafeNavigation';
import { useEffect, useState } from 'react';

export default function useMenuActions(): Record<
  MenuActionTypeName,
  MenuActionType
> {
  const header = useHeader();
  const { navigate } = useSafeNavigation();
  const [isDarkMode, setIsDarkMode] = useState(false);

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

  const getDarkModeFolder = () => {
    return `${isDarkMode ? 'white/' : ''}`;
  };

  return {
    expandContainer: {
      label: 'Expandir Container',
      icon: `${getDarkModeFolder()}expand`,
      action: () => alert('Expandir Container'),
    },
    dashboard: {
      label: 'Dashboard',
      icon: `${getDarkModeFolder()}home`,
      action: () => navigate('/dashboard'),
    },
    fleet: {
      label: 'Fleet',
      icon: `${getDarkModeFolder()}network`,
      action: () => navigate('/fleet'),
    },
    home: {
      label: 'Cockpit',
      icon: `${getDarkModeFolder()}bot-home`,
      action: () => navigate('/cockpit'),
    },
    chart: {
      label: 'Chart',
      icon: `${getDarkModeFolder()}chart`,
      action: () => navigate('/charts'),
    },
    chatMenu: {
      label: 'Chat',
      icon: `${getDarkModeFolder()}chat`,
      action: () => header.setChatPopupOpen(!header.chatPopupOpen),
    },
    chatNew: {
      label: 'Novo chat com o robô',
      icon: `${getDarkModeFolder()}new-chat`,
      action: () => alert('Novo chat com o robô'),
    },
    user: {
      label: 'User',
      icon: `${getDarkModeFolder()}user`,
      action: () => navigate('/profile'),
    },
    settings: {
      label: 'Settings',
      icon: `${getDarkModeFolder()}settings`,
      action: () => navigate('/settings'),
    },
    help: {
      label: 'Help',
      icon: `${getDarkModeFolder()}circle-help`,
      action: () => navigate('/help'),
    },
    mapEdit: {
      label: 'Map Edit',
      icon: `${getDarkModeFolder()}map`,
      action: () => navigate('/map-edit'),
    },
    labs: {
      label: 'Labs',
      icon: `${getDarkModeFolder()}flask`,
      action: () => navigate('/labs'),
    },
    extras: {
      label: 'Extras',
      icon: header.extrasBarVisible ? `${getDarkModeFolder()}chevron-down` : `${getDarkModeFolder()}chevron-up`,
      action: () => header.setExtrasBarVisible(!header.extrasBarVisible),
    },
    dPad: {
      label: 'D-Pad',
      icon: `${getDarkModeFolder()}dpad`,
      action: () => alert('D-Pad Menu'),
    },
    fullScreen: {
      label: 'Full Screen',
      icon: `${getDarkModeFolder()}expand`,
      action: () => {
        toggleFullscreen();
      },
    },
    menu: {
      label: 'Menu',
      icon: `${getDarkModeFolder()}meatballs-menu`,
      iconDefaultSize: 15,
      action: () => alert('Menu'),
    },
    myUi: {
      label: 'My UI',
      icon: `${getDarkModeFolder()}layout`,
      action: () => navigate('/my-ui'),
    },
    health: {
      label: 'Health',
      icon: `${getDarkModeFolder()}heart`,
      action: () => navigate('/health'),
    },
    ai: {
      label: 'AI',
      icon: `${getDarkModeFolder()}brain`,
      action: () => navigate('/ai'),
    },
    maps: {
      label: 'Maps',
      icon: `${getDarkModeFolder()}map`,
      action: () => navigate('/maps'),
    },
    soundboard: {
      label: 'Soundboard',
      icon: `${getDarkModeFolder()}sound`,
      action: () => navigate('/soundboard'),
    },
  };
}
