'use client';

import { useState, useRef, useEffect } from 'react';
import NavButtonEnhanced from './ui/nav-button-enhanced';
import Image from 'next/image';
import { useHeader } from '../contexts/HeaderContext';
import { MenuActionType } from '../types/RobotActionTypes';
import useWindowWidth from '@/hooks/useWindowWidth';
import { JoysticksWrapper } from './ui/JoysticksWrapper';
import useMenuActions from '@/hooks/useMenuActions';
import { usePathname } from 'next/navigation';
import Link from 'next/link';
import {
  Network,
  Layout,
  FlaskConical,
  Sparkles,
  MapIcon,
  Heart,
  Volume2,
  User,
  Settings,
  ChevronDown,
  ChevronUp,
  Home,
  LayoutDashboard
} from 'lucide-react';

const defaultPath = '/icons/list-purple.svg';

export default function NavMenu() {
  const [open, setOpen] = useState(false);
  const [listIcon, setListIcon] = useState(defaultPath);
  const [isDarkMode, setIsDarkMode] = useState(false);
  const { joystickEnabled } = useHeader();
  const menuRef = useRef<HTMLDivElement>(null);
  const buttonRef = useRef<HTMLButtonElement>(null);
  const collapsibleBreakpoint = 1280; // Changed to xl breakpoint for tablets to use mobile
  const pathname = usePathname();

  const { navMenuCollapsed, updateNavMenuCollapsed } = useHeader();

  const desktopNavClasses = navMenuCollapsed ? 'hidden' : 'hidden xl:flex items-center gap-3';
  const mobileNavClasses = navMenuCollapsed ? 'relative xl:hidden' : 'hidden';
  const menuActions = useMenuActions();

  // Icon mapping for menu items
  const iconMap: Record<string, React.ComponentType<any> | null> = {
    'Dashboard': LayoutDashboard,
    'Fleet': Network,
    'Robot Control': null, // Keep custom SVG for Robot Control
    'My UI': Layout,
    'Labs': FlaskConical,
    'AI': Sparkles,
    'Maps': MapIcon,
    'Health': Heart,
    'Soundboard': Volume2,
    'User': User,
    'Settings': Settings,
    'Extras': (typeof menuActions.extras.icon === 'string' && menuActions.extras.icon.includes('chevron-up')) ? ChevronUp : ChevronDown
  };

  // OSS nav items - always available
  const ossNavButtons: MenuActionType[] = [
    menuActions.dashboard,
    menuActions.fleet,
    menuActions.home,
    menuActions.myUi,
    menuActions.labs,
    menuActions.maps,
    menuActions.health,
    menuActions.user,
    menuActions.settings,
    menuActions.extras,
  ];

  // Pro nav items - only available in Pro builds
  const proNavButtons: MenuActionType[] = __PRO__ ? [
    menuActions.ai,
    menuActions.soundboard,
  ] : [];

  // Insert Pro items before user/settings/extras
  const navButtons: MenuActionType[] = [
    ...ossNavButtons.slice(0, -3), // All items except user, settings, extras
    ...proNavButtons,
    ...ossNavButtons.slice(-3), // user, settings, extras
  ];

  const width = useWindowWidth();

  // Check for dark mode from DOM
  useEffect(() => {
    const checkDarkMode = () => {
      const isDark = document.documentElement.classList.contains('dark');
      setIsDarkMode(isDark);
      if (isDark) {
        setListIcon('/icons/white/list.svg');
      } else {
        setListIcon(defaultPath);
      }
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

  // Atualiza o estado do header conforme a largura da janela do navegador
  useEffect(() => {
    const collapsed = width < collapsibleBreakpoint;
    updateNavMenuCollapsed(collapsed);
    if (!collapsed) setOpen(false);
  }, [width, updateNavMenuCollapsed, collapsibleBreakpoint]);

  // Fecha o menu se clicar fora (válido para a versão mobile)
  useEffect(() => {
    const handler = (event: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        setOpen(false);
      }
    };

    document.addEventListener('mousedown', handler);
    return () => document.removeEventListener('mousedown', handler);
  }, []);

  return (
    <nav className="flex flex-row justify-end">
      {/* Desktop Version: visible from xl breakpoint only */}
      <div className={desktopNavClasses}>
        {/* <span>{width}px</span> */}
        {navButtons.map((btn, index) => (
          <NavButtonEnhanced
            key={index}
            name={btn.label}
            icon={iconMap[btn.label] || btn.icon}
            onClick={btn.action}
            size={btn.iconDefaultSize || 20}
            isActive={
              (btn.label === 'Dashboard' && pathname === '/dashboard') ||
              (btn.label === 'Fleet' && pathname === '/fleet') ||
              (btn.label === 'Home' && pathname === '/robot-home') ||
              (btn.label === 'My UI' && pathname === '/my-ui') ||
              (btn.label === 'User' && pathname === '/profile') ||
              (btn.label === 'Settings' && pathname === '/settings') ||
              (btn.label === 'Labs' && pathname === '/labs') ||
              (btn.label === 'AI' && pathname === '/ai') ||
              (btn.label === 'Maps' && pathname === '/maps') ||
              (btn.label === 'Soundboard' && pathname === '/soundboard') ||
              (btn.label === 'Health' && pathname === '/health')
            }
          />
        ))}
      </div>

      {/* Mobile Version: displayed only on screens smaller than lg */}
      <div className={mobileNavClasses} ref={menuRef}>
        {/* Hamburger button */}
        <button
          ref={buttonRef}
          onClick={() => setOpen(!open)}
          className="p-2 focus:outline-none transition-transform duration-200 hover:scale-110"
          aria-label="Menu"
          aria-expanded={open}
        >
          <Image src={listIcon} alt="menu" width={25} height={24} />
        </button>
        {open && (
          <div className="absolute right-0 mt-2 w-48 bg-white dark:bg-botbot-darker ring-1 ring-gray-200 dark:ring-gray-700 shadow-xl rounded-lg px-3 py-3 z-50 flex flex-col gap-1">
            {navButtons.map((btn, index) => (
              <NavButtonEnhanced
                key={index}
                name={btn.label}
                icon={iconMap[btn.label] || btn.icon}
                layout="mobile"
                onClick={btn.action}
                isActive={
                  (btn.label === 'Dashboard' && pathname === '/dashboard') ||
                  (btn.label === 'Fleet' && pathname === '/fleet') ||
                  (btn.label === 'Home' && pathname === '/robot-home') ||
                  (btn.label === 'My UI' && pathname === '/my-ui') ||
                  (btn.label === 'User' && pathname === '/profile') ||
                  (btn.label === 'Settings' && pathname === '/settings') ||
                  (btn.label === 'Labs' && pathname === '/labs') ||
                  (btn.label === 'AI' && pathname === '/ai') ||
                  (btn.label === 'Maps' && pathname === '/maps') ||
                  (btn.label === 'Soundboard' && pathname === '/soundboard') ||
                  (btn.label === 'Health' && pathname === '/health')
                }
              />
            ))}
          </div>
        )}
      </div>

      <JoysticksWrapper enabled={joystickEnabled} />
    </nav>
  );
}
