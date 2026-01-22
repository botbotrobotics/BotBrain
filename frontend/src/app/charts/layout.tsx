'use client';

import { ReactNode } from 'react';
import RobotHeader from '@/components/robot-header';
import SeparatorLine from '@/components/ui/separator-line';
import { ThemeProvider } from '@/contexts/ThemeContext';
import { RobotCustomModeProvider } from '@/contexts/RobotCustomModesContext';
import { ExtrasBar } from '@/components/extras-bar';

export default function ChartsLayout({ children }: { children: ReactNode }) {
  return (
    <div
      className="min-w-screen h-full bg-clear-pink dark:bg-botbot-darkest text-black dark:text-white relative"
      style={{ isolation: 'isolate' }}
    >
      <ThemeProvider>
        <RobotCustomModeProvider>
          <RobotHeader />
          <SeparatorLine />
          <ExtrasBar />

          <div className="w-full h-full relative z-0 bg-clear-pink">
            {children}
          </div>
        </RobotCustomModeProvider>
      </ThemeProvider>
    </div>
  );
}
