'use client';

import { ReactNode } from 'react';
import RobotHeader from '@/components/robot-header';
import SeparatorLine from '@/components/ui/separator-line';
import { ThemeProvider } from '@/contexts/ThemeContext';
import { RobotCustomModeProvider } from '@/contexts/RobotCustomModesContext';
import { ExtrasBar } from '@/components/extras-bar';

export default function MapsLayout({ children }: { children: ReactNode }) {
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

          <main className="p-0 relative z-0 h-[calc(100vh-70px-24px)] overflow-hidden">
            {children}
          </main>
        </RobotCustomModeProvider>
      </ThemeProvider>
    </div>
  );
}