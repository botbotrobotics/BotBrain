'use client';

import RobotHeader from '@/components/robot-header';
import SeparatorLine from '@/components/ui/separator-line';
import { ThemeProvider } from '@/contexts/ThemeContext';
import { RobotCustomModeProvider } from '@/contexts/RobotCustomModesContext';
import { ExtrasBar } from '@/components/extras-bar';
import ProtectedRoute from '@/components/protected-route';

export default function LogLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <ProtectedRoute>
      <div
        className="min-w-screen min-h-screen bg-clear-pink dark:bg-botbot-darkest text-black dark:text-white relative overflow-hidden"
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
    </ProtectedRoute>
  );
} 