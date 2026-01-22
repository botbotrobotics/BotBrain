'use client';

import RobotHeader from '@/components/robot-header';
import SeparatorLine from '@/components/ui/separator-line';
import { ExtrasBar } from '@/components/extras-bar';
import ProtectedRoute from '@/components/protected-route';

export default function FleetLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <ProtectedRoute>
      <div className="min-w-screen min-h-screen bg-clear-pink dark:bg-botbot-darkest text-black dark:text-white">
        <RobotHeader />
        <SeparatorLine />
        <ExtrasBar />
        <main>{children}</main>
      </div>
    </ProtectedRoute>
  );
} 