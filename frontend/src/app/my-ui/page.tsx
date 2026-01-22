'use client';

import { DashboardProvider } from '@/contexts/DashboardContext';
import { Dashboard } from '@/components/dashboard/Dashboard';
import { useWakeLock } from '@/hooks/useWakeLock';

// Force dynamic rendering to prevent static generation issues with chart components
export const dynamic = 'force-dynamic';

export default function MyUI() {
  // Keep the screen awake while on this page
  useWakeLock();
  return (
    <>
      <div className="w-full h-[calc(100vh-56px-24px)] flex flex-col md:flex-row items-stretch justify-between relative px-1">
        {/* Main content area */}
        <div className="w-full h-full flex flex-col justify-center pt-2 px-1">
          <div className="w-full h-full bg-white/5 backdrop-blur-sm rounded-lg">
            <DashboardProvider>
              <Dashboard />
            </DashboardProvider>
          </div>
        </div>
      </div>
    </>
  );
}
