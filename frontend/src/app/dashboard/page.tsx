'use client';

import { useEffect } from 'react';
import WelcomeSection from './components/WelcomeSection';
import EnhancedQuickStats from './components/EnhancedQuickStats';
import ActivityHeatmap from './components/ActivityHeatmap';
import RobotUsageAnalytics from './components/RobotUsageAnalytics';
import CompactQuickAccess from './components/CompactQuickAccess';
import RobotFleetSection from './components/RobotFleetSection';

export default function DashboardPage() {
  useEffect(() => {
    // Set page title
    document.title = 'Dashboard - BotBot';
  }, []);

  return (
    <div className="h-screen overflow-y-auto bg-gradient-to-b from-transparent to-purple-50/10 dark:to-purple-900/5">
      {/* Main content */}
      <div className="p-4 sm:p-6 lg:p-8">
        <div className="max-w-[2000px] mx-auto space-y-5 lg:space-y-6">
          {/* Welcome Section with fade-in animation */}
          <div className="animate-fade-in">
            <WelcomeSection />
          </div>

          {/* Quick Access - Compact and at the top for easy navigation */}
          <div className="animate-fade-in animation-delay-50">
            <CompactQuickAccess />
          </div>

          {/* Enhanced Quick Stats with staggered animation */}
          <div className="animate-fade-in animation-delay-100">
            <EnhancedQuickStats />
          </div>

          {/* Analytics Grid - Activity and Robot Usage */}
          <div className="grid grid-cols-1 xl:grid-cols-2 gap-5 animate-fade-in animation-delay-150">
            <div className="min-h-[280px]">
              <ActivityHeatmap />
            </div>
            <div className="min-h-[280px]">
              <RobotUsageAnalytics />
            </div>
          </div>

          {/* Robot Fleet Section with slide-up animation */}
          <div className="animate-slide-up animation-delay-200 pb-8">
            <RobotFleetSection />
          </div>
        </div>
      </div>
    </div>
  );
}