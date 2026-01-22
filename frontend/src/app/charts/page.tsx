'use client';

import { useEffect } from 'react';
import ChartsDashboard from '@/components/charts-dashboard';

// Force dynamic rendering to prevent static generation issues with ApexCharts
export const dynamic = 'force-dynamic';

export default function ChartsPage() {
  useEffect(() => {
    // Set page title
    document.title = 'Charts - BotBot';
  }, []);
  return (
    <div className="h-[calc(100vh-42px)]">
      <ChartsDashboard />
    </div>
  );
}
