'use client';

import { useEffect, useState } from 'react';
import type { ApexOptions } from 'apexcharts';

interface ApexChartWrapperProps {
  options: ApexOptions;
  series: any;
  type: any;
  height?: number | string;
}

export default function ApexChartWrapper({ options, series, type, height = 320 }: ApexChartWrapperProps) {
  const [Chart, setChart] = useState<any>(null);

  useEffect(() => {
    // Only import ApexCharts on the client side after component mount
    const loadChart = async () => {
      const { default: ApexChart } = await import('react-apexcharts');
      setChart(() => ApexChart);
    };
    
    loadChart();
  }, []);

  if (!Chart) {
    return (
      <div className="flex items-center justify-center h-full">
        <span className="text-sm text-gray-500">Loading chart...</span>
      </div>
    );
  }

  return <Chart options={options} series={series} type={type} height={height} />;
} 