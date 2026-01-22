'use client';

import { useRef, useEffect } from 'react';
import { useDashboard } from '@/contexts/DashboardContext';
import { CameraWidget } from '../widgets/CameraWidget';
import { GaugeWidget } from '../widgets/GaugeWidget';
import { SidewaysGaugeWidget } from '../widgets/SidewaysGaugeWidget';
import { Visualizer3DWidget } from '../widgets/Visualizer3DWidget';
import { InfoWidget } from '../widgets/InfoWidget';
import { ChatWidget } from '../widgets/ChatWidget';
import { ButtonWidget } from '../widgets/ButtonWidget';
import { JoystickWidgetWrapper } from '../widgets/JoystickWidgetWrapper';
import { WidgetSelector } from './WidgetSelector';
import { useLanguage } from '@/contexts/LanguageContext';
import { ButtonGroupWidget } from '../widgets/ButtonGroupWidget';
import { AudioWidget } from '../widgets/AudioWidget';
import { MicrophoneWidget } from '../widgets/MicrophoneWidget';
import { MapWidget } from '../widgets/MapWidget';
import { AIStreamWidget } from '../widgets/AIStreamWidget';
import { RecentDetectionsWidget } from '../widgets/RecentDetectionsWidget';
import { TTSPresetsWidget } from '../widgets/TTSPresetsWidget';
import { MapsManagementWidget } from '../widgets/MapsManagementWidget';
import { SoundClipsWidget } from '../widgets/SoundClipsWidget';
import { RecorderWidget } from '../widgets/RecorderWidget';
import { DeliveryWidget } from '../widgets/DeliveryWidget';
import { MissionsWidget } from '../widgets/MissionsWidget';

export function Dashboard() {
  const { widgets, removeWidget } = useDashboard();
  const dashboardRef = useRef<HTMLDivElement>(null);
  const { t } = useLanguage();

  useEffect(() => {
    // Set page title
    document.title = 'My UI - BotBot';
  }, []);

  useEffect(() => {
    // Ensure the dashboard container has correct position for absolute positioning
    if (dashboardRef.current) {
      const computedStyle = window.getComputedStyle(dashboardRef.current);
      if (computedStyle.position === 'static') {
        dashboardRef.current.style.position = 'relative';
      }
    }

    // Set min-height to ensure there's space for widgets
    const updateMinHeight = () => {
      if (dashboardRef.current) {
        dashboardRef.current.style.minHeight = '500px';
      }
    };

    updateMinHeight();
    window.addEventListener('resize', updateMinHeight);

    return () => {
      window.removeEventListener('resize', updateMinHeight);
    };
  }, []);


  const renderWidget = (widget: any) => {
    const { id, type, position, size, props } = widget;
    const widgetProps = {
      id,
      initialPosition: position,
      initialSize: size,
      props,
      onRemove: removeWidget,
    };

    switch (type) {
      case 'camera':
        return <CameraWidget key={id} {...widgetProps} />;
      case 'gauge':
        return <GaugeWidget key={id} {...widgetProps} />;
      case 'sidewaysgauge':
        return <SidewaysGaugeWidget key={id} {...widgetProps} />;
      case 'visualization3d':
        return <Visualizer3DWidget key={id} {...widgetProps} />;
      case 'info':
        return <InfoWidget key={id} {...widgetProps} />;
      case 'chat':
        return <ChatWidget key={id} {...widgetProps} />;
      case 'button':
        return <ButtonWidget key={id} {...widgetProps} />;
      case 'buttonGroup':
        return <ButtonGroupWidget key={id} {...widgetProps} />;
      case 'joystick':
        return <JoystickWidgetWrapper key={id} {...widgetProps} />;
      case 'audio':
        return <AudioWidget key={id} {...widgetProps} />;
      case 'microphone':
        return <MicrophoneWidget key={id} {...widgetProps} />;
      case 'map':
        return <MapWidget key={id} {...widgetProps} />;
      case 'aiStream':
        return <AIStreamWidget key={id} {...widgetProps} />;
      case 'recentDetections':
        return <RecentDetectionsWidget key={id} {...widgetProps} />;
      case 'ttsPresets':
        return <TTSPresetsWidget key={id} {...widgetProps} />;
      case 'mapsManagement':
        return <MapsManagementWidget key={id} {...widgetProps} />;
      case 'soundClips':
        return <SoundClipsWidget key={id} {...widgetProps} />;
      case 'recorder':
        return <RecorderWidget key={id} {...widgetProps} />;
      case 'delivery':
        return <DeliveryWidget key={id} {...widgetProps} />;
      case 'missions':
        return <MissionsWidget key={id} {...widgetProps} />;
      default:
        return null;
    }
  };

  return (
    <div className="w-full flex flex-col h-full p-2">
      <div className="pb-2 border-gray-200 dark:border-gray-800 flex items-center justify-start flex-wrap gap-2">
        <WidgetSelector />
      </div>

      <div
        ref={dashboardRef}
        data-dashboard-container
        className="flex-1 relative overflow-hidden bg-gray-50 dark:bg-botbot-darkest rounded-xl"
        style={{ height: '100%' }}
      >
        {widgets.length === 0 ? (
          <div className="absolute inset-0 flex flex-col items-center justify-center text-gray-400 dark:text-gray-600">
            <p className="text-2xl mb-4">{t('myUI', 'emptyDashboard')}</p>
            <p className="text-sm">{t('myUI', 'clickAddWidget')}</p>
          </div>
        ) : (
          widgets.map(renderWidget)
        )}
      </div>
    </div>
  );
}
