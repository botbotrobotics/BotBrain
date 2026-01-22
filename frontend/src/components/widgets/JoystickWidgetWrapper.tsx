'use client';

import dynamic from 'next/dynamic';

interface JoystickWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  title?: string;
  props?: {
    title?: string;
    joystickSide?: 'left' | 'right';
  };
}

// Dynamically import JoystickWidget with SSR disabled to avoid nipplejs window errors
const JoystickWidget = dynamic(
  () => import('./JoystickWidget').then(mod => mod.JoystickWidget),
  { 
    ssr: false,
    loading: () => (
      <div className="w-full h-full flex items-center justify-center">
        <div className="text-gray-500">Loading Joystick...</div>
      </div>
    )
  }
);

export function JoystickWidgetWrapper(props: JoystickWidgetProps) {
  return <JoystickWidget {...props} />;
} 