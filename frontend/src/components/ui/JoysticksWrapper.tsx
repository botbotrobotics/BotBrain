'use client';

import dynamic from 'next/dynamic';

interface JoysticksWrapperProps {
  enabled: boolean;
}

// Dynamically import Joysticks with SSR disabled to avoid nipplejs window errors
const Joysticks = dynamic(
  () => import('./joysticks').then(mod => mod.Joysticks),
  { 
    ssr: false,
    loading: () => null // No loading state needed for this component
  }
);

export function JoysticksWrapper({ enabled }: JoysticksWrapperProps) {
  return <Joysticks enabled={enabled} />;
} 