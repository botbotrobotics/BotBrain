'use client';

import dynamic from 'next/dynamic';

interface MapEditorWrapperProps {
  className?: string;
}

// Dynamically import MapEditor with SSR disabled
const MapEditor = dynamic(
  () => import('./MapEditor').then(mod => mod.MapEditor),
  { 
    ssr: false,
    loading: () => (
      <div className="w-full h-full flex items-center justify-center">
        <div className="text-white">Loading Map Editor...</div>
      </div>
    )
  }
);

export function MapEditorWrapper({ className }: MapEditorWrapperProps) {
  return <MapEditor className={className} />;
} 