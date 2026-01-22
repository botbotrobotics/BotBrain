'use client';

import { AlertTriangle, Map, Loader2 } from 'lucide-react';

interface MissionMapWarningBannerProps {
  missionMapName: string;
  currentMapName: string | null;
  onSwitchMap: () => Promise<void>;
  isLoading?: boolean;
  className?: string;
}

// Helper to format map name for display
function formatMapName(name: string): string {
  // Remove path prefix and .db extension, replace underscores with spaces
  const fileName = name.split('/').pop() || name;
  return fileName.replace(/\.db$/i, '').replace(/_/g, ' ');
}

export function MissionMapWarningBanner({
  missionMapName,
  currentMapName,
  onSwitchMap,
  isLoading = false,
  className = '',
}: MissionMapWarningBannerProps) {
  return (
    <div className={`flex items-center gap-3 p-3 bg-amber-50 dark:bg-amber-900/20 border border-amber-200 dark:border-amber-800 rounded-lg ${className}`}>
      <AlertTriangle className="w-5 h-5 text-amber-600 dark:text-amber-400 flex-shrink-0" />
      <div className="flex-1 min-w-0">
        <p className="text-sm font-medium text-amber-800 dark:text-amber-200">
          Map Mismatch
        </p>
        <p className="text-xs text-amber-700 dark:text-amber-300 mt-0.5 truncate">
          Mission created on <strong>{formatMapName(missionMapName)}</strong>
          {currentMapName ? (
            <>, currently <strong>{formatMapName(currentMapName)}</strong> is loaded</>
          ) : (
            <>, but no map is currently loaded</>
          )}
        </p>
      </div>
      <button
        onClick={onSwitchMap}
        disabled={isLoading}
        className="flex items-center gap-2 px-3 py-1.5 bg-amber-500 hover:bg-amber-600 disabled:bg-amber-400 text-white text-sm font-medium rounded-lg transition-colors flex-shrink-0"
      >
        {isLoading ? (
          <Loader2 className="w-4 h-4 animate-spin" />
        ) : (
          <Map className="w-4 h-4" />
        )}
        <span className="hidden sm:inline">Switch Map</span>
      </button>
    </div>
  );
}
