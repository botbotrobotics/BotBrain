'use client';

import { useState, useEffect, useMemo, useCallback } from 'react';
import { createPortal } from 'react-dom';
import { X, Search, Loader2, Check, AlertCircle, RefreshCw } from 'lucide-react';
import { cn } from '@/utils/cn';
import { useYoloUploaderConfig } from '@/hooks/ros/useYoloUploaderConfig';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';

interface YoloSettingsDialogProps {
  isOpen: boolean;
  onClose: () => void;
}

// YOLO COCO classes organized by category
const YOLO_CLASSES: Record<string, { id: number; name: string }[]> = {
  'People': [
    { id: 0, name: 'person' },
  ],
  'Vehicles': [
    { id: 1, name: 'bicycle' },
    { id: 2, name: 'car' },
    { id: 3, name: 'motorcycle' },
    { id: 4, name: 'airplane' },
    { id: 5, name: 'bus' },
    { id: 6, name: 'train' },
    { id: 7, name: 'truck' },
    { id: 8, name: 'boat' },
  ],
  'Traffic': [
    { id: 9, name: 'traffic light' },
    { id: 10, name: 'fire hydrant' },
    { id: 11, name: 'stop sign' },
    { id: 12, name: 'parking meter' },
    { id: 13, name: 'bench' },
  ],
  'Animals': [
    { id: 14, name: 'bird' },
    { id: 15, name: 'cat' },
    { id: 16, name: 'dog' },
    { id: 17, name: 'horse' },
    { id: 18, name: 'sheep' },
    { id: 19, name: 'cow' },
    { id: 20, name: 'elephant' },
    { id: 21, name: 'bear' },
    { id: 22, name: 'zebra' },
    { id: 23, name: 'giraffe' },
  ],
  'Accessories': [
    { id: 24, name: 'backpack' },
    { id: 25, name: 'umbrella' },
    { id: 26, name: 'handbag' },
    { id: 27, name: 'tie' },
    { id: 28, name: 'suitcase' },
  ],
  'Sports': [
    { id: 29, name: 'frisbee' },
    { id: 30, name: 'skis' },
    { id: 31, name: 'snowboard' },
    { id: 32, name: 'sports ball' },
    { id: 33, name: 'kite' },
    { id: 34, name: 'baseball bat' },
    { id: 35, name: 'baseball glove' },
    { id: 36, name: 'skateboard' },
    { id: 37, name: 'surfboard' },
    { id: 38, name: 'tennis racket' },
  ],
  'Kitchen': [
    { id: 39, name: 'bottle' },
    { id: 40, name: 'wine glass' },
    { id: 41, name: 'cup' },
    { id: 42, name: 'fork' },
    { id: 43, name: 'knife' },
    { id: 44, name: 'spoon' },
    { id: 45, name: 'bowl' },
  ],
  'Food': [
    { id: 46, name: 'banana' },
    { id: 47, name: 'apple' },
    { id: 48, name: 'sandwich' },
    { id: 49, name: 'orange' },
    { id: 50, name: 'broccoli' },
    { id: 51, name: 'carrot' },
    { id: 52, name: 'hot dog' },
    { id: 53, name: 'pizza' },
    { id: 54, name: 'donut' },
    { id: 55, name: 'cake' },
  ],
  'Furniture': [
    { id: 56, name: 'chair' },
    { id: 57, name: 'couch' },
    { id: 58, name: 'potted plant' },
    { id: 59, name: 'bed' },
    { id: 60, name: 'dining table' },
    { id: 61, name: 'toilet' },
  ],
  'Electronics': [
    { id: 62, name: 'tv' },
    { id: 63, name: 'laptop' },
    { id: 64, name: 'mouse' },
    { id: 65, name: 'remote' },
    { id: 66, name: 'keyboard' },
    { id: 67, name: 'cell phone' },
  ],
  'Appliances': [
    { id: 68, name: 'microwave' },
    { id: 69, name: 'oven' },
    { id: 70, name: 'toaster' },
    { id: 71, name: 'sink' },
    { id: 72, name: 'refrigerator' },
  ],
  'Misc': [
    { id: 73, name: 'book' },
    { id: 74, name: 'clock' },
    { id: 75, name: 'vase' },
    { id: 76, name: 'scissors' },
    { id: 77, name: 'teddy bear' },
    { id: 78, name: 'hair drier' },
    { id: 79, name: 'toothbrush' },
  ],
};

// Flatten all classes for searching
const ALL_CLASSES = Object.values(YOLO_CLASSES).flat();

export default function YoloSettingsDialog({ isOpen, onClose }: YoloSettingsDialogProps) {
  const { connection } = useRobotConnection();
  const { getParameters, applySettings, isLoading, isFetching, lastError, clearError } = useYoloUploaderConfig();

  const [minConfidence, setMinConfidence] = useState(0.5);
  const [selectedObjects, setSelectedObjects] = useState<Set<string>>(new Set());
  const [searchQuery, setSearchQuery] = useState('');
  const [success, setSuccess] = useState(false);
  const [mounted, setMounted] = useState(false);
  const [fetchError, setFetchError] = useState<string | null>(null);
  const [hasFetched, setHasFetched] = useState(false);

  useEffect(() => {
    setMounted(true);
    return () => setMounted(false);
  }, []);

  // Fetch current settings when dialog opens
  const fetchCurrentSettings = useCallback(async () => {
    if (!connection.online) return;

    setFetchError(null);
    try {
      const config = await getParameters();
      setMinConfidence(config.minConfidence);

      // Parse object filter string into Set
      if (config.objectFilter && config.objectFilter.trim()) {
        const objects = config.objectFilter.split(',').map(s => s.trim()).filter(Boolean);
        // Match objects by name or ID
        const matchedNames = new Set<string>();
        objects.forEach(obj => {
          // Try to find by name first
          const byName = ALL_CLASSES.find(c => c.name.toLowerCase() === obj.toLowerCase());
          if (byName) {
            matchedNames.add(byName.name);
          } else {
            // Try by ID
            const id = parseInt(obj, 10);
            if (!isNaN(id)) {
              const byId = ALL_CLASSES.find(c => c.id === id);
              if (byId) {
                matchedNames.add(byId.name);
              }
            }
          }
        });
        setSelectedObjects(matchedNames);
      } else {
        setSelectedObjects(new Set());
      }
      setHasFetched(true);
    } catch (error) {
      setFetchError(error instanceof Error ? error.message : 'Failed to fetch settings');
    }
  }, [connection.online, getParameters]);

  // Reset and fetch when dialog opens
  useEffect(() => {
    if (isOpen) {
      setSuccess(false);
      clearError();
      setHasFetched(false);
      fetchCurrentSettings();
    }
  }, [isOpen, clearError, fetchCurrentSettings]);

  // Handle ESC key
  useEffect(() => {
    if (!isOpen) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, onClose]);

  // Filter classes based on search
  const filteredCategories = useMemo(() => {
    if (!searchQuery.trim()) return YOLO_CLASSES;

    const query = searchQuery.toLowerCase();
    const result: typeof YOLO_CLASSES = {};

    Object.entries(YOLO_CLASSES).forEach(([category, items]) => {
      const filtered = items.filter(
        item => item.name.toLowerCase().includes(query) || item.id.toString().includes(query)
      );
      if (filtered.length > 0) {
        result[category] = filtered;
      }
    });

    return result;
  }, [searchQuery]);

  const toggleObject = useCallback((name: string) => {
    setSelectedObjects(prev => {
      const next = new Set(prev);
      if (next.has(name)) {
        next.delete(name);
      } else {
        next.add(name);
      }
      return next;
    });
  }, []);

  const selectAll = useCallback(() => {
    setSelectedObjects(new Set(ALL_CLASSES.map(c => c.name)));
  }, []);

  const clearAll = useCallback(() => {
    setSelectedObjects(new Set());
  }, []);

  const handleApply = async () => {
    try {
      // Build the filter string: either names or IDs, comma-separated
      const filterString = Array.from(selectedObjects).join(',');

      await applySettings(minConfidence, filterString);
      setSuccess(true);

      // Close after a brief delay to show success
      setTimeout(() => {
        onClose();
      }, 1000);
    } catch (error) {
      // Error is already handled by the hook
    }
  };

  // Get confidence color based on value
  const getConfidenceColor = (value: number) => {
    if (value >= 0.9) return 'bg-green-500';
    if (value >= 0.7) return 'bg-yellow-500';
    return 'bg-orange-500';
  };

  if (!isOpen || !mounted) return null;

  const isConnected = connection.online;

  return createPortal(
    <div
      className="fixed inset-0 z-50 flex items-center justify-center bg-black/50 backdrop-blur-sm"
      onClick={(e) => {
        if (e.target === e.currentTarget) onClose();
      }}
    >
      <div
        className="bg-white dark:bg-botbot-darker rounded-xl shadow-2xl w-full max-w-2xl max-h-[85vh] flex flex-col overflow-hidden"
        onClick={(e) => e.stopPropagation()}
      >
        {/* Header */}
        <div className="flex items-center justify-between px-6 py-4 border-b border-gray-200 dark:border-gray-700">
          <div>
            <h2 className="text-xl font-semibold text-gray-900 dark:text-white">
              Detection Upload Settings
            </h2>
            <p className="text-sm text-gray-500 dark:text-gray-400 mt-0.5">
              Configure which detections are saved to the database
            </p>
          </div>
          <button
            onClick={onClose}
            className="p-2 hover:bg-gray-100 dark:hover:bg-botbot-dark rounded-lg transition-colors"
          >
            <X className="w-5 h-5 text-gray-500 dark:text-gray-400" />
          </button>
        </div>

        {/* Content */}
        <div className="flex-1 overflow-y-auto p-6 space-y-6">
          {/* Loading State */}
          {isFetching && (
            <div className="flex items-center justify-center py-8">
              <div className="flex items-center gap-3 text-gray-500 dark:text-gray-400">
                <Loader2 className="w-5 h-5 animate-spin" />
                <span>Loading current settings...</span>
              </div>
            </div>
          )}

          {/* Fetch Error */}
          {fetchError && !isFetching && (
            <div className="flex items-center justify-between gap-3 p-4 bg-orange-50 dark:bg-orange-900/20 border border-orange-200 dark:border-orange-800 rounded-lg">
              <div className="flex items-center gap-3">
                <AlertCircle className="w-5 h-5 text-orange-600 dark:text-orange-400 flex-shrink-0" />
                <p className="text-sm text-orange-700 dark:text-orange-300">
                  Could not load current settings: {fetchError}
                </p>
              </div>
              <button
                onClick={fetchCurrentSettings}
                className="flex items-center gap-1.5 px-3 py-1.5 text-sm bg-orange-100 dark:bg-orange-900/30 text-orange-700 dark:text-orange-300 rounded-md hover:bg-orange-200 dark:hover:bg-orange-900/50 transition-colors"
              >
                <RefreshCw className="w-3.5 h-3.5" />
                Retry
              </button>
            </div>
          )}

          {/* Connection Warning */}
          {!isConnected && !isFetching && (
            <div className="flex items-center gap-3 p-4 bg-yellow-50 dark:bg-yellow-900/20 border border-yellow-200 dark:border-yellow-800 rounded-lg">
              <AlertCircle className="w-5 h-5 text-yellow-600 dark:text-yellow-400 flex-shrink-0" />
              <p className="text-sm text-yellow-700 dark:text-yellow-300">
                Robot is not connected. Connect to load and apply settings.
              </p>
            </div>
          )}

          {/* Form Content - hidden while fetching */}
          {!isFetching && (
            <>
              {/* Minimum Confidence Section */}
              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <label className="text-sm font-medium text-gray-700 dark:text-gray-300">
                    Minimum Confidence
                  </label>
              <span className={cn(
                "px-2.5 py-1 rounded-full text-sm font-medium text-white",
                getConfidenceColor(minConfidence)
              )}>
                {Math.round(minConfidence * 100)}%
              </span>
            </div>
            <div className="space-y-2">
              <input
                type="range"
                min="0"
                max="1"
                step="0.01"
                value={minConfidence}
                onChange={(e) => setMinConfidence(parseFloat(e.target.value))}
                className="w-full h-2 bg-gray-200 dark:bg-botbot-dark rounded-lg appearance-none cursor-pointer accent-purple-600"
              />
              <div className="flex justify-between text-xs text-gray-500 dark:text-gray-400">
                <span>0%</span>
                <span>50%</span>
                <span>100%</span>
              </div>
            </div>
            <p className="text-xs text-gray-500 dark:text-gray-400">
              Only detections with confidence above this threshold will be saved
            </p>
          </div>

          {/* Object Filter Section */}
          <div className="space-y-3">
            <div className="flex items-center justify-between">
              <label className="text-sm font-medium text-gray-700 dark:text-gray-300">
                Object Filter
              </label>
              <div className="flex items-center gap-2">
                <span className="text-xs text-gray-500 dark:text-gray-400">
                  {selectedObjects.size === 0 ? 'All objects' : `${selectedObjects.size} selected`}
                </span>
                <button
                  onClick={selectAll}
                  className="text-xs px-2 py-1 text-purple-600 dark:text-purple-400 hover:bg-purple-50 dark:hover:bg-purple-900/20 rounded transition-colors"
                >
                  Select All
                </button>
                <button
                  onClick={clearAll}
                  className="text-xs px-2 py-1 text-gray-600 dark:text-gray-400 hover:bg-gray-100 dark:hover:bg-botbot-dark rounded transition-colors"
                >
                  Clear
                </button>
              </div>
            </div>

            {/* Search */}
            <div className="relative">
              <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-gray-400" />
              <input
                type="text"
                placeholder="Search objects..."
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                className="w-full pl-10 pr-4 py-2 text-sm border border-gray-200 dark:border-gray-700 rounded-lg bg-white dark:bg-botbot-dark focus:outline-none focus:ring-2 focus:ring-purple-500 dark:focus:ring-purple-400"
              />
            </div>

            {/* Selected Objects Chips */}
            {selectedObjects.size > 0 && (
              <div className="flex flex-wrap gap-2">
                {Array.from(selectedObjects).slice(0, 10).map(name => (
                  <span
                    key={name}
                    className="inline-flex items-center gap-1 px-2 py-1 bg-purple-100 dark:bg-purple-900/30 text-purple-700 dark:text-purple-300 rounded-full text-xs"
                  >
                    {name}
                    <button
                      onClick={() => toggleObject(name)}
                      className="hover:text-purple-900 dark:hover:text-purple-100"
                    >
                      <X className="w-3 h-3" />
                    </button>
                  </span>
                ))}
                {selectedObjects.size > 10 && (
                  <span className="px-2 py-1 text-xs text-gray-500 dark:text-gray-400">
                    +{selectedObjects.size - 10} more
                  </span>
                )}
              </div>
            )}

            {/* Categories Grid */}
            <div className="max-h-64 overflow-y-auto border border-gray-200 dark:border-gray-700 rounded-lg">
              {Object.entries(filteredCategories).map(([category, items]) => (
                <div key={category} className="border-b border-gray-100 dark:border-gray-800 last:border-0">
                  <div className="px-3 py-2 bg-gray-50 dark:bg-botbot-dark/50 text-xs font-medium text-gray-600 dark:text-gray-400 uppercase tracking-wide">
                    {category}
                  </div>
                  <div className="p-2 flex flex-wrap gap-1.5">
                    {items.map(item => (
                      <button
                        key={item.id}
                        onClick={() => toggleObject(item.name)}
                        className={cn(
                          "px-2.5 py-1 text-xs rounded-md transition-colors",
                          selectedObjects.has(item.name)
                            ? "bg-purple-600 text-white"
                            : "bg-gray-100 dark:bg-botbot-dark text-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-botbot-darker"
                        )}
                      >
                        {item.name}
                      </button>
                    ))}
                  </div>
                </div>
              ))}
              {Object.keys(filteredCategories).length === 0 && (
                <div className="p-4 text-center text-sm text-gray-500 dark:text-gray-400">
                  No objects match your search
                </div>
              )}
            </div>

            <p className="text-xs text-gray-500 dark:text-gray-400">
              {selectedObjects.size === 0
                ? 'All detected objects will be saved. Select specific objects to filter.'
                : 'Only selected objects will be saved to the database'}
            </p>
          </div>
            </>
          )}
        </div>

        {/* Footer */}
        <div className="px-6 py-4 border-t border-gray-200 dark:border-gray-700 bg-gray-50 dark:bg-botbot-darkest/50">
          {/* Error Message */}
          {lastError && (
            <div className="flex items-center gap-2 mb-3 p-3 bg-red-50 dark:bg-red-900/20 border border-red-200 dark:border-red-800 rounded-lg">
              <AlertCircle className="w-4 h-4 text-red-600 dark:text-red-400 flex-shrink-0" />
              <p className="text-sm text-red-700 dark:text-red-300">{lastError}</p>
            </div>
          )}

          {/* Success Message */}
          {success && (
            <div className="flex items-center gap-2 mb-3 p-3 bg-green-50 dark:bg-green-900/20 border border-green-200 dark:border-green-800 rounded-lg">
              <Check className="w-4 h-4 text-green-600 dark:text-green-400 flex-shrink-0" />
              <p className="text-sm text-green-700 dark:text-green-300">Settings applied successfully!</p>
            </div>
          )}

          <div className="flex items-center justify-end gap-3">
            <button
              onClick={onClose}
              disabled={isLoading}
              className="px-4 py-2 text-sm font-medium text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-botbot-dark rounded-lg transition-colors disabled:opacity-50"
            >
              Cancel
            </button>
            <button
              onClick={handleApply}
              disabled={isLoading || isFetching || !isConnected}
              className={cn(
                "flex items-center gap-2 px-4 py-2 text-sm font-medium text-white rounded-lg transition-colors",
                isLoading || isFetching || !isConnected
                  ? "bg-purple-400 cursor-not-allowed"
                  : "bg-purple-600 hover:bg-purple-700"
              )}
            >
              {isLoading ? (
                <>
                  <Loader2 className="w-4 h-4 animate-spin" />
                  Applying...
                </>
              ) : (
                'Apply Settings'
              )}
            </button>
          </div>
        </div>
      </div>
    </div>,
    document.body
  );
}
