'use client';

import { Clock, Image, Sparkles, ChevronDown, X, Trash2, RefreshCw, Settings } from 'lucide-react';
import { useState, useEffect, useRef, useCallback } from 'react';
import { createPortal } from 'react-dom';
import { cn } from '@/utils/cn';

export type TimeFilter = 'all' | '1h' | '6h' | '24h' | '7d';
export type ImageFilter = 'all' | 'with-images' | 'without-images';
export type ConfidenceFilter = 'all' | 'high' | 'medium' | 'low';

interface YoloFilterBarProps {
  timeFilter: TimeFilter;
  setTimeFilter: (filter: TimeFilter) => void;
  imageFilter: ImageFilter;
  setImageFilter: (filter: ImageFilter) => void;
  confidenceFilter: ConfidenceFilter;
  setConfidenceFilter: (filter: ConfidenceFilter) => void;
  totalCount: number;
  filteredCount: number;
  onClearFilters: () => void;
  onDeleteAll?: () => void;
  realtimeEnabled?: boolean;
  onRealtimeToggle?: (enabled: boolean) => void;
  onOpenSettings?: () => void;
}

const timeOptions: { value: TimeFilter; label: string }[] = [
  { value: 'all', label: 'All time' },
  { value: '1h', label: 'Last hour' },
  { value: '6h', label: 'Last 6 hours' },
  { value: '24h', label: 'Last 24 hours' },
  { value: '7d', label: 'Last 7 days' },
];

const imageOptions: { value: ImageFilter; label: string }[] = [
  { value: 'all', label: 'All' },
  { value: 'with-images', label: 'With images' },
  { value: 'without-images', label: 'Without images' },
];

const confidenceOptions: { value: ConfidenceFilter; label: string; description: string }[] = [
  { value: 'all', label: 'All', description: 'Any confidence' },
  { value: 'high', label: 'High', description: '≥ 90%' },
  { value: 'medium', label: 'Medium', description: '70-89%' },
  { value: 'low', label: 'Low', description: '< 70%' },
];

interface DropdownButtonProps {
  icon: React.ReactNode;
  label: string;
  value: string;
  options: { value: string; label: string; description?: string }[];
  onChange: (value: any) => void;
  isActive: boolean;
}

function DropdownButton({ icon, label, value, options, onChange, isActive }: DropdownButtonProps) {
  const [isOpen, setIsOpen] = useState(false);
  const [dropdownPosition, setDropdownPosition] = useState({ top: 0, left: 0, width: 0 });
  const buttonRef = useRef<HTMLButtonElement>(null);
  const dropdownRef = useRef<HTMLDivElement>(null);
  const [mounted, setMounted] = useState(false);
  
  const selectedOption = options.find(opt => opt.value === value);

  useEffect(() => {
    setMounted(true);
    return () => setMounted(false);
  }, []);

  useEffect(() => {
    if (isOpen && buttonRef.current) {
      const rect = buttonRef.current.getBoundingClientRect();
      setDropdownPosition({
        top: rect.bottom + 4,
        left: rect.left,
        width: Math.max(rect.width, 150)
      });
    }
  }, [isOpen]);

  // Handle option selection
  const handleOptionClick = useCallback((optionValue: string) => {
    console.log('Option selected:', optionValue);
    onChange(optionValue);
    setIsOpen(false);
  }, [onChange]);

  // Close dropdown when clicking outside
  useEffect(() => {
    if (!isOpen) return;
    
    const handleClickOutside = (e: MouseEvent) => {
      const target = e.target as HTMLElement;
      
      // Don't close if clicking the button itself
      if (buttonRef.current?.contains(target)) {
        return;
      }
      
      // Don't close if clicking inside the dropdown
      if (dropdownRef.current?.contains(target)) {
        return;
      }
      
      setIsOpen(false);
    };

    // Small delay to prevent immediate close on open
    const timer = setTimeout(() => {
      document.addEventListener('mousedown', handleClickOutside);
    }, 10);

    return () => {
      clearTimeout(timer);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  return (
    <div className="relative">
      <button
        ref={buttonRef}
        type="button"
        onClick={() => {
          setIsOpen(!isOpen);
        }}
        className={cn(
          "flex items-center gap-2 px-3 py-1.5 text-sm rounded-md transition-all duration-200",
          "border border-gray-200 dark:border-gray-700",
          isActive
            ? "bg-purple-50 dark:bg-purple-900/20 text-purple-700 dark:text-purple-400 border-purple-300 dark:border-purple-700"
            : "bg-white dark:bg-botbot-darker hover:bg-gray-50 dark:hover:bg-botbot-dark text-gray-700 dark:text-gray-300"
        )}
      >
        {icon}
        <span className="font-medium">{label}</span>
        <span className="text-xs opacity-70">
          {selectedOption?.label}
        </span>
        <ChevronDown className={cn(
          "w-3.5 h-3.5 transition-transform duration-200",
          isOpen && "rotate-180"
        )} />
      </button>

      {isOpen && mounted && createPortal(
        <div 
          ref={dropdownRef}
          className="fixed bg-white dark:bg-botbot-darker rounded-lg shadow-xl border border-gray-200 dark:border-gray-700 overflow-hidden"
          style={{
            zIndex: 999999,
            top: `${dropdownPosition.top}px`,
            left: `${dropdownPosition.left}px`,
            minWidth: `${dropdownPosition.width}px`,
            pointerEvents: 'auto'
          }}
          onMouseDown={(e) => {
            // Prevent any default behavior
            e.stopPropagation();
          }}
        >
          {options.map((option) => (
            <div
              key={option.value}
              onClick={(e) => {
                e.stopPropagation();
                e.preventDefault();
                handleOptionClick(option.value);
              }}
              onMouseDown={(e) => {
                // Prevent default to avoid focus issues
                e.preventDefault();
                e.stopPropagation();
              }}
              className={cn(
                "w-full px-3 py-2 text-left text-sm transition-colors cursor-pointer",
                "hover:bg-gray-50 dark:hover:bg-botbot-dark",
                value === option.value && "bg-purple-50 dark:bg-purple-900/20 text-purple-700 dark:text-purple-400"
              )}
              role="button"
              tabIndex={0}
            >
              <div className="font-medium pointer-events-none">{option.label}</div>
              {option.description && (
                <div className="text-xs opacity-60 mt-0.5 pointer-events-none">{option.description}</div>
              )}
            </div>
          ))}
        </div>,
        document.body
      )}
    </div>
  );
}

export default function YoloFilterBar({
  timeFilter,
  setTimeFilter,
  imageFilter,
  setImageFilter,
  confidenceFilter,
  setConfidenceFilter,
  totalCount,
  filteredCount,
  onClearFilters,
  onDeleteAll,
  realtimeEnabled = false,
  onRealtimeToggle,
  onOpenSettings,
}: YoloFilterBarProps) {
  const hasActiveFilters = timeFilter !== 'all' || imageFilter !== 'all' || confidenceFilter !== 'all';
  
  return (
    <div className="flex items-center justify-between gap-4 p-3 bg-white/50 dark:bg-botbot-darker/30 backdrop-blur-sm rounded-lg border border-gray-200 dark:border-gray-700">
      <div className="flex items-center gap-2 flex-wrap">
        <DropdownButton
          icon={<Clock className="w-3.5 h-3.5" />}
          label="Time"
          value={timeFilter}
          options={timeOptions}
          onChange={setTimeFilter}
          isActive={timeFilter !== 'all'}
        />
        
        <DropdownButton
          icon={<Image className="w-3.5 h-3.5" />}
          label="Images"
          value={imageFilter}
          options={imageOptions}
          onChange={setImageFilter}
          isActive={imageFilter !== 'all'}
        />
        
        <DropdownButton
          icon={<Sparkles className="w-3.5 h-3.5" />}
          label="Confidence"
          value={confidenceFilter}
          options={confidenceOptions}
          onChange={setConfidenceFilter}
          isActive={confidenceFilter !== 'all'}
        />

        {hasActiveFilters && (
          <button
            onClick={onClearFilters}
            className="flex items-center gap-1.5 px-3 py-1.5 text-sm text-gray-600 dark:text-gray-400 hover:text-gray-900 dark:hover:text-white transition-colors"
          >
            <X className="w-3.5 h-3.5" />
            Clear filters
          </button>
        )}

        {onRealtimeToggle && (
          <div className="flex items-center gap-2 ml-2 pl-2 border-l border-gray-300 dark:border-gray-600">
            <button
              onClick={() => onRealtimeToggle(!realtimeEnabled)}
              className={cn(
                "flex items-center gap-2 px-3 py-1.5 text-sm rounded-md transition-all duration-200",
                "border",
                realtimeEnabled
                  ? "bg-green-50 dark:bg-green-900/20 text-green-700 dark:text-green-400 border-green-300 dark:border-green-700"
                  : "bg-white dark:bg-botbot-darker hover:bg-gray-50 dark:hover:bg-botbot-dark text-gray-700 dark:text-gray-300 border-gray-200 dark:border-gray-700"
              )}
              title={realtimeEnabled ? "Disable auto-refresh" : "Enable auto-refresh every 2 seconds"}
            >
              <RefreshCw className={cn(
                "w-3.5 h-3.5",
                realtimeEnabled && "animate-spin"
              )} />
              <span className="font-medium">Realtime</span>
              {realtimeEnabled && <span className="text-xs opacity-70">2s</span>}
            </button>
          </div>
        )}

        {onOpenSettings && (
          <div className="flex items-center gap-2 ml-2 pl-2 border-l border-gray-300 dark:border-gray-600">
            <button
              onClick={onOpenSettings}
              className={cn(
                "flex items-center gap-2 px-3 py-1.5 text-sm rounded-md transition-all duration-200",
                "border border-gray-200 dark:border-gray-700",
                "bg-white dark:bg-botbot-darker hover:bg-gray-50 dark:hover:bg-botbot-dark text-gray-700 dark:text-gray-300"
              )}
              title="Upload settings"
            >
              <Settings className="w-3.5 h-3.5" />
              <span className="font-medium">Settings</span>
            </button>
          </div>
        )}
      </div>

      <div className="flex items-center gap-3">
        <div className="flex items-center gap-2 text-sm text-gray-600 dark:text-gray-400">
          {filteredCount !== totalCount && (
            <>
              <span className="font-medium text-purple-600 dark:text-purple-400">
                {filteredCount}
              </span>
              <span>of</span>
            </>
          )}
          <span className="font-medium">{totalCount}</span>
          <span>detection{totalCount !== 1 ? 's' : ''}</span>
        </div>
        
        {onDeleteAll && filteredCount > 0 && (
          <button
            onClick={onDeleteAll}
            className="flex items-center gap-1.5 px-3 py-1.5 text-sm bg-red-500/10 text-red-600 dark:text-red-400 hover:bg-red-500/20 rounded-md transition-colors font-medium"
            title={`Delete ${filteredCount} detection${filteredCount !== 1 ? 's' : ''}`}
          >
            <Trash2 className="w-3.5 h-3.5" />
            <span>Delete {filteredCount !== totalCount && `${filteredCount}`}</span>
          </button>
        )}
      </div>
    </div>
  );
}