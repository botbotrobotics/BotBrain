'use client';

import { useState, useEffect } from 'react';
import { Palette, RotateCcw } from 'lucide-react';
import { generateTheme, applyTheme, resetTheme } from '@/utils/theme-generator';

interface ColorPickerProps {
  value: string | null;
  onChange: (color: string | null) => void;
  defaultColor?: string;
}

export default function ColorPicker({ value, onChange, defaultColor = '#821db7' }: ColorPickerProps) {
  const [inputValue, setInputValue] = useState(value || '');
  const [isOpen, setIsOpen] = useState(false);
  const [previewColor, setPreviewColor] = useState<string | null>(null);

  // Popular colors that work well as base theme colors
  const presetColors = [
    '#821db7', // Default purple
    '#3B82F6', // Blue
    '#10B981', // Emerald
    '#F59E0B', // Amber
    '#EF4444', // Red
    '#8B5CF6', // Violet
    '#EC4899', // Pink
    '#14B8A6', // Teal
    '#F97316', // Orange
    '#6366F1', // Indigo
    '#84CC16', // Lime
    '#06B6D4', // Cyan
  ];

  useEffect(() => {
    setInputValue(value || '');
  }, [value]);

  // Apply preview theme on hover
  useEffect(() => {
    if (previewColor) {
      const colors = generateTheme(previewColor);
      if (colors) {
        applyTheme(colors);
      }
    } else if (value) {
      // Restore the actual selected color
      const colors = generateTheme(value);
      if (colors) {
        applyTheme(colors);
      }
    } else {
      resetTheme();
    }
  }, [previewColor, value]);

  const handleColorChange = (color: string) => {
    setInputValue(color);
    onChange(color);
    setIsOpen(false);
    setPreviewColor(null);
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const newValue = e.target.value;
    setInputValue(newValue);
    
    // Validate hex color
    if (/^#[0-9A-F]{6}$/i.test(newValue)) {
      onChange(newValue);
    }
  };

  const handleReset = () => {
    onChange(null);
    setInputValue('');
    setIsOpen(false);
    setPreviewColor(null);
  };

  return (
    <div className="relative">
      <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-2">
        Theme Color
      </label>
      
      <div className="flex items-center space-x-2">
        {/* Color preview and picker button */}
        <button
          type="button"
          onClick={() => setIsOpen(!isOpen)}
          className="w-12 h-12 rounded-lg border-2 border-gray-300 dark:border-botbot-darker 
                     hover:border-primary dark:hover:border-botbot-accent transition-all duration-300
                     flex items-center justify-center overflow-hidden shadow-md hover:shadow-lg"
          style={{ 
            backgroundColor: value || defaultColor,
            transform: isOpen ? 'scale(1.05)' : 'scale(1)'
          }}
        >
          {!value && <Palette className="w-6 h-6 text-white" />}
        </button>

        {/* Hex input */}
        <input
          type="text"
          value={inputValue}
          onChange={handleInputChange}
          placeholder="#821db7"
          className="flex-1 p-2 border border-gray-300 dark:border-botbot-darker rounded-md 
                     focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent
                     bg-white dark:bg-botbot-dark text-gray-800 dark:text-gray-100
                     font-mono text-sm uppercase transition-all duration-200"
          maxLength={7}
        />

        {/* Reset button */}
        <button
          type="button"
          onClick={handleReset}
          className="p-2 text-gray-500 hover:text-gray-700 dark:text-gray-400 
                     dark:hover:text-gray-200 transition-all duration-200
                     hover:bg-gray-100 dark:hover:bg-gray-800 rounded-md"
          title="Reset to default"
        >
          <RotateCcw className="w-5 h-5" />
        </button>
      </div>

      {/* Color palette dropdown */}
      {isOpen && (
        <div className="absolute top-full left-0 mt-2 p-4 bg-white dark:bg-botbot-darker 
                        rounded-lg shadow-xl border border-gray-200 dark:border-botbot-dark z-50
                        animate-fadeIn">
          <p className="text-xs text-gray-600 dark:text-gray-400 mb-3 font-medium">
            Hover to preview • Click to apply
          </p>
          
          <div className="grid grid-cols-6 gap-3">
            {presetColors.map((color) => (
              <button
                key={color}
                type="button"
                onClick={() => handleColorChange(color)}
                onMouseEnter={() => setPreviewColor(color)}
                onMouseLeave={() => setPreviewColor(null)}
                className="w-12 h-12 rounded-lg hover:scale-110 transition-all duration-200
                           border-2 border-transparent hover:border-gray-300 dark:hover:border-gray-600
                           shadow-sm hover:shadow-md relative group"
                style={{ backgroundColor: color }}
                title={color}
              >
                {value === color && (
                  <div className="absolute inset-0 flex items-center justify-center">
                    <div className="w-3 h-3 bg-white rounded-full shadow-md"></div>
                  </div>
                )}
              </button>
            ))}
          </div>
          
          {/* Color input for custom colors */}
          <div className="mt-4 pt-4 border-t border-gray-200 dark:border-botbot-dark">
            <label className="text-xs text-gray-600 dark:text-gray-400 block mb-2">
              Custom color
            </label>
            <input
              type="color"
              value={value || defaultColor}
              onChange={(e) => handleColorChange(e.target.value.toUpperCase())}
              className="w-full h-12 cursor-pointer rounded-md border-2 border-gray-300 
                         dark:border-botbot-dark hover:border-primary dark:hover:border-botbot-accent
                         transition-all duration-200"
            />
          </div>
        </div>
      )}

      <p className="text-xs text-gray-500 dark:text-gray-400 mt-1">
        Choose a color to customize the entire app theme
      </p>
    </div>
  );
} 