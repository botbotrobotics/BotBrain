'use client';

import { useState, useEffect, useCallback } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import {
  Camera,
  Gauge,
  Box,
  Info,
  MessageSquare,
  Battery,
  Square,
  Gamepad,
  Volume2,
  Map,
  Brain,
  Sparkles,
  Mic,
  Package,
  Circle,
  X,
  Search,
  Plus,
  Route,
} from 'lucide-react';
import { WidgetType } from '@/contexts/DashboardContext';
import { useLanguage } from '@/contexts/LanguageContext';

interface AddWidgetModalProps {
  isOpen: boolean;
  onClose: () => void;
  onAddWidget: (type: WidgetType) => void;
}

interface WidgetOption {
  type: WidgetType;
  label: string;
  description: string;
  icon: React.ReactNode;
  category: 'visualization' | 'control' | 'media' | 'information' | 'ai';
}

export function AddWidgetModal({ isOpen, onClose, onAddWidget }: AddWidgetModalProps) {
  const { t } = useLanguage();
  const [searchQuery, setSearchQuery] = useState('');
  const [selectedCategory, setSelectedCategory] = useState<string>('all');

  // Handle ESC key
  const handleKeyDown = useCallback(
    (event: KeyboardEvent) => {
      if (event.key === 'Escape' && isOpen) {
        onClose();
      }
    },
    [isOpen, onClose]
  );

  useEffect(() => {
    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [handleKeyDown]);

  // Handle click outside
  const handleOverlayClick = (event: React.MouseEvent) => {
    if (event.target === event.currentTarget) {
      onClose();
    }
  };

  // Prevent body scroll when open
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = 'auto';
    }
    return () => {
      document.body.style.overflow = 'auto';
    };
  }, [isOpen]);

  // Pro-only widget types (excluded from OSS builds)
  const proOnlyWidgetTypes: WidgetType[] = ['aiStream', 'recentDetections'];

  const allWidgetOptions: WidgetOption[] = [
    // Visualization widgets
    {
      type: 'camera' as WidgetType,
      label: t('myUI', 'widgetCamera'),
      description: 'Live video feed from robot cameras',
      icon: <Camera className="w-6 h-6" />,
      category: 'visualization',
    },
    {
      type: 'visualization3d' as WidgetType,
      label: t('myUI', 'widget3DVisualization'),
      description: '3D robot model and environment',
      icon: <Box className="w-6 h-6" />,
      category: 'visualization',
    },
    {
      type: 'map' as WidgetType,
      label: t('myUI', 'widgetMapView'),
      description: 'Live map and navigation view',
      icon: <Map className="w-6 h-6" />,
      category: 'visualization',
    },
    // Control widgets
    {
      type: 'joystick' as WidgetType,
      label: t('myUI', 'widgetJoystick'),
      description: 'Virtual joystick for robot control',
      icon: <Gamepad className="w-6 h-6" />,
      category: 'control',
    },
    {
      type: 'button' as WidgetType,
      label: t('myUI', 'widgetButton'),
      description: 'Custom action button',
      icon: <Square className="w-6 h-6" />,
      category: 'control',
    },
    {
      type: 'buttonGroup' as WidgetType,
      label: t('myUI', 'widgetButtonGroup'),
      description: 'Multiple action buttons',
      icon: <Square className="w-6 h-6" />,
      category: 'control',
    },
    {
      type: 'delivery' as WidgetType,
      label: 'Delivery',
      description: 'Delivery management controls',
      icon: <Package className="w-6 h-6" />,
      category: 'control',
    },
    {
      type: 'missions' as WidgetType,
      label: 'Missions',
      description: 'Manage and execute robot missions',
      icon: <Route className="w-6 h-6" />,
      category: 'control',
    },
    // Media widgets
    {
      type: 'audio' as WidgetType,
      label: t('myUI', 'widgetAudioStream'),
      description: 'Audio streaming controls',
      icon: <Volume2 className="w-6 h-6" />,
      category: 'media',
    },
    {
      type: 'microphone' as WidgetType,
      label: 'Microphone',
      description: 'Voice input and recording',
      icon: <Mic className="w-6 h-6" />,
      category: 'media',
    },
    {
      type: 'ttsPresets' as WidgetType,
      label: 'TTS Presets',
      description: 'Text-to-speech presets',
      icon: <Volume2 className="w-6 h-6" />,
      category: 'media',
    },
    {
      type: 'soundClips' as WidgetType,
      label: 'Sound Clips',
      description: 'Play pre-recorded sounds',
      icon: <Volume2 className="w-6 h-6" />,
      category: 'media',
    },
    {
      type: 'recorder' as WidgetType,
      label: 'Recorder',
      description: 'Audio/video recording',
      icon: <Circle className="w-6 h-6" />,
      category: 'media',
    },
    // Information widgets
    {
      type: 'gauge' as WidgetType,
      label: t('myUI', 'widgetGauge'),
      description: 'Display numeric values',
      icon: <Gauge className="w-6 h-6" />,
      category: 'information',
    },
    {
      type: 'sidewaysgauge' as WidgetType,
      label: t('myUI', 'widgetSidewaysGauge'),
      description: 'Horizontal gauge display',
      icon: <Battery className="w-6 h-6" />,
      category: 'information',
    },
    {
      type: 'info' as WidgetType,
      label: t('myUI', 'widgetInformation'),
      description: 'Display robot information',
      icon: <Info className="w-6 h-6" />,
      category: 'information',
    },
    {
      type: 'mapsManagement' as WidgetType,
      label: 'Maps Management',
      description: 'Manage saved maps',
      icon: <Map className="w-6 h-6" />,
      category: 'information',
    },
    // AI widgets
    {
      type: 'chat' as WidgetType,
      label: t('myUI', 'widgetChat'),
      description: 'Chat interface',
      icon: <MessageSquare className="w-6 h-6" />,
      category: 'ai',
    },
    {
      type: 'aiStream' as WidgetType,
      label: 'AI Stream',
      description: 'AI processing stream',
      icon: <Brain className="w-6 h-6" />,
      category: 'ai',
    },
    {
      type: 'recentDetections' as WidgetType,
      label: 'Recent Detections',
      description: 'Object detection history',
      icon: <Sparkles className="w-6 h-6" />,
      category: 'ai',
    },
  ];

  // Filter out Pro-only widgets in OSS builds
  const widgetOptions = __PRO__
    ? allWidgetOptions
    : allWidgetOptions.filter(widget => !proOnlyWidgetTypes.includes(widget.type));

  const categories = [
    { id: 'all', label: 'All Widgets', icon: '🎯' },
    { id: 'visualization', label: 'Visualization', icon: '👁️' },
    { id: 'control', label: 'Control', icon: '🎮' },
    { id: 'media', label: 'Media', icon: '🎵' },
    { id: 'information', label: 'Information', icon: '📊' },
    { id: 'ai', label: 'AI & Smart', icon: '🤖' },
  ];

  const filteredWidgets = widgetOptions.filter(widget => {
    const matchesSearch = widget.label.toLowerCase().includes(searchQuery.toLowerCase()) ||
                          widget.description.toLowerCase().includes(searchQuery.toLowerCase());
    const matchesCategory = selectedCategory === 'all' || widget.category === selectedCategory;
    return matchesSearch && matchesCategory;
  });

  const handleAddWidget = (type: WidgetType) => {
    onAddWidget(type);
    setSearchQuery('');
    setSelectedCategory('all');
  };

  return (
    <AnimatePresence>
      {isOpen && (
        <motion.div
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          exit={{ opacity: 0 }}
          transition={{ duration: 0.2 }}
          className="fixed inset-0 bg-black/20 dark:bg-black/40 backdrop-blur-sm z-[90] flex items-center justify-center p-4"
          onClick={handleOverlayClick}
        >
          <motion.div
            initial={{ opacity: 0, scale: 0.95 }}
            animate={{ opacity: 1, scale: 1 }}
            exit={{ opacity: 0, scale: 0.95 }}
            transition={{
              type: 'spring',
              damping: 25,
              stiffness: 300,
            }}
            className="bg-white dark:bg-botbot-dark rounded-lg shadow-xl max-w-4xl w-full max-h-[80vh] flex flex-col"
            onClick={(e) => e.stopPropagation()}
          >
        {/* Header */}
        <div className="px-6 py-4 border-b border-gray-200 dark:border-gray-700">
          <div className="flex items-center justify-between">
            <h2 className="text-xl font-semibold text-gray-800 dark:text-white">
              Add Widget to Dashboard
            </h2>
            <button
              onClick={onClose}
              className="p-1 rounded-lg hover:bg-gray-100 dark:hover:bg-botbot-darker transition-colors"
            >
              <X className="w-5 h-5 text-gray-500 dark:text-gray-400" />
            </button>
          </div>

          {/* Search Bar */}
          <div className="mt-4 relative">
            <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 w-4 h-4 text-gray-400" />
            <input
              type="text"
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              placeholder="Search widgets..."
              className="w-full pl-10 pr-4 py-2 border border-gray-300 dark:border-gray-600 rounded-lg
                       bg-white dark:bg-botbot-darker text-gray-800 dark:text-white
                       placeholder-gray-400 dark:placeholder-gray-500
                       focus:outline-none focus:ring-2 focus:ring-violet-500 dark:focus:ring-botbot-purple"
            />
          </div>
        </div>

        {/* Category Tabs */}
        <div className="px-6 py-3 border-b border-gray-200 dark:border-gray-700">
          <div className="flex gap-2 overflow-x-auto pb-2">
            {categories.map(category => (
              <button
                key={category.id}
                onClick={() => setSelectedCategory(category.id)}
                className={`px-4 py-2 rounded-lg text-sm font-medium whitespace-nowrap transition-all
                          ${selectedCategory === category.id
                            ? 'bg-violet-600 dark:bg-botbot-purple text-white'
                            : 'bg-gray-100 dark:bg-botbot-darker text-gray-600 dark:text-gray-400 hover:bg-gray-200 dark:hover:bg-gray-700'
                          }`}
              >
                <span className="mr-2">{category.icon}</span>
                {category.label}
              </button>
            ))}
          </div>
        </div>

        {/* Widget Grid */}
        <div className="flex-1 overflow-y-auto p-6">
          {filteredWidgets.length === 0 ? (
            <div className="text-center py-12">
              <p className="text-gray-500 dark:text-gray-400">
                No widgets found matching your search.
              </p>
            </div>
          ) : (
            <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-4">
              {filteredWidgets.map((widget) => (
                <button
                  key={widget.type}
                  onClick={() => handleAddWidget(widget.type)}
                  className="group relative bg-gradient-to-br from-gray-50 to-gray-100 dark:from-botbot-darker dark:to-botbot-darkest
                           rounded-xl p-4 border border-gray-200 dark:border-gray-700
                           hover:from-white hover:to-gray-50 dark:hover:from-botbot-darkest dark:hover:to-botbot-darker
                           hover:border-violet-400 dark:hover:border-botbot-purple
                           hover:shadow-xl hover:-translate-y-1 transform
                           transition-all duration-300 text-left overflow-hidden"
                >
                  {/* Background gradient overlay on hover */}
                  <div className="absolute inset-0 bg-gradient-to-br from-violet-600/5 to-purple-600/5
                                dark:from-botbot-purple/5 dark:to-violet-600/5
                                opacity-0 group-hover:opacity-100 transition-opacity duration-300" />

                  <div className="relative flex items-start gap-3">
                    <div className="p-2.5 bg-white dark:bg-botbot-dark rounded-xl shadow-sm
                                  group-hover:bg-violet-50 dark:group-hover:bg-botbot-purple/10
                                  group-hover:shadow-md
                                  text-gray-600 dark:text-gray-400
                                  group-hover:text-violet-600 dark:group-hover:text-botbot-purple
                                  transition-all duration-300">
                      {widget.icon}
                    </div>
                    <div className="flex-1 min-w-0">
                      <h3 className="font-semibold text-gray-800 dark:text-white mb-1
                                   group-hover:text-violet-700 dark:group-hover:text-botbot-purple
                                   transition-colors duration-300">
                        {widget.label}
                      </h3>
                      <p className="text-xs text-gray-500 dark:text-gray-400 line-clamp-2
                                  group-hover:text-gray-600 dark:group-hover:text-gray-300
                                  transition-colors duration-300">
                        {widget.description}
                      </p>
                    </div>
                  </div>

                  {/* Floating add button */}
                  <div className="absolute top-3 right-3 opacity-0 group-hover:opacity-100
                                transform translate-x-2 group-hover:translate-x-0
                                transition-all duration-300">
                    <div className="bg-violet-600 dark:bg-botbot-purple text-white rounded-full p-1.5
                                  shadow-lg">
                      <Plus className="w-3.5 h-3.5" />
                    </div>
                  </div>
                </button>
              ))}
            </div>
          )}
        </div>

        {/* Footer */}
        <div className="px-6 py-4 border-t border-gray-200 dark:border-gray-700">
          <div className="flex justify-between items-center">
            <p className="text-sm text-gray-500 dark:text-gray-400">
              {filteredWidgets.length} widget{filteredWidgets.length !== 1 ? 's' : ''} available
            </p>
            <button
              onClick={onClose}
              className="px-4 py-2 text-gray-600 dark:text-gray-400 hover:text-gray-800
                       dark:hover:text-white transition-colors"
            >
              Cancel
            </button>
          </div>
          </div>
          </motion.div>
        </motion.div>
      )}
    </AnimatePresence>
  );
}