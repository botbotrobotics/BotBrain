'use client';

import { useState, useEffect } from 'react';
import { useLanguage } from '@/contexts/LanguageContext';
import { languageNames, LanguageCode } from '@/utils/translations';
import { getConfig, setConfig } from '@/utils/config';
import { Check, Save } from 'lucide-react';
import { auditLogger } from '@/utils/audit-logger';
import GamepadConfig from '@/components/settings/gamepad-config';
import type { GamepadButtonMapping, GamepadAxisMapping } from '@/components/settings/gamepad-config';

// Force dynamic rendering to prevent static generation issues
export const dynamic = 'force-dynamic';

export default function SettingsPage() {
  const { t, language, setLanguage } = useLanguage();
  const [invertJoystick, setInvertJoystick] = useState(false);
  const [videoFormat, setVideoFormat] = useState<'mp4' | 'webm'>('mp4');
  const [overlayColor, setOverlayColor] = useState<'white' | 'black' | 'red' | 'purple' | 'blue' | 'green'>('white');
  const [gamepadMappings, setGamepadMappings] = useState<GamepadButtonMapping>({});
  const [gamepadAxisMappings, setGamepadAxisMappings] = useState<GamepadAxisMapping>({ left: 'movement', right: 'rotation' });
  const [joystickVisualizationOnly, setJoystickVisualizationOnly] = useState(false);

  // Track original values to detect changes
  const [originalValues, setOriginalValues] = useState({
    invertJoystick: false,
    language: '',
    videoFormat: 'mp4' as 'mp4' | 'webm',
    overlayColor: 'white' as 'white' | 'black' | 'red' | 'purple' | 'blue' | 'green',
    gamepadMappings: {} as GamepadButtonMapping,
    gamepadAxisMappings: { left: 'movement', right: 'rotation' } as GamepadAxisMapping,
    joystickVisualizationOnly: false,
  });

  // Track if any setting has changed
  const [hasChanges, setHasChanges] = useState(false);
  
  // Track save status for feedback
  const [saveStatus, setSaveStatus] = useState<'idle' | 'saving' | 'saved'>('idle');

  // Set page title
  useEffect(() => {
    document.title = 'Settings - BotBot';
  }, []);
  
  // Load saved settings when component mounts
  useEffect(() => {
    const savedVideoFormat = getConfig('videoFormat');
    const savedOverlayColor = getConfig('overlayColor');
    const savedGamepadMappings = getConfig('gamepadMappings');
    const savedGamepadAxisMappings = getConfig('gamepadAxisMappings') || { left: 'movement', right: 'rotation' };
    const savedJoystickVisualizationOnly = getConfig('joystickVisualizationOnly') || false;

    setVideoFormat(savedVideoFormat);
    setOverlayColor(savedOverlayColor);
    setGamepadMappings(savedGamepadMappings);
    setGamepadAxisMappings(savedGamepadAxisMappings);
    setJoystickVisualizationOnly(savedJoystickVisualizationOnly);

    // Store original values to detect changes
    setOriginalValues({
      invertJoystick,
      language,
      videoFormat: savedVideoFormat,
      overlayColor: savedOverlayColor,
      gamepadMappings: savedGamepadMappings,
      gamepadAxisMappings: savedGamepadAxisMappings,
      joystickVisualizationOnly: savedJoystickVisualizationOnly,
    });
  }, []);

  // Check for changes whenever any setting is updated
  useEffect(() => {
    const hasChanged =
      invertJoystick !== originalValues.invertJoystick ||
      language !== originalValues.language ||
      videoFormat !== originalValues.videoFormat ||
      overlayColor !== originalValues.overlayColor ||
      JSON.stringify(gamepadMappings) !== JSON.stringify(originalValues.gamepadMappings) ||
      JSON.stringify(gamepadAxisMappings) !== JSON.stringify(originalValues.gamepadAxisMappings) ||
      joystickVisualizationOnly !== originalValues.joystickVisualizationOnly;

    setHasChanges(hasChanged);

    // Reset save status when changes are made after saving
    if (hasChanged && saveStatus === 'saved') {
      setSaveStatus('idle');
    }
  }, [invertJoystick, language, videoFormat, overlayColor, gamepadMappings, gamepadAxisMappings, joystickVisualizationOnly, originalValues, saveStatus]);

  // Handle language change
  const handleLanguageChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setLanguage(e.target.value as LanguageCode);
  };
  
  // Handle video format change
  const handleVideoFormatChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setVideoFormat(e.target.value as 'mp4' | 'webm');
  };
  
  // Handle overlay color change
  const handleOverlayColorChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setOverlayColor(e.target.value as 'white' | 'black' | 'red' | 'purple' | 'blue' | 'green');
  };
  
  // Save all settings
  const handleSave = async () => {
    setSaveStatus('saving');

    // Save all changes to config
    setConfig('videoFormat', videoFormat);
    setConfig('overlayColor', overlayColor);
    setConfig('gamepadMappings', gamepadMappings);
    setConfig('gamepadAxisMappings', gamepadAxisMappings);
    setConfig('joystickVisualizationOnly', joystickVisualizationOnly);

    // Log settings changes
    const changes: Record<string, any> = {};

    if (invertJoystick !== originalValues.invertJoystick) {
      changes.invertJoystick = { from: originalValues.invertJoystick, to: invertJoystick };
    }
    if (language !== originalValues.language) {
      changes.language = { from: originalValues.language, to: language };
    }
    if (videoFormat !== originalValues.videoFormat) {
      changes.videoFormat = { from: originalValues.videoFormat, to: videoFormat };
    }
    if (overlayColor !== originalValues.overlayColor) {
      changes.overlayColor = { from: originalValues.overlayColor, to: overlayColor };
    }
    if (JSON.stringify(gamepadMappings) !== JSON.stringify(originalValues.gamepadMappings)) {
      changes.gamepadMappings = { from: originalValues.gamepadMappings, to: gamepadMappings };
    }
    if (JSON.stringify(gamepadAxisMappings) !== JSON.stringify(originalValues.gamepadAxisMappings)) {
      changes.gamepadAxisMappings = { from: originalValues.gamepadAxisMappings, to: gamepadAxisMappings };
    }
    if (joystickVisualizationOnly !== originalValues.joystickVisualizationOnly) {
      changes.joystickVisualizationOnly = { from: originalValues.joystickVisualizationOnly, to: joystickVisualizationOnly };
    }
    
    await auditLogger.log({
      event_type: 'system',
      event_action: 'settings_updated',
      event_details: {
        changes,
        settings: {
          invertJoystick,
          language,
          videoFormat,
          overlayColor,
          gamepadMappings,
          gamepadAxisMappings,
          joystickVisualizationOnly
        }
      }
    });

    // Update original values
    setOriginalValues({
      invertJoystick,
      language,
      videoFormat,
      overlayColor,
      gamepadMappings,
      gamepadAxisMappings,
      joystickVisualizationOnly,
    });
    
    // Show saved status briefly
    setTimeout(() => {
      setSaveStatus('saved');
      setTimeout(() => {
        if (!hasChanges) {
          setSaveStatus('idle');
        }
      }, 2000);
    }, 500);
  };
  
  return (
    <div className="w-full min-h-[calc(100vh-56px-24px)] flex flex-col md:flex-row items-stretch justify-between relative px-1 overflow-hidden">
      {/* Main content area */}
      <div className="w-full flex flex-col pt-2 px-1 pb-2">
        <div className="w-full flex-1 bg-white/5 backdrop-blur-sm rounded-lg p-4 overflow-y-auto max-h-[calc(100vh-56px-24px-1rem)]">
          <div className="max-w-4xl mx-auto space-y-6 pb-4">
            {/* Settings Card */}
            <div className="bg-white dark:bg-botbot-darker rounded-lg shadow-md p-6">
              <div className="flex justify-between items-center mb-6">
              <h1 className="text-2xl font-semibold text-gray-800 dark:text-white">{t('settings', 'title')}</h1>
              
              {/* Save Button */}
              <button
                onClick={handleSave}
                disabled={!hasChanges || saveStatus === 'saving'}
                className={`flex items-center justify-center px-4 py-2 rounded-md text-sm font-medium 
                  transition-all duration-200 ${
                    hasChanges && saveStatus !== 'saving'
                      ? 'bg-primary dark:bg-botbot-accent text-white hover:bg-primary/90 dark:hover:bg-botbot-accent/90'
                      : 'bg-gray-200 dark:bg-botbot-dark text-gray-500 dark:text-gray-400 cursor-not-allowed'
                  }`}
              >
                {saveStatus === 'saving' ? (
                  <>
                    <div className="w-4 h-4 border-2 border-white border-t-transparent rounded-full animate-spin mr-2"></div>
                    Saving...
                  </>
                ) : saveStatus === 'saved' ? (
                  <>
                    <Check className="w-4 h-4 mr-2" />
                    Saved
                  </>
                ) : (
                  <>
                    <Save className="w-4 h-4 mr-2" />
                    Save
                  </>
                )}
              </button>
            </div>
            
            <div className="space-y-6">
              {/* Video Format Selection */}
              <div className="space-y-2">
                <label htmlFor="video-format" className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                  {t('settings', 'videoFormat')}
                </label>
                <select
                  id="video-format"
                  value={videoFormat}
                  onChange={handleVideoFormatChange}
                  className="w-full p-2 border border-gray-300 dark:border-botbot-darker rounded-md 
                             focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent
                             bg-white dark:bg-botbot-dark text-gray-800 dark:text-gray-100"
                >
                  <option value="mp4">MP4</option>
                  <option value="webm">WebM</option>
                </select>
                <p className="text-xs text-gray-500 dark:text-gray-400">
                  {t('settings', 'videoFormatDescription')}
                </p>
              </div>
              
              {/* Language Selection */}
              <div className="space-y-2">
                <label htmlFor="language" className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                  {t('settings', 'language')}
                </label>
                <select
                  id="language"
                  value={language}
                  onChange={handleLanguageChange}
                  className="w-full p-2 border border-gray-300 dark:border-botbot-darker rounded-md 
                             focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent
                             bg-white dark:bg-botbot-dark text-gray-800 dark:text-gray-100"
                >
                  {Object.entries(languageNames).map(([code, name]) => (
                    <option key={code} value={code}>{name}</option>
                  ))}
                </select>
                <p className="text-xs text-gray-500 dark:text-gray-400">
                  {t('settings', 'languageDescription')}
                </p>
              </div>
              
              {/* Overlay Color Selection */}
              <div className="space-y-2">
                <label htmlFor="overlay-color" className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                  {t('settings', 'overlayColor')}
                </label>
                <select
                  id="overlay-color"
                  value={overlayColor}
                  onChange={handleOverlayColorChange}
                  className="w-full p-2 border border-gray-300 dark:border-botbot-darker rounded-md 
                             focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent
                             bg-white dark:bg-botbot-dark text-gray-800 dark:text-gray-100"
                >
                  <option value="white">{t('settings', 'colorWhite')}</option>
                  <option value="black">{t('settings', 'colorBlack')}</option>
                  <option value="red">{t('settings', 'colorRed')}</option>
                  <option value="purple">{t('settings', 'colorPurple')}</option>
                  <option value="blue">{t('settings', 'colorBlue')}</option>
                  <option value="green">{t('settings', 'colorGreen')}</option>
                </select>
                <p className="text-xs text-gray-500 dark:text-gray-400">
                  {t('settings', 'overlayColorDescription')}
                </p>
              </div>
              
              {/* Invert Joystick L/R */}
              <div className="flex items-center justify-between">
                <div>
                  <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300">{t('settings', 'invertJoystick')}</h3>
                  <p className="text-xs text-gray-500 dark:text-gray-400">
                    {t('settings', 'invertJoystickDescription')}
                  </p>
                </div>
                <label className="relative inline-flex items-center cursor-pointer">
                  <input
                    type="checkbox"
                    className="sr-only peer"
                    checked={invertJoystick}
                    onChange={() => setInvertJoystick(!invertJoystick)}
                  />
                  <div className="w-11 h-6 bg-gray-200 dark:bg-botbot-dark peer-focus:outline-none
                                 peer-focus:ring-2 peer-focus:ring-primary dark:peer-focus:ring-botbot-accent
                                 rounded-full peer peer-checked:after:translate-x-full
                                 peer-checked:after:border-white after:content-[''] after:absolute
                                 after:top-[2px] after:left-[2px] after:bg-white after:border-gray-300
                                 after:border after:rounded-full after:h-5 after:w-5 after:transition-all
                                 peer-checked:bg-primary dark:peer-checked:bg-botbot-accent">
                  </div>
                </label>
              </div>

              {/* Joystick Visualization Only Mode */}
              <div className="flex items-center justify-between">
                <div>
                  <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300">Joystick Visualization Only</h3>
                  <p className="text-xs text-gray-500 dark:text-gray-400">
                    Display joystick input in visualizer without sending commands to robot
                  </p>
                </div>
                <label className="relative inline-flex items-center cursor-pointer">
                  <input
                    type="checkbox"
                    className="sr-only peer"
                    checked={joystickVisualizationOnly}
                    onChange={() => setJoystickVisualizationOnly(!joystickVisualizationOnly)}
                  />
                  <div className="w-11 h-6 bg-gray-200 dark:bg-botbot-dark peer-focus:outline-none
                                 peer-focus:ring-2 peer-focus:ring-primary dark:peer-focus:ring-botbot-accent
                                 rounded-full peer peer-checked:after:translate-x-full
                                 peer-checked:after:border-white after:content-[''] after:absolute
                                 after:top-[2px] after:left-[2px] after:bg-white after:border-gray-300
                                 after:border after:rounded-full after:h-5 after:w-5 after:transition-all
                                 peer-checked:bg-primary dark:peer-checked:bg-botbot-accent">
                  </div>
                </label>
              </div>
              </div>
            </div>

            {/* Gamepad Configuration */}
            <GamepadConfig
              mappings={gamepadMappings}
              onMappingsChange={setGamepadMappings}
              axisMappings={gamepadAxisMappings}
              onAxisMappingsChange={setGamepadAxisMappings}
              hasChanges={hasChanges}
            />
          </div>
        </div>
      </div>
    </div>
  );
} 