'use client';

import { useState, useEffect, useRef } from 'react';
import Container from '@/components/ui/container';
import GamepadVisualizer from './gamepad-visualizer';
import { useLanguage } from '@/contexts/LanguageContext';
import useRobotActions from '@/hooks/ros/useRobotActions';
import { Gamepad2, RefreshCw, Save, Trash2 } from 'lucide-react';
import { cn } from '@/utils/cn';

export type GamepadButtonMapping = Record<number, string>;
export type GamepadAxisMapping = Record<string, string>;

interface GamepadConfigProps {
  mappings: GamepadButtonMapping;
  onMappingsChange: (mappings: GamepadButtonMapping) => void;
  axisMappings?: GamepadAxisMapping;
  onAxisMappingsChange?: (mappings: GamepadAxisMapping) => void;
  hasChanges?: boolean;
}

const BUTTON_NAMES: Record<number, string> = {
  0: 'A',
  1: 'B',
  2: 'X',
  3: 'Y',
  4: 'LB',
  5: 'RB',
  6: 'LT',
  7: 'RT',
  8: 'Back',
  9: 'Start',
  10: 'L3',
  11: 'R3',
  12: 'D-Pad Up',
  13: 'D-Pad Down',
  14: 'D-Pad Left',
  15: 'D-Pad Right',
};

const AXIS_NAMES: Record<string, string> = {
  'left': 'Left Joystick',
  'right': 'Right Joystick',
};

const AXIS_ACTIONS = [
  { value: 'movement', label: 'Movement (Forward/Back, Left/Right)' },
  { value: 'rotation', label: 'Rotation (Turn Left/Right, Look Up/Down)' },
  { value: 'none', label: 'No Action' },
];

const DEFAULT_MAPPINGS: GamepadButtonMapping = {
  0: 'getUp',       // A - Stand up
  1: 'getDown',     // B - Stand down
  2: 'stopMove',    // X - Stop
  3: 'hello',       // Y - Hello gesture
  4: 'lightOff',    // LB - Light off
  5: 'lightOn',     // RB - Light on
  6: 'antiCollisionOff', // LT - Anti-collision off
  7: 'antiCollisionOn',  // RT - Anti-collision on
  8: 'balanceStand',     // Back - Balance stand
  9: 'emergencyOn',      // Start - Emergency stop
};

const DEFAULT_AXIS_MAPPINGS: GamepadAxisMapping = {
  'left': 'movement',   // Left stick controls movement
  'right': 'rotation',  // Right stick controls rotation
};

export default function GamepadConfig({
  mappings,
  onMappingsChange,
  axisMappings = DEFAULT_AXIS_MAPPINGS,
  onAxisMappingsChange,
  hasChanges = false
}: GamepadConfigProps) {
  const { t } = useLanguage();
  const robotActions = useRobotActions();
  const [pressedButtons, setPressedButtons] = useState<Set<number>>(new Set());
  const [selectedButton, setSelectedButton] = useState<number | null>(null);
  const [selectedAxis, setSelectedAxis] = useState<string | null>(null);
  const [gamepadConnected, setGamepadConnected] = useState(false);
  const animationFrameRef = useRef<number | undefined>(undefined);

  // Get ALL available action names from robotActions
  const availableActions = Object.keys(robotActions).filter(
    action => action !== 'prompt' // Only exclude prompt as it needs input
  );

  // Gamepad monitoring
  useEffect(() => {
    const updateGamepadState = () => {
      const gamepads = navigator.getGamepads();
      const pressed = new Set<number>();

      for (const gamepad of gamepads) {
        if (!gamepad) continue;

        // Check button states
        gamepad.buttons.forEach((button, index) => {
          if (button.pressed) {
            pressed.add(index);
          }
        });
      }

      setPressedButtons(pressed);
      animationFrameRef.current = requestAnimationFrame(updateGamepadState);
    };

    const handleGamepadConnected = () => {
      setGamepadConnected(true);
      animationFrameRef.current = requestAnimationFrame(updateGamepadState);
    };

    const handleGamepadDisconnected = () => {
      const gamepads = navigator.getGamepads();
      const hasGamepad = Array.from(gamepads).some(g => g !== null);
      setGamepadConnected(hasGamepad);

      if (!hasGamepad && animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };

    // Check initial state
    handleGamepadDisconnected();

    window.addEventListener('gamepadconnected', handleGamepadConnected);
    window.addEventListener('gamepaddisconnected', handleGamepadDisconnected);

    return () => {
      window.removeEventListener('gamepadconnected', handleGamepadConnected);
      window.removeEventListener('gamepaddisconnected', handleGamepadDisconnected);
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, []);

  const handleButtonClick = (buttonIndex: number) => {
    setSelectedButton(selectedButton === buttonIndex ? null : buttonIndex);
    setSelectedAxis(null);
  };

  const handleActionAssign = (action: string) => {
    if (selectedButton === null) return;

    const newMappings = { ...mappings };
    if (action === 'none') {
      delete newMappings[selectedButton];
    } else {
      newMappings[selectedButton] = action;
    }

    onMappingsChange(newMappings);
  };

  const handleResetToDefault = () => {
    onMappingsChange(DEFAULT_MAPPINGS);
    if (onAxisMappingsChange) {
      onAxisMappingsChange(DEFAULT_AXIS_MAPPINGS);
    }
    setSelectedButton(null);
    setSelectedAxis(null);
  };

  const handleClearAll = () => {
    onMappingsChange({});
    if (onAxisMappingsChange) {
      onAxisMappingsChange({ left: 'none', right: 'none' });
    }
    setSelectedButton(null);
    setSelectedAxis(null);
  };

  const handleAxisAssign = (axis: string, action: string) => {
    if (onAxisMappingsChange) {
      const newMappings = { ...axisMappings };
      newMappings[axis] = action;
      onAxisMappingsChange(newMappings);
    }
  };

  const handleAxisClick = (axis: string) => {
    setSelectedAxis(selectedAxis === axis ? null : axis);
    setSelectedButton(null);
  };

  return (
    <Container
      title={
        <div className="flex items-center gap-2">
          <Gamepad2 className={cn(
            'w-5 h-5',
            gamepadConnected ? 'text-green-500' : 'text-gray-400'
          )} />
          <span>{t('settings', 'gamepadConfig')}</span>
          {gamepadConnected && (
            <span className="text-xs text-green-500 ml-2">
              {t('settings', 'gamepadConnected')}
            </span>
          )}
        </div>
      }
      className="w-full"
    >
      <div className="space-y-6">
        {/* Gamepad Visualizer */}
        <div className="bg-gray-50 dark:bg-botbot-darker rounded-lg p-4">
          <GamepadVisualizer
            pressedButtons={pressedButtons}
            mappedButtons={mappings}
            onButtonClick={handleButtonClick}
            selectedButton={selectedButton}
          />

          {!gamepadConnected && (
            <div className="text-center text-sm text-gray-500 dark:text-gray-400 mt-2">
              {t('settings', 'connectGamepadPrompt')}
            </div>
          )}
        </div>

        {/* Button Configuration */}
        {selectedButton !== null && (
          <div className="bg-blue-50 dark:bg-blue-900/20 border border-blue-200 dark:border-blue-800 rounded-lg p-4">
            <h3 className="font-medium mb-3">
              {t('settings', 'configureButton')}: {BUTTON_NAMES[selectedButton] || `Button ${selectedButton}`}
            </h3>

            <div className="space-y-2">
              <label className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                {t('settings', 'assignAction')}
              </label>
              <select
                value={mappings[selectedButton] || 'none'}
                onChange={(e) => handleActionAssign(e.target.value)}
                className="w-full p-2 border border-gray-300 dark:border-botbot-darker rounded-md
                           focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent
                           bg-white dark:bg-botbot-dark text-gray-800 dark:text-gray-100"
              >
                <option value="none">{t('settings', 'noAction')}</option>

                {/* Movement & Posture Actions */}
                <optgroup label={t('settings', 'movementActions')}>
                  {robotActions.getUp && <option value="getUp">{robotActions.getUp.label}</option>}
                  {robotActions.getDown && <option value="getDown">{robotActions.getDown.label}</option>}
                  {robotActions.stopMove && <option value="stopMove">{robotActions.stopMove.label}</option>}
                  {robotActions.balanceStand && <option value="balanceStand">{robotActions.balanceStand.label}</option>}
                  {robotActions.jointLock && <option value="jointLock">{robotActions.jointLock.label}</option>}
                  {robotActions.sit && <option value="sit">{robotActions.sit.label}</option>}
                  {robotActions.riseSit && <option value="riseSit">{robotActions.riseSit.label}</option>}
                </optgroup>

                {/* Gesture & Animation Actions */}
                <optgroup label={t('settings', 'gestureActions')}>
                  {robotActions.hello && <option value="hello">{robotActions.hello.label}</option>}
                  {robotActions.stretch && <option value="stretch">{robotActions.stretch.label}</option>}
                  {robotActions.dance && <option value="dance">{robotActions.dance.label}</option>}
                </optgroup>

                {/* System Control Actions */}
                <optgroup label={t('settings', 'systemActions')}>
                  {robotActions.lightOn && <option value="lightOn">{robotActions.lightOn.label}</option>}
                  {robotActions.lightOff && <option value="lightOff">{robotActions.lightOff.label}</option>}
                  {robotActions.antiCollisionOn && <option value="antiCollisionOn">{robotActions.antiCollisionOn.label}</option>}
                  {robotActions.antiCollisionOff && <option value="antiCollisionOff">{robotActions.antiCollisionOff.label}</option>}
                  {robotActions.poseOn && <option value="poseOn">{robotActions.poseOn.label}</option>}
                  {robotActions.poseOff && <option value="poseOff">{robotActions.poseOff.label}</option>}
                </optgroup>

                {/* Emergency Actions */}
                <optgroup label={t('settings', 'emergencyActions')}>
                  {robotActions.emergencyOn && <option value="emergencyOn">{robotActions.emergencyOn.label}</option>}
                  {robotActions.emergencyOff && <option value="emergencyOff">{robotActions.emergencyOff.label}</option>}
                </optgroup>
              </select>
            </div>
          </div>
        )}

        {/* Joystick Configuration */}
        <div className="space-y-2">
          <h3 className="font-medium text-sm text-gray-700 dark:text-gray-300">
            Joystick Configuration
          </h3>
          <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
            {Object.entries(AXIS_NAMES).map(([axis, name]) => {
              const mapping = axisMappings[axis];
              const action = AXIS_ACTIONS.find(a => a.value === mapping);

              return (
                <div
                  key={axis}
                  onClick={() => handleAxisClick(axis)}
                  className={cn(
                    'flex items-center justify-between p-2 rounded-md border cursor-pointer transition-colors',
                    selectedAxis === axis
                      ? 'border-blue-500 bg-blue-50 dark:bg-blue-900/20'
                      : 'border-gray-200 dark:border-botbot-darker hover:bg-gray-50 dark:hover:bg-botbot-darker'
                  )}
                >
                  <span className="font-medium text-sm">{name}</span>
                  <span className="text-sm text-gray-600 dark:text-gray-400">
                    {action?.label || 'Not Configured'}
                  </span>
                </div>
              );
            })}
          </div>
        </div>

        {/* Axis Configuration Panel */}
        {selectedAxis !== null && (
          <div className="bg-green-50 dark:bg-green-900/20 border border-green-200 dark:border-green-800 rounded-lg p-4">
            <h3 className="font-medium mb-3">
              Configure: {AXIS_NAMES[selectedAxis]}
            </h3>

            <div className="space-y-2">
              <label className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                Assign Function
              </label>
              <select
                value={axisMappings[selectedAxis] || 'none'}
                onChange={(e) => handleAxisAssign(selectedAxis, e.target.value)}
                className="w-full p-2 border border-gray-300 dark:border-botbot-darker rounded-md
                           focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent
                           bg-white dark:bg-botbot-dark text-gray-800 dark:text-gray-100"
              >
                {AXIS_ACTIONS.map(action => (
                  <option key={action.value} value={action.value}>
                    {action.label}
                  </option>
                ))}
              </select>
            </div>
          </div>
        )}

        {/* Current Mappings List */}
        <div className="space-y-2">
          <h3 className="font-medium text-sm text-gray-700 dark:text-gray-300">
            {t('settings', 'currentMappings')}
          </h3>
          <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
            {Object.entries(BUTTON_NAMES).map(([index, name]) => {
              const buttonIndex = parseInt(index);
              const mapping = mappings[buttonIndex];
              const action = mapping ? robotActions[mapping as keyof typeof robotActions] : null;

              return (
                <div
                  key={index}
                  onClick={() => handleButtonClick(buttonIndex)}
                  className={cn(
                    'flex items-center justify-between p-2 rounded-md border cursor-pointer transition-colors',
                    selectedButton === buttonIndex
                      ? 'border-blue-500 bg-blue-50 dark:bg-blue-900/20'
                      : 'border-gray-200 dark:border-botbot-darker hover:bg-gray-50 dark:hover:bg-botbot-darker'
                  )}
                >
                  <span className="font-medium text-sm">{name}</span>
                  <span className="text-sm text-gray-600 dark:text-gray-400">
                    {action?.label || t('settings', 'notMapped')}
                  </span>
                </div>
              );
            })}
          </div>
        </div>

        {/* Action Buttons */}
        <div className="flex flex-wrap gap-2 pt-2">
          <button
            onClick={handleResetToDefault}
            className="flex items-center gap-2 px-3 py-2 bg-gray-200 dark:bg-botbot-darker
                       hover:bg-gray-300 dark:hover:bg-botbot-dark rounded-md text-sm
                       transition-colors"
          >
            <RefreshCw className="w-4 h-4" />
            {t('settings', 'resetToDefault')}
          </button>

          <button
            onClick={handleClearAll}
            className="flex items-center gap-2 px-3 py-2 bg-gray-200 dark:bg-botbot-darker
                       hover:bg-gray-300 dark:hover:bg-botbot-dark rounded-md text-sm
                       transition-colors"
          >
            <Trash2 className="w-4 h-4" />
            {t('settings', 'clearAll')}
          </button>
        </div>
      </div>
    </Container>
  );
}