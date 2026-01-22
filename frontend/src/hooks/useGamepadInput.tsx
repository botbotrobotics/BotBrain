'use client';

import { useEffect, useRef, useCallback } from 'react';
import { Vector2 } from 'three';
import { Vector3 } from 'roslib';
import { StickType } from '@/types/StickType';
import { getConfig } from '@/utils/config';
import useRobotActions from '@/hooks/ros/useRobotActions';

interface GamepadHandler {
  onGamepadUpdate: (options: {
    stick: StickType;
    vector?: Vector2;
    turn?: number;
    angular?: Vector3;
    linear?: Vector2;
  }) => void;
  onGamepadDisconnected?: () => void;
}

const DEADZONE = 0.1; // Ignore small stick movements

export default function useGamepadInput({ onGamepadUpdate, onGamepadDisconnected }: GamepadHandler) {
  const requestRef = useRef<number | undefined>(undefined);
  const gamepadsRef = useRef<Map<number, Gamepad>>(new Map());
  const prevGamepadCountRef = useRef(0);
  // Track the state of both sticks separately
  const leftStickStateRef = useRef<Vector2>(new Vector2(0, 0));
  const rightStickXRef = useRef<number>(0);
  const rightStickYRef = useRef<number>(0);

  // Track button states to handle button press events
  const prevButtonStatesRef = useRef<boolean[]>([]);
  const buttonMappingsRef = useRef<Record<number, string>>({});
  const axisMappingsRef = useRef<Record<string, string>>({ left: 'movement', right: 'rotation' });
  const robotActions = useRobotActions();

  // Check if we're on the settings page or if visualization only mode is enabled
  const isSettingsPage = typeof window !== 'undefined' && window.location.pathname === '/settings';
  const visualizationOnlyRef = useRef(false);
  
  // Memoize callback to avoid dependency changes
  const handleGamepadDisconnected = useCallback(() => {
    if (onGamepadDisconnected) {
      onGamepadDisconnected();
    }
  }, [onGamepadDisconnected]);
  
  // Handle gamepad connection
  useEffect(() => {
    const handleGamepadConnected = (event: GamepadEvent) => {
      console.log(`Gamepad connected: ${event.gamepad.id}`);
      gamepadsRef.current.set(event.gamepad.index, event.gamepad);
    };

    const handleDisconnect = (event: GamepadEvent) => {
      console.log(`Gamepad disconnected: ${event.gamepad.id}`);
      gamepadsRef.current.delete(event.gamepad.index);
      
      // Call disconnection callback
      handleGamepadDisconnected();
    };

    window.addEventListener('gamepadconnected', handleGamepadConnected);
    window.addEventListener('gamepaddisconnected', handleDisconnect);

    return () => {
      window.removeEventListener('gamepadconnected', handleGamepadConnected);
      window.removeEventListener('gamepaddisconnected', handleDisconnect);
      
      // Ensure we cancel the animation frame when unmounting
      if (requestRef.current) {
        cancelAnimationFrame(requestRef.current);
      }
    };
  }, [handleGamepadDisconnected]);

  // Load button and axis mappings periodically to pick up changes
  useEffect(() => {
    const loadMappings = () => {
      buttonMappingsRef.current = getConfig('gamepadMappings');
      axisMappingsRef.current = getConfig('gamepadAxisMappings') || { left: 'movement', right: 'rotation' };
      visualizationOnlyRef.current = getConfig('joystickVisualizationOnly') || false;
    };

    // Load initial mappings
    loadMappings();

    // Reload mappings periodically to pick up settings changes
    const interval = setInterval(loadMappings, 1000);

    return () => clearInterval(interval);
  }, []);

  // Apply deadzone to joystick values
  const applyDeadzone = (value: number): number => {
    return Math.abs(value) < DEADZONE ? 0 : value;
  };

  // Main gamepad polling loop
  useEffect(() => {
    const updateGamepadState = () => {
      // Get fresh gamepad data
      const gamepads = navigator.getGamepads();
      let hasActiveGamepad = false;
      let leftStickUpdated = false;
      let rightStickUpdated = false;
      
      if (gamepads) {
        for (const gamepad of gamepads) {
          if (!gamepad) continue;
          
          hasActiveGamepad = true;
          // Store updated gamepad data
          gamepadsRef.current.set(gamepad.index, gamepad);

          // Handle button presses with configured mappings (skip if visualization only mode is enabled)
          if (!visualizationOnlyRef.current) {
            gamepad.buttons.forEach((button, index) => {
              const wasPressed = prevButtonStatesRef.current[index] || false;
              const isPressed = button.pressed;

              // Detect button press (not held)
              if (isPressed && !wasPressed) {
                const actionName = buttonMappingsRef.current[index];
                if (actionName && robotActions[actionName as keyof typeof robotActions]) {
                  const action = robotActions[actionName as keyof typeof robotActions];
                  // Execute the mapped action
                  action.action?.();
                }
              }
            });
          }

          // Update previous button states
          prevButtonStatesRef.current = gamepad.buttons.map(b => b.pressed);

          // Skip joystick movement commands on settings page or if visualization only mode is enabled
          if (!isSettingsPage && !visualizationOnlyRef.current) {
            // Get current axis mappings
            const leftMapping = axisMappingsRef.current.left;
            const rightMapping = axisMappingsRef.current.right;

            // Left stick (axes 0 and 1)
            const leftX = applyDeadzone(gamepad.axes[0]);
            const leftY = applyDeadzone(gamepad.axes[1]);

            if (leftMapping !== 'none') {
              if (leftX !== 0 || leftY !== 0) {
                // Update left stick state - match nipplejs format: (vector.y, -vector.x)
                leftStickStateRef.current.set(-leftY, leftX);
                leftStickUpdated = true;
              } else if (prevGamepadCountRef.current > 0) {
                // If stick was previously active but now centered, reset its state
                leftStickStateRef.current.set(0, 0);
                leftStickUpdated = true;
              }
            }

            // Right stick (axes 2 and 3)
            const rightX = applyDeadzone(gamepad.axes[2]);
            const rightY = applyDeadzone(gamepad.axes[3]);

            if (rightMapping !== 'none') {
              if (rightX !== 0 || rightY !== 0) {
                // Update right stick state
                rightStickXRef.current = rightX;
                rightStickYRef.current = rightY;
                rightStickUpdated = true;
              } else if (prevGamepadCountRef.current > 0) {
                // If stick was previously active but now centered, reset its state
                rightStickXRef.current = 0;
                rightStickYRef.current = 0;
                rightStickUpdated = true;
              }
            }

            // Send appropriate commands based on axis mappings
            if (leftStickUpdated && leftMapping === 'movement') {
              // Left stick for movement
              onGamepadUpdate({
                stick: 'left',
                linear: leftStickStateRef.current,
                angular: new Vector3()
              });
            } else if (leftStickUpdated && leftMapping === 'rotation') {
              // Left stick for rotation
              const angularVector = new Vector3({
                x: 0,
                y: -leftStickStateRef.current.x, // Y-axis control (up/down)
                z: -leftStickStateRef.current.y  // Z-axis control (left/right)
              });
              onGamepadUpdate({
                stick: 'left',
                angular: angularVector
              });
            }

            if (rightStickUpdated && rightMapping === 'movement') {
              // Right stick for movement
              const linear = new Vector2(-rightStickYRef.current, rightStickXRef.current);
              onGamepadUpdate({
                stick: 'right',
                linear: linear,
                angular: new Vector3()
              });
            } else if (rightStickUpdated && rightMapping === 'rotation') {
              // Right stick for rotation
              const angularVector = new Vector3({
                x: 0,
                y: -rightStickYRef.current, // Y-axis control (up/down)
                z: -rightStickXRef.current  // Z-axis control (left/right)
              });
              onGamepadUpdate({
                stick: 'right',
                angular: angularVector
              });
            }
          }
        }
      }
      
      // Update gamepad count for next frame
      prevGamepadCountRef.current = hasActiveGamepad ? 1 : 0;
      
      // Continue the loop
      requestRef.current = requestAnimationFrame(updateGamepadState);
    };
    
    // Start the gamepad polling loop
    requestRef.current = requestAnimationFrame(updateGamepadState);
    
    return () => {
      if (requestRef.current) {
        cancelAnimationFrame(requestRef.current);
      }
    };
  }, [onGamepadUpdate]);

  // Utility function to check if gamepads are connected
  const hasConnectedGamepads = (): boolean => {
    return gamepadsRef.current.size > 0;
  };

  return {
    connectedGamepads: gamepadsRef.current,
    hasConnectedGamepads
  };
} 