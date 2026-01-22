'use client';

import { useState, useEffect } from 'react';
import { Gamepad } from 'lucide-react';
import Container from '@/ui/container';

export default function GamepadControlsInfo() {
  const [gamepadConnected, setGamepadConnected] = useState(false);

  useEffect(() => {
    const checkGamepads = () => {
      const gamepads = navigator.getGamepads();
      setGamepadConnected(
        gamepads.some((gamepad) => gamepad !== null)
      );
    };

    // Initial check
    checkGamepads();

    // Event listeners for gamepad connections/disconnections
    const handleGamepadConnected = () => {
      setGamepadConnected(true);
    };

    const handleGamepadDisconnected = () => {
      // We need to check if any gamepads are still connected
      setTimeout(checkGamepads, 100);
    };

    window.addEventListener('gamepadconnected', handleGamepadConnected);
    window.addEventListener('gamepaddisconnected', handleGamepadDisconnected);

    // Regular check for gamepad status
    const interval = setInterval(checkGamepads, 1000);

    return () => {
      window.removeEventListener('gamepadconnected', handleGamepadConnected);
      window.removeEventListener('gamepaddisconnected', handleGamepadDisconnected);
      clearInterval(interval);
    };
  }, []);

  return (
    <Container
      title={
        <div className="flex items-center">
          <Gamepad className={`mr-2 w-5 h-5 ${gamepadConnected ? 'text-green-500' : 'text-gray-400'}`} />
          <span>
            {gamepadConnected 
              ? 'Gamepad Connected' 
              : 'Gamepad Disconnected'}
          </span>
        </div>
      }
      className="w-full mt-2"
    >
      <div className="p-2 text-sm">
        {gamepadConnected ? (
          <div className="space-y-2">
            <div>
              <h3 className="font-medium mb-1">Movement Controls:</h3>
              <ul className="list-disc pl-5 space-y-1">
                <li>Left Stick: Move robot forward/backward/left/right</li>
                <li>Right Stick: Turn robot left/right</li>
              </ul>
            </div>
          </div>
        ) : (
          <p>Connect a gamepad controller to control the robot. Press any button on the controller to activate it.</p>
        )}
      </div>
    </Container>
  );
} 