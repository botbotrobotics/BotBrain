'use client';

import { useKeyboardInput } from '@/hooks/useKeyboardInput';
import { MoveKeyType } from '@/types/MoveKeyType';
import useRobotVelocityPublisher from './ros/useRobotVelocityPublisher';
import { Vector3 } from 'roslib';
import { Twist } from '@/interfaces/ros/Twist';
import { Vector2 } from 'three';
import { StickType } from '@/types/StickType';
import { useRef, useState, useEffect } from 'react';
import { getConfig } from '@/utils/config';

const VELOCITY = 1;
const leftStick: StickType = 'left';
const rightStick: StickType = 'right';

interface JoyStickMoveOptions {
  stick: StickType;
  angle?: number;
  force?: number;
  linear?: Vector2;
  angular?: Vector3;
  inputSource?: 'joystick' | 'keyboard' | 'gamepad';
}

const keysPressed = new Set();

const isMoveKey = (key: MoveKeyType): boolean => {
  return key === 'w' || key === 'a' || key === 's' || key === 'd';
};

const isRotationKey = (key: MoveKeyType): boolean => {
  return (
    key === 'ArrowDown' ||
    key === 'ArrowUp' ||
    key === 'ArrowLeft' ||
    key === 'ArrowRight' ||
    key === 'q' ||
    key === 'e'
  );
};

const isMoveXAxisKey = (key: MoveKeyType): boolean => {
  return key === 'a' || key === 'd';
};

const isMoveYAxisKey = (key: MoveKeyType): boolean => {
  return key === 'w' || key === 's';
};

export default function useJoystickMove() {
  const publishVelocity = useRobotVelocityPublisher();
  const [joyVel, setJoyVel] = useState({ x: 0, y: 0 });
  const directionRef = useRef<Vector2>(new Vector2());
  const yRotationRef = useRef(0);
  const zRotationRef = useRef(0);
  const visualizationOnlyRef = useRef(false);

  // Load visualization only setting periodically
  useEffect(() => {
    const loadSettings = () => {
      visualizationOnlyRef.current = getConfig('joystickVisualizationOnly') || false;
    };

    // Load initial settings
    loadSettings();

    // Reload settings periodically to pick up changes
    const interval = setInterval(loadSettings, 1000);

    return () => clearInterval(interval);
  }, []);

  useKeyboardInput('keydown', (key: MoveKeyType, _event) => {
    const direction = directionRef.current;
    let zRotation = zRotationRef.current;
    let yRotation = yRotationRef.current;
    let stick: StickType = 'left';

    keysPressed.add(key);
    // console.log('Teclas pressionadas:', [...keysPressed]);

    keysPressed.forEach((keyPressed) => {
      if (keyPressed === 'w') {
        direction.set(VELOCITY, direction.y);
      }

      if (keyPressed === 's') {
        direction.set(-VELOCITY, direction.y);
      }

      if (keyPressed === 'a') {
        direction.set(direction.x, VELOCITY);
      }

      if (keyPressed === 'd') {
        direction.set(direction.x, -VELOCITY);
      }

      if (keyPressed === 'q' || keyPressed === 'ArrowLeft') {
        stick = 'right';
        zRotation = VELOCITY;
      }

      if (keyPressed === 'e' || keyPressed === 'ArrowRight') {
        stick = 'right';
        zRotation = -VELOCITY;
      }

      if (keyPressed === 'ArrowUp' || keyPressed === 'ArrowDown') {
        stick = 'right';
        yRotation = keyPressed === 'ArrowUp' ? -1 : 1;
      }
    });

    setJoyVel({ x: direction.x, y: direction.y });
    joystickMove({
      stick,
      linear: direction,
      angular: new Vector3({ y: yRotation, z: zRotation }),
    });
  });

  useKeyboardInput('keyup', (key: MoveKeyType, event) => {
    keysPressed.delete(event.key);

    if (isMoveKey(key)) {
      const newVector = new Vector2();
      if (isMoveXAxisKey(key)) {
        newVector.x = directionRef.current.x;
      } else if (isMoveYAxisKey(key)) {
        newVector.y = directionRef.current.y;
      }

      directionRef.current.set(newVector.x, newVector.y);
      joystickMove({ stick: leftStick, linear: directionRef.current });
    } else if (isRotationKey(key)) {
      joystickMove({
        stick: rightStick,
        linear: directionRef.current,
        angular: new Vector3(),
      });
    }
  });

  function joystickMove(options: JoyStickMoveOptions) {
    let twist: Twist = {
      linear: new Vector3(),
      angular: new Vector3(),
    };

    // Ensure we have valid objects for linear and angular
    const linear = new Vector3({
      x: options.linear?.x || 0,
      y: options.linear?.y || 0,
      z: 0,
    });

    // Create angular vector, handling both the Vector3 case and the individual components case
    let angular: Vector3;
    if (options.angular) {
      // If options.angular is a Vector3, use it directly - ensure all components are captured
      angular = new Vector3({
        x: options.angular.x || 0,
        y: options.angular.y || 0,
        z: options.angular.z || 0,
      });
    } else {
      // Otherwise construct from any provided values
      angular = new Vector3({
        x: 0,
        y: 0,
        z: 0,
      });
    }

    // Handle both movements in a combined way regardless of which stick is active
    twist = {
      linear,
      angular,
    };

    // Send all axes to the robot - both linear movement and all angular components
    // Only send commands if not in visualization only mode
    if (!visualizationOnlyRef.current) {
      // Determine input source - use provided source or infer from context
      const inputSource = options.inputSource ||
                         (keysPressed.size > 0 ? 'keyboard' : 'joystick');
      publishVelocity(twist, inputSource);
    }
  }

  return { joyVel, joystickMove };
}
