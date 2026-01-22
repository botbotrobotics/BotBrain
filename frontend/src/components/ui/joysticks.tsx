'use client';

import { useEffect, useRef, useState } from 'react';
import nipplejs, { JoystickManagerOptions, JoystickOutputData } from 'nipplejs';
import useJoystickMove from '@/hooks/useJoystickMove';
import useGamepadInput from '@/hooks/useGamepadInput';
import { Vector2 } from 'three';
import { Vector3 } from 'roslib';
import useRobotStatus from '@/hooks/ros/useRobotStatus';
import { useRobotCustomModeContext } from '@/contexts/RobotCustomModesContext';

export function Joysticks({ enabled }: { enabled: boolean }) {
  const [isDarkMode, setIsDarkMode] = useState(false);
  const { joystickMove } = useJoystickMove();
  const { robotStatus } = useRobotStatus();
  const { isPose } = useRobotCustomModeContext();
  const isLightTheme = () => !isDarkMode;
  const leftJoystickRef = useRef<HTMLDivElement>(null);
  const rightJoystickRef = useRef<HTMLDivElement>(null);
  const useLeftStick = useRef(true);
  const useRightStick = useRef(true);
  const leftStickOutputData = useRef<JoystickOutputData | undefined>(undefined);
  const rightStickOutputData = useRef<JoystickOutputData | undefined>(
    undefined
  );
  const leftJoystickManagerRef = useRef<nipplejs.JoystickManager | null>(null);
  const rightJoystickManagerRef = useRef<nipplejs.JoystickManager | null>(null);
  const leftStickVectorRef = useRef<Vector2>(new Vector2(0, 0));
  const rightStickTurnRef = useRef<Vector3>(new Vector3());

  // Track which input method is currently active for each joystick
  const [leftInputActive, setLeftInputActive] = useState<
    'mouse' | 'gamepad' | null
  >(null);
  const [rightInputActive, setRightInputActive] = useState<
    'mouse' | 'gamepad' | null
  >(null);

  const createJoystickStyles = (zone: HTMLElement) => {
    const front = zone.querySelector('.front') as HTMLElement;
    const back = zone.querySelector('.back') as HTMLElement;
    if (front && back) {
      front.style.background = isLightTheme() ? '#B388FF' : '#6b21a8';
      back.style.background = isLightTheme() ? '#2D1A45' : 'white';
      back.style.boxShadow = isLightTheme()
        ? '0 0 15px rgba(138, 43, 226, 0.3)'
        : '0 0 15px rgba(179, 136, 255, 0.3)';
      front.style.border = '2px solid rgba(255, 255, 255, 0.1)';
    }
  };

  const normalizeAngle = (theta: number): number => {
    // Vamos definir os extremos:
    // - Se theta estiver entre -π/4 e π/4 (mais para a direita), retorna 1.
    // - Se theta estiver entre 3π/4 e -3π/4 (mais para a esquerda), retorna -1.
    // Para os intervalos intermediários, interpolamos linearmente.
    if (theta >= -Math.PI / 4 && theta <= Math.PI / 4) {
      return 1;
    } else if (theta >= (3 * Math.PI) / 4 || theta <= -(3 * Math.PI) / 4) {
      return -1;
    } else if (theta > Math.PI / 4 && theta < (3 * Math.PI) / 4) {
      // Interpolação entre 1 (em π/4) e -1 (em 3π/4)
      return 1 - 2 * ((theta - Math.PI / 4) / (Math.PI / 2));
    } else {
      // theta entre -3π/4 e -π/4
      return -1 - 2 * ((theta + Math.PI / 4) / (Math.PI / 2));
    }
  };

  // Function to send combined joystick commands
  const sendCombinedJoystickCommands = () => {
    // Send combined state to both stick handlers to ensure both movements happen
    joystickMove?.({
      stick: 'left',
      linear: leftStickVectorRef.current,
      angular: rightStickTurnRef.current,
    });

    joystickMove?.({
      stick: 'right',
      linear: leftStickVectorRef.current,
      angular: rightStickTurnRef.current,
    });
  };

  // Reset joysticks to center position
  const resetJoysticks = () => {
    // Only reset if joysticks aren't being used by touch/mouse
    if (leftInputActive !== 'mouse') {
      setLeftInputActive(null);
      leftStickVectorRef.current.set(0, 0);
    }

    if (rightInputActive !== 'mouse') {
      setRightInputActive(null);
      rightStickTurnRef.current = new Vector3();
    }

    // Send zero movement commands if no input is active
    if (leftInputActive === null && rightInputActive === null) {
      sendCombinedJoystickCommands();
    }
  };

  // Use gamepad input
  useGamepadInput({
    onGamepadUpdate: (options) => {
      // Only use gamepad input if touch/mouse isn't active
      if (options.stick === 'left') {
        // Check if there's actual gamepad movement
        const hasMovement =
          options.linear &&
          (Math.abs(options.linear.x) > 0.01 ||
            Math.abs(options.linear.y) > 0.01);

        if (hasMovement && options.linear) {
          // Gamepad is providing movement - take control if mouse isn't active
          if (leftInputActive !== 'mouse') {
            setLeftInputActive('gamepad');
            
            // Update our internal state with gamepad values
            leftStickVectorRef.current.copy(options.linear);

            // Forward gamepad inputs to joystickMove function with proper format
            joystickMove?.({
              stick: 'left',
              linear: options.linear,
              angular: rightStickTurnRef.current, // Keep current angular value
              inputSource: 'gamepad'
            });

            // Visually update nipplejs joysticks to match gamepad position
            if (leftJoystickRef.current) {
              const frontElement = leftJoystickRef.current.querySelector(
                '.front'
              ) as HTMLElement;
              if (frontElement) {
                const maxDistance = 40; // Maximum pixel distance the joystick can move
                const x = options.linear.y * maxDistance; // Swap x/y based on your mapping
                const y = -options.linear.x * maxDistance;

                frontElement.style.transform = `translate(${x}px, ${y}px)`;
              }
            }
          }
        } else if (leftInputActive === 'gamepad') {
          // Gamepad has stopped providing input, release control
          setLeftInputActive(null);

          // Reset visual position
          if (leftJoystickRef.current) {
            const frontElement = leftJoystickRef.current.querySelector(
              '.front'
            ) as HTMLElement;
            if (frontElement) {
              frontElement.style.transform = 'translate(0px, 0px)';
            }
          }

          // Reset our internal state
          leftStickVectorRef.current.set(0, 0);

          // Send zero movement
          joystickMove?.({
            stick: 'left',
            linear: new Vector2(),
            angular: rightStickTurnRef.current // Keep current angular value
          });
        }
      }

      if (options.stick === 'right') {
        // Check if there's actual gamepad movement
        const hasMovement =
          options.angular &&
          (Math.abs(options.angular.z) > 0.01 || Math.abs(options.angular.y) > 0.01);

        if (hasMovement && options.angular) {
          // Gamepad is providing movement - take control if mouse isn't active
          if (rightInputActive !== 'mouse') {
            setRightInputActive('gamepad');
            
            // Update our internal state
            rightStickTurnRef.current = options.angular;

            // Forward gamepad inputs to joystickMove function
            joystickMove?.({
              stick: 'right',
              linear: leftStickVectorRef.current, // Keep current linear value
              angular: options.angular,
              inputSource: 'gamepad'
            });

            // Visually update nipplejs joysticks to match gamepad position
            if (rightJoystickRef.current) {
              const frontElement = rightJoystickRef.current.querySelector(
                '.front'
              ) as HTMLElement;
              if (frontElement) {
                const maxDistance = 40;
                // Apply both x and y transformations
                const x = -options.angular.z * maxDistance;
                const y = options.angular.y * maxDistance;

                frontElement.style.transform = `translate(${x}px, ${y}px)`;
              }
            }
          }
        } else if (rightInputActive === 'gamepad') {
          // Gamepad has stopped providing input, release control
          setRightInputActive(null);

          // Reset visual position
          if (rightJoystickRef.current) {
            const frontElement = rightJoystickRef.current.querySelector(
              '.front'
            ) as HTMLElement;
            if (frontElement) {
              frontElement.style.transform = 'translate(0px, 0px)';
            }
          }

          // Reset our internal state
          rightStickTurnRef.current = new Vector3();

          // Send zero movement
          joystickMove?.({
            stick: 'right',
            linear: leftStickVectorRef.current, // Keep current linear value
            angular: new Vector3()
          });
        }
      }
    },
    onGamepadDisconnected: resetJoysticks,
  });

  const startLeftJoystick = (
    _evt: nipplejs.EventData,
    data: nipplejs.JoystickOutputData
  ) => {
    useLeftStick.current = true;
    leftStickOutputData.current = data;
  };

  const moveLeftJoystick = (
    _evt: nipplejs.EventData,
    data: nipplejs.JoystickOutputData
  ) => {
    if (useLeftStick.current) {
      leftStickOutputData.current = data;
    }
  };

  const startRightJoystick = (
    _evt: nipplejs.EventData,
    data: nipplejs.JoystickOutputData
  ) => {
    useRightStick.current = true;
    rightStickOutputData.current = data;
    
    // Immediately update position to avoid any delays in processing
    if (data?.vector) {
      rightStickTurnRef.current = new Vector3({
        y: -data.vector.y,
        z: -data.vector.x,
      });
    }
  };

  const moveRightJoystick = (
    _evt: nipplejs.EventData,
    data: nipplejs.JoystickOutputData
  ) => {
    if (useRightStick.current) {
      rightStickOutputData.current = data;
      
      // Immediately update position to get more responsive control
      if (data?.vector) {
        console.log('Right joystick move:', data.vector.x, data.vector.y);
        rightStickTurnRef.current = new Vector3({
          y: -data.vector.y,
          z: -data.vector.x,
        });
        
        // Send command immediately for better responsiveness
        sendCombinedJoystickCommands();
      }
    }
  };

  const resetLeftJoystick = () => {
    useLeftStick.current = false;
    setLeftInputActive(null);
    leftStickVectorRef.current = new Vector2();
    sendCombinedJoystickCommands();
  };

  const resetRightJoystick = () => {
    useRightStick.current = false;
    setRightInputActive(null);
    rightStickTurnRef.current = new Vector3();
    sendCombinedJoystickCommands();
  };

  // Function to fully recreate joysticks
  const recreateJoysticks = () => {
    if (!enabled || !leftJoystickRef.current || !rightJoystickRef.current)
      return;

    // Clean up existing joysticks
    if (leftJoystickManagerRef.current) {
      leftJoystickManagerRef.current.destroy();
      leftJoystickManagerRef.current = null;
    }

    if (rightJoystickManagerRef.current) {
      rightJoystickManagerRef.current.destroy();
      rightJoystickManagerRef.current = null;
    }

    // Create new joysticks with options optimized for responsiveness
    const options: JoystickManagerOptions = {
      mode: 'static',
      size: 120,
      position: { bottom: '50%', left: '50%' },
      color: '#8A2BE2',
      fadeTime: 100,
      lockX: false,
      lockY: false,
      dynamicPage: true,
      restJoystick: true,
      catchDistance: 150, // Increased catch distance for better responsiveness
      restOpacity: 0.6, // More visible when at rest
    };

    // Create left joystick
    const leftJoystick = nipplejs.create({
      ...options,
      zone: leftJoystickRef.current,
    });
    leftJoystickManagerRef.current = leftJoystick;

    // Create right joystick
    const rightJoystick = nipplejs.create({
      ...options,
      lockX: false,
      lockY: false,
      zone: rightJoystickRef.current,
    });
    rightJoystickManagerRef.current = rightJoystick;

    if (leftJoystickRef.current) createJoystickStyles(leftJoystickRef.current);
    if (rightJoystickRef.current)
      createJoystickStyles(rightJoystickRef.current);

    const interval = 250; // milliseconds

    leftJoystick.on('start', startLeftJoystick);
    leftJoystick.on('move', moveLeftJoystick);
    leftJoystick.on('end', resetLeftJoystick);

    rightJoystick.on('start', startRightJoystick);
    rightJoystick.on('move', moveRightJoystick);
    rightJoystick.on('end', resetRightJoystick);

    // Use a more lightweight approach for interval updates as a backup
    // Direct event handlers already provide immediate updates
    let _intervalIdLeft: NodeJS.Timeout;
    let _intervalIdRight: NodeJS.Timeout;

    // Only use intervals as a fallback to ensure continuous updates
    _intervalIdLeft = setInterval(() => {
      if (useLeftStick.current && leftStickOutputData.current) {
        const data = leftStickOutputData.current;
        leftStickVectorRef.current.set(
          data?.vector.y,
          -(data?.vector.x ?? 0)
        );
        sendCombinedJoystickCommands();
      }
    }, interval);

    _intervalIdRight = setInterval(() => {
      if (useRightStick.current && rightStickOutputData.current) {
        const data = rightStickOutputData.current;
        
        // Always use both x and y vector components directly
        const rightX = data.vector.x || 0;
        const rightY = data.vector.y || 0;
        
        // Set the Vector3 with both components
        rightStickTurnRef.current = new Vector3({
          y: -rightY,
          z: -rightX,
        });
        
        sendCombinedJoystickCommands();
      }
    }, interval);

    return () => {
      clearInterval(_intervalIdLeft);
      clearInterval(_intervalIdRight);
    };
  };

  // Check for dark mode from DOM
  useEffect(() => {
    const checkDarkMode = () => {
      setIsDarkMode(document.documentElement.classList.contains('dark'));
    };

    // Initial check
    checkDarkMode();

    // Observe changes to the dark class
    const observer = new MutationObserver(checkDarkMode);
    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['class'],
    });

    return () => observer.disconnect();
  }, []);

  // Initialize joysticks when component is mounted or enabled changes
  useEffect(() => {
    if (enabled) {
      recreateJoysticks();
    }

    return () => {
      // Clean up on unmount
      if (leftJoystickManagerRef.current) {
        leftJoystickManagerRef.current.destroy();
        leftJoystickManagerRef.current = null;
      }

      if (rightJoystickManagerRef.current) {
        rightJoystickManagerRef.current.destroy();
        rightJoystickManagerRef.current = null;
      }
    };
  }, [enabled, isPose]);

  // Reset joysticks when input state changes
  useEffect(() => {
    // If input state changes (e.g., from mouse to no input), recreate joysticks
    // This helps ensure nipplejs is always ready to handle the first click
    if (leftInputActive === null && rightInputActive === null) {
      // Add a small delay to ensure DOM has updated
      const timer = setTimeout(() => {
        if (enabled) {
          recreateJoysticks();
        }
      }, 50);

      return () => clearTimeout(timer);
    }
  }, [leftInputActive, rightInputActive, enabled]);

  // Apply styles when theme changes
  useEffect(() => {
    if (leftJoystickRef.current) createJoystickStyles(leftJoystickRef.current);
    if (rightJoystickRef.current)
      createJoystickStyles(rightJoystickRef.current);
  }, [isDarkMode]);

  if (!enabled) return null;

  return (
    <>
      <div
        ref={leftJoystickRef}
        className="fixed left-8 bottom-8 w-[120px] h-[120px] pointer-events-auto z-[9999]"
        style={{
          background: isLightTheme()
            ? 'rgba(45, 26, 69, 0.3)'
            : 'rgba(106, 33, 168, 0.3)',
          borderRadius: '60px',
          backdropFilter: 'blur(8px)',
        }}
      />
      <div
        ref={rightJoystickRef}
        className="fixed right-8 bottom-8 w-[120px] h-[120px] pointer-events-auto z-[9999]"
        style={{
          background: isLightTheme()
            ? 'rgba(45, 26, 69, 0.3)'
            : 'rgba(106, 33, 168, 0.3)',
          borderRadius: '60px',
          backdropFilter: 'blur(8px)',
        }}
      />
    </>
  );
}
