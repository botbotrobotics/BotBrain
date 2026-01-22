import { useHeader } from '@/contexts/HeaderContext';
import { MoveKeyType } from '@/types/MoveKeyType';
import { useEffect } from 'react';

type KeyHandler = (key: MoveKeyType, event: KeyboardEvent) => void;

const keys: MoveKeyType[] = [
  'q',
  'e',
  'w',
  'a',
  's',
  'd',
  'ArrowUp',
  'ArrowDown',
  'ArrowLeft',
  'ArrowRight',
];

type KeyEvent = 'keydown' | 'keyup';

export function useKeyboardInput(keyEventType: KeyEvent, handler: KeyHandler) {
  const { joystickEnabled } = useHeader();

  useEffect(() => {
    if (!joystickEnabled) return;

    const keyEventHandler = (event: KeyboardEvent) => {
      if (keys.includes(event.key as MoveKeyType)) {
        handler(event.key as MoveKeyType, event);
      }
    };

    window.addEventListener(keyEventType, keyEventHandler);

    return () => {
      window.removeEventListener(keyEventType, keyEventHandler);
    };
  }, [handler]);
}
