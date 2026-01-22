'use client';

import { Widget } from './Widget';
import Chat from '../chat';
import { useState } from 'react';
import { useLanguage } from '@/contexts/LanguageContext';

interface ChatWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  title?: string;
}

export function ChatWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 350, height: 400 },
  title = 'Chat',
}: ChatWidgetProps) {
  const [ttsEnabled, setTtsEnabled] = useState(false);
  const { t } = useLanguage();

  return (
    <Widget
      id={id}
      title={
        <div className="flex items-center justify-between w-full">
          <span>{title}</span>
          <div className="flex items-center space-x-2 mr-2">
            <span className="text-sm text-gray-600 dark:text-gray-300">{t('chat', 'tts')}</span>
            <label className="relative inline-flex items-center cursor-pointer">
              <input 
                type="checkbox" 
                className="sr-only peer" 
                checked={ttsEnabled}
                onChange={() => setTtsEnabled(!ttsEnabled)}
              />
              <div className="w-9 h-5 bg-gray-200 dark:bg-botbot-dark peer-focus:outline-none 
                             peer-focus:ring-2 peer-focus:ring-primary dark:peer-focus:ring-botbot-accent 
                             rounded-full peer peer-checked:after:translate-x-full 
                             peer-checked:after:border-white after:content-[''] after:absolute 
                             after:top-[1px] after:left-[2px] after:bg-white after:border-gray-300 
                             after:border after:rounded-full after:h-4 after:w-4 after:transition-all 
                             peer-checked:bg-primary dark:peer-checked:bg-botbot-accent">
              </div>
            </label>
          </div>
        </div>
      }
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      initialPosition={initialPosition}
      initialSize={initialSize}
      minWidth={300}
      minHeight={350}
    >
      <div className="h-full flex flex-col">
        <Chat ttsEnabled={ttsEnabled} />
      </div>
    </Widget>
  );
}
