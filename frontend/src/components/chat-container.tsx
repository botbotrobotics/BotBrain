'use client';

import Container from './ui/container';
import { MenuActionType } from '../types/RobotActionTypes';
import useMenuActions from '@/hooks/useMenuActions';
import Chat from './chat';
import { MessageSquare, Volume2 } from 'lucide-react';
import { useLanguage } from '@/contexts/LanguageContext';
import { useState } from 'react';
import { cn } from '@/utils/cn';

export default function ChatContainer() {
  const menuActions = useMenuActions();
  const { t } = useLanguage();
  const [ttsEnabled, setTtsEnabled] = useState(false);
  const actions: MenuActionType[] = [];

  return (
    <Container
      title={
        <div className="flex items-center">
          <MessageSquare className="mr-2 w-5 h-5" />
          <span>{t('chat', 'title')}</span>
        </div>
      }
      className="h-full flex flex-col justify-between mb-0 pb-0"
      customContentClasses="h-full mb-0 pt-1 overflow-auto"
      actions={actions}
      customActions={
        <button
          onClick={() => setTtsEnabled(!ttsEnabled)}
          className={cn(
            "flex items-center gap-2 px-3 py-1.5 rounded-menu-btn-border",
            "transition-all duration-200 text-sm font-medium",
            ttsEnabled 
              ? "bg-primary text-white hover:bg-primary/90 dark:bg-botbot-accent dark:hover:bg-botbot-accent/90" 
              : "bg-gray-100 text-gray-700 hover:bg-gray-200 dark:bg-botbot-darker dark:text-gray-300 dark:hover:bg-botbot-dark"
          )}
          title={ttsEnabled ? "TTS Enabled" : "TTS Disabled"}
        >
          <Volume2 className="w-4 h-4" />
          <span>{t('chat', 'tts')}</span>
        </button>
      }
    >
      <Chat ttsEnabled={ttsEnabled} />
    </Container>
  );
}
