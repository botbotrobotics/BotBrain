'use client';

import Container from './ui/container';
import { MenuActionType } from '../types/RobotActionTypes';
import useMenuActions from '@/hooks/useMenuActions';
import ChatTTS from './chat-tts';
import { Volume2 } from 'lucide-react';
import { useLanguage } from '@/contexts/LanguageContext';

export default function ChatContainerTTSOnly() {
  const menuActions = useMenuActions();
  const { t } = useLanguage();
  const actions: MenuActionType[] = [];

  return (
    <Container
      title={
        <div className="flex items-center">
          <Volume2 className="mr-2 w-5 h-5" />
          <span>{t('chat', 'tts')} {t('chat', 'title')}</span>
        </div>
      }
      className="h-full flex flex-col"
      customContentClasses="flex-1 min-h-0 p-0"
      actions={actions}
    >
      <ChatTTS />
    </Container>
  );
}