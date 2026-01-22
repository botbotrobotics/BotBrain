import useRobotProfile from '@/hooks/ros/useRobotProfile';
import { useRobotConnection } from '../contexts/RobotConnectionContext';
import { MessageSquare, Circle } from 'lucide-react';
import React from 'react';
import { useLanguage } from '@/contexts/LanguageContext';

export default function ChatHeader() {
  const { connection } = useRobotConnection();
  const profile = useRobotProfile();
  const { t } = useLanguage();

  return (
    <div className="h-auto py-2 border-[#e6d6f5] dark:border-botbot-darker flex items-center justify-between px-4">
      <div className="flex items-center gap-3">
        <div className="w-10 h-10 rounded-full bg-[#821db7] dark:bg-botbot-dark flex items-center justify-center text-white">
          <MessageSquare className="w-5 h-5" />
        </div>
        <div>
          <h3 className="font-medium">{t('chatHeader', 'robot')} {profile.name}</h3>
          <div className="flex items-center gap-2">
            {connection.online && (
              <>
                <Circle className="w-3 h-3 fill-[#22c55e] text-[#22c55e]" />
                <span className="text-sm text-gray-500 dark:text-white">
                  {t('chatHeader', 'online')}
                </span>
              </>
            )}

            {!connection.online && (
              <>
                <Circle className="w-3 h-3 fill-[#ccc] text-[#ccc]" />
                <span className="text-sm text-gray-500 dark:text-white">
                  {t('chatHeader', 'offline')}
                </span>
              </>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
