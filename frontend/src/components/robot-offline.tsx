import { BotOff } from 'lucide-react';
import React from 'react';
import { useLanguage } from '@/contexts/LanguageContext';
import { cn } from '@/utils/cn';

export default function RobotOffline({
  useBorder,
  msg,
}: {
  useBorder: boolean;
  msg?: string;
}) {
  const { t } = useLanguage();

  return (
    // <div className="w-full h-full border rounded-default-border border-gray-300 flex flex-col items-center justify-center text-md text-center text-gray-600 dark:text-white">
    <div
      className={cn(
        'w-full h-full flex flex-col items-center justify-center text-md text-center text-gray-600 dark:text-white',
        useBorder ? 'border rounded-default-border border-gray-300' : ''
      )}
    >
      <BotOff className="w-8 h-8 mb-2" />
      <span className="body-text mb-1">
        {msg ?? t('robotOffline', 'connectionError')}
      </span>
    </div>
  );
}
