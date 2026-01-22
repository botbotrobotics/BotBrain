'use client';

import React from 'react';
import { Wifi, WifiOff, Bot, ExternalLink } from 'lucide-react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import Popup from './ui/popup';
import { useLanguage } from '@/contexts/LanguageContext';
import { useRouter } from 'next/navigation';

interface RobotConnectionPopupProps {
  isOpen: boolean;
  onClose: () => void;
}

export default function RobotConnectionPopup({
  isOpen,
  onClose,
}: RobotConnectionPopupProps) {
  const { connection } = useRobotConnection();
  const { t } = useLanguage();
  const router = useRouter();

  const handleOpenFleet = () => {
    onClose();
    router.push('/fleet');
  };

  return (
    <Popup
      isOpen={isOpen}
      onClose={onClose}
      title={t('connectionPopup', 'title')}
      className="w-full max-w-md"
      customContentClasses="p-6"
    >
      <div className="space-y-4">
        {/* Current connection status */}
        <div className="flex items-center justify-between p-4 bg-gray-50 dark:bg-botbot-dark rounded-lg">
          <div className="flex items-center space-x-3">
            <div className={`w-10 h-10 rounded-full flex items-center justify-center ${
              connection.online ? 'bg-green-100 dark:bg-green-900/20' : 'bg-gray-200 dark:bg-gray-700'
            }`}>
              {connection.online ? (
                <Wifi className="w-5 h-5 text-green-600 dark:text-green-400" />
              ) : (
                <WifiOff className="w-5 h-5 text-gray-500 dark:text-gray-400" />
              )}
            </div>
            <div>
              <p className="text-sm font-medium text-gray-900 dark:text-white">
                {connection.online && connection.connectedRobot
                  ? `Connected to ${connection.connectedRobot.name}`
                  : 'Not Connected'}
              </p>
              <p className="text-xs text-gray-500 dark:text-gray-400">
                {connection.online && connection.connectedRobot
                  ? connection.connectedRobot.address
                  : 'No robot connected'}
              </p>
            </div>
          </div>
        </div>

        {/* Fleet management button */}
        <button
          onClick={handleOpenFleet}
          className="w-full flex items-center justify-between p-4 bg-purple-50 dark:bg-purple-900/20 rounded-lg hover:bg-purple-100 dark:hover:bg-purple-900/30 transition-colors"
        >
          <div className="flex items-center space-x-3">
            <Bot className="w-5 h-5 text-purple-600 dark:text-purple-400" />
            <div className="text-left">
              <p className="text-sm font-medium text-gray-900 dark:text-white">
                Manage Robot Fleet
              </p>
              <p className="text-xs text-gray-500 dark:text-gray-400">
                Add, edit, or connect to your robots
              </p>
            </div>
          </div>
          <ExternalLink className="w-4 h-4 text-gray-400" />
        </button>

        {/* Info message */}
        <div className="text-center py-2">
          <p className="text-sm text-gray-500 dark:text-gray-400">
            Click above to manage your robot connections
          </p>
        </div>
      </div>
    </Popup>
  );
}
