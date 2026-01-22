'use client';

import { useEffect, useState } from 'react';
import {
  AlertCircle,
  AlertTriangle,
  CheckCircle2,
  Info,
  X,
  BotIcon as Robot,
} from 'lucide-react';
import { AnimatePresence, motion } from 'framer-motion';
import {
  useNotifications,
  type Notification,
} from '@/contexts/NotificationsContext';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useLanguage } from '@/contexts/LanguageContext';

const notificationIcons = {
  info: Info,
  success: CheckCircle2,
  warning: AlertTriangle,
  error: AlertCircle,
};

const notificationColors = {
  info: 'bg-[#9a89ff]',
  success: 'bg-[#4ade80]',
  warning: 'bg-[#fbe38e]',
  error: 'bg-[#feaeae]',
};

export function NotificationItem({
  notification,
  onRemove,
}: {
  notification: Notification;
  onRemove: () => void;
}) {
  const Icon = notificationIcons[notification.type];

  return (
    <motion.div
      initial={{ opacity: 0, x: 50 }}
      animate={{ opacity: 1, x: 0 }}
      exit={{ opacity: 0, x: 50 }}
      className="flex items-start gap-3 p-4 bg-white dark:bg-botbot-dark rounded-lg shadow-lg border border-gray-100 dark:border-black"
    >
      <div
        className={`p-2 rounded-full ${notificationColors[notification.type]}`}
      >
        <Icon className="w-4 h-4 text-white" />
      </div>
      <div className="flex-1 min-w-0">
        <h4 className="text-sm font-medium">{notification.title}</h4>
        <p className="text-xs text-gray-500 dark:text-white mt-1">
          {notification.message}
        </p>
        <span className="text-xs text-gray-400 dark:text-botbot-accent mt-2 block">
          {new Date(notification.timestamp).toLocaleTimeString()}
        </span>
      </div>
      <button
        onClick={(e) => {
          e.stopPropagation();
          onRemove();
        }}
        className="p-1 hover:bg-gray-100 dark:hover:bg-botbot-dark rounded-full transition-colors"
      >
        <X className="w-4 h-4" />
        <span className="sr-only">Remove notification</span>
      </button>
    </motion.div>
  );
}

export function NotificationsPanel() {
  const { state, dispatch } = useNotifications();
  const [isVisible, _setIsVisible] = useState(false);
  const { connection } = useRobotConnection();
  const { t } = useLanguage();

  // Mark all as read when panel becomes visible
  useEffect(() => {
    if (isVisible) {
      state.notifications.forEach((notification) => {
        if (!notification.read) {
          dispatch({ type: 'MARK_AS_READ', payload: notification.id });
        }
      });
    }
  }, [isVisible, state.notifications, dispatch]);

  return (
    <AnimatePresence>
      <motion.div
        initial={{ opacity: 0, x: '100%' }}
        animate={{ opacity: 1, x: 0 }}
        exit={{ opacity: 0, x: '100%' }}
        transition={{ type: 'spring', damping: 20, stiffness: 300 }}
        className="fixed inset-y-0 right-0 w-80 bg-[#f0d5ff] dark:bg-botbot-darker shadow-2xl z-[100] flex flex-col"
      >
        {/* Header */}
        <div className="p-4 border-b dark:border-botbot-dark bg-white dark:bg-botbot-dark">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Robot className="w-5 h-5 text-[#821db7] dark:text-white" />
              <h3 className="text-lg font-semibold">{t('notifications', 'title')}</h3>
            </div>
            <button
              onClick={() => dispatch({ type: 'CLEAR_ALL' })}
              className="text-xs text-gray-500 hover:text-gray-700 dark:text-white dark:hover:text-gray-300"
            >
              {t('notifications', 'markAllAsRead')}
            </button>
          </div>
        </div>

        {/* Notifications list */}
        <div className="flex-1 overflow-y-auto p-4 space-y-3">
          <AnimatePresence mode="popLayout">
            {state.notifications.length === 0 ? (
              <motion.div
                initial={{ opacity: 0 }}
                animate={{ opacity: 1 }}
                className="flex flex-col items-center justify-center h-full text-gray-500 dark:text-white"
              >
                <Robot className="w-12 h-12 mb-4 opacity-50" />
                <p className="text-sm">{t('notifications', 'noNotifications')}</p>
                <p className="text-xs mt-2 text-center max-w-[200px]">
                  Robot events and system notifications will appear here
                </p>
              </motion.div>
            ) : (
              state.notifications.map((notification) => (
                <NotificationItem
                  key={notification.id}
                  notification={notification}
                  onRemove={() =>
                    dispatch({
                      type: 'REMOVE_NOTIFICATION',
                      payload: notification.id,
                    })
                  }
                />
              ))
            )}
          </AnimatePresence>
        </div>

        {/* Footer */}
        <div className="p-4 border-t dark:border-botbot-dark bg-white dark:bg-botbot-dark">
          <p className="text-xs text-center text-gray-500 dark:text-white">
            {connection.online 
              ? connection.connectedRobot 
                ? `Connected to ${connection.connectedRobot.name}`
                : 'Connected to robot' 
              : 'Not connected to BotBot'}
          </p>
        </div>
      </motion.div>
    </AnimatePresence>
  );
}
