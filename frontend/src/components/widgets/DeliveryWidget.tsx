'use client';

import { Widget } from './Widget';
import { useState, useEffect } from 'react';
import { useDashboard } from '@/contexts/DashboardContext';
import { useLanguage } from '@/contexts/LanguageContext';
import { PackageOpen, PackageX, Loader2, AlertCircle } from 'lucide-react';
import { cn } from '@/utils/cn';
import useDeliveryControl from '@/hooks/ros/useDeliveryControl';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { useNotifications } from '@/contexts/NotificationsContext';

interface DeliveryWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  title?: string;
  props?: {
    title?: string;
  };
}

export function DeliveryWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 300, height: 200 },
  title = 'Delivery',
  props,
}: DeliveryWidgetProps) {
  const { updateWidgetProps } = useDashboard();
  const { t } = useLanguage();
  const { connection } = useRobotConnection();
  const { dispatch } = useNotifications();

  // ROS delivery control hook
  const { openDelivery, closeDelivery, isOpen, isLoading, error } = useDeliveryControl();

  // Use props from dashboard context if available, otherwise use defaults
  const [currentTitle, setCurrentTitle] = useState(props?.title || title);
  const [showSettings, setShowSettings] = useState(false);
  const [currentAction, setCurrentAction] = useState<'opening' | 'closing' | null>(null);

  // Update dashboard context when settings change
  const updateSettings = (updates: any) => {
    const newProps = {
      ...props,
      ...updates,
    };
    updateWidgetProps(id, newProps);
    return newProps;
  };

  const handleTitleChange = (newTitle: string) => {
    setCurrentTitle(newTitle);
    updateSettings({ title: newTitle });
  };

  const handleOpen = () => {
    setCurrentAction('opening');
    openDelivery({
      onSuccess: (response) => {
        setCurrentAction(null);
        dispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'success',
            title: 'Delivery Opened',
            message: response.message || 'Delivery compartment opened successfully',
          }
        });
      },
      onError: (error) => {
        setCurrentAction(null);
        dispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'error',
            title: 'Failed to Open',
            message: error,
          }
        });
      }
    });
  };

  const handleClose = () => {
    setCurrentAction('closing');
    closeDelivery({
      onSuccess: (response) => {
        setCurrentAction(null);
        dispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'success',
            title: 'Delivery Closed',
            message: response.message || 'Delivery compartment closed successfully',
          }
        });
      },
      onError: (error) => {
        setCurrentAction(null);
        dispatch({
          type: 'ADD_NOTIFICATION',
          payload: {
            type: 'error',
            title: 'Failed to Close',
            message: error,
          }
        });
      }
    });
  };

  const isConnected = connection.ros && connection.ros.isConnected;

  return (
    <Widget
      id={id}
      title={currentTitle}
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      initialPosition={initialPosition}
      initialSize={initialSize}
      minWidth={250}
      minHeight={180}
      onSettingsClick={() => setShowSettings(!showSettings)}
    >
      <div className="h-full flex flex-col p-4">
        {showSettings && (
          <div className="w-full bg-gray-100 dark:bg-botbot-darker rounded-md p-3 mb-4 text-sm">
            <div className="mb-3">
              <label className="block text-gray-700 dark:text-gray-300 mb-1">
                {t('myUI', 'widgetName')}
              </label>
              <input
                type="text"
                value={currentTitle}
                onChange={(e) => handleTitleChange(e.target.value)}
                className="w-full p-2 border rounded-md dark:bg-botbot-dark dark:border-botbot-darker"
                placeholder="Enter widget name"
              />
            </div>
          </div>
        )}

        <div className="flex-1 flex flex-col items-center justify-center">
          {/* Connection Status Warning */}
          {!isConnected && (
            <div className="mb-4 flex items-center gap-2 text-amber-600 dark:text-amber-400 text-sm">
              <AlertCircle className="w-4 h-4" />
              <span>No robot connection</span>
            </div>
          )}

          {/* Error Display */}
          {error && (
            <div className="mb-4 px-3 py-2 bg-red-100 dark:bg-red-900/30 text-red-700 dark:text-red-400 rounded-lg text-sm">
              {error}
            </div>
          )}

          {/* Status Indicator */}
          <div className="mb-6 text-center">
            <div className={cn(
              "inline-flex items-center px-3 py-1 rounded-full text-sm font-medium",
              isLoading
                ? "bg-blue-100 dark:bg-blue-900/30 text-blue-800 dark:text-blue-300"
                : isOpen
                ? "bg-green-100 dark:bg-green-900/30 text-green-800 dark:text-green-300"
                : "bg-gray-100 dark:bg-gray-800 text-gray-700 dark:text-gray-300"
            )}>
              {isLoading && currentAction ? (
                <>
                  <Loader2 className="w-3 h-3 mr-2 animate-spin" />
                  {currentAction === 'opening' ? 'Opening...' : 'Closing...'}
                </>
              ) : (
                <>
                  <span className={cn(
                    "w-2 h-2 rounded-full mr-2",
                    isOpen ? "bg-green-500 animate-pulse" : "bg-gray-500"
                  )} />
                  {isOpen ? 'Open' : 'Closed'}
                </>
              )}
            </div>
          </div>

          {/* Buttons */}
          <div className="flex gap-4 w-full max-w-xs">
            <button
              onClick={handleOpen}
              disabled={isOpen || isLoading || !isConnected}
              className={cn(
                "flex-1 flex items-center justify-center gap-2 px-4 py-3",
                "rounded-lg font-medium transition-all duration-200",
                "border-2",
                (isOpen || isLoading || !isConnected)
                  ? "bg-gray-100 dark:bg-gray-800 border-gray-300 dark:border-gray-700 text-gray-400 dark:text-gray-600 cursor-not-allowed"
                  : "bg-green-500 hover:bg-green-600 active:bg-green-700 border-green-600 dark:border-green-700 text-white shadow-sm hover:shadow-md"
              )}
            >
              {isLoading && !isOpen ? (
                <Loader2 className="w-5 h-5 animate-spin" />
              ) : (
                <PackageOpen className="w-5 h-5" />
              )}
              <span>Open</span>
            </button>

            <button
              onClick={handleClose}
              disabled={!isOpen || isLoading || !isConnected}
              className={cn(
                "flex-1 flex items-center justify-center gap-2 px-4 py-3",
                "rounded-lg font-medium transition-all duration-200",
                "border-2",
                (!isOpen || isLoading || !isConnected)
                  ? "bg-gray-100 dark:bg-gray-800 border-gray-300 dark:border-gray-700 text-gray-400 dark:text-gray-600 cursor-not-allowed"
                  : "bg-red-500 hover:bg-red-600 active:bg-red-700 border-red-600 dark:border-red-700 text-white shadow-sm hover:shadow-md"
              )}
            >
              {isLoading && isOpen ? (
                <Loader2 className="w-5 h-5 animate-spin" />
              ) : (
                <PackageX className="w-5 h-5" />
              )}
              <span>Close</span>
            </button>
          </div>
        </div>
      </div>
    </Widget>
  );
}