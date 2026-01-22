'use client';

import { useState, useCallback } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { ROSServiceFactory } from '@/utils/ros/topics-and-services';
import { auditLogger } from '@/utils/audit-logger';

interface DeliveryControlOptions {
  onSuccess?: (response: any) => void;
  onError?: (error: string) => void;
}

export default function useDeliveryControl() {
  const { connection } = useRobotConnection();
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const callDeliveryService = useCallback(
    (open: boolean, options?: DeliveryControlOptions) => {
      if (!connection.ros || !connection.ros.isConnected) {
        const errorMsg = 'Unable to control delivery: No ROS connection';
        setError(errorMsg);
        options?.onError?.(errorMsg);
        return;
      }

      setIsLoading(true);
      setError(null);

      // Log the command
      const robotId = connection.connectedRobot?.id;
      const robotName = connection.connectedRobot?.name;
      auditLogger.logCommand('delivery', robotId, robotName, {
        action: open ? 'open' : 'close'
      });

      const serviceFactory = new ROSServiceFactory(connection.ros);

      serviceFactory.callServiceWithCallback({
        typeKey: 'delivery',
        request: {
          data: open // true = open, false = close
        },
        callback: (response: { success: boolean; message: string }) => {
          setIsLoading(false);
          if (response.success) {
            setIsOpen(open);
            options?.onSuccess?.(response);

            // Log successful action
            auditLogger.logButtonPress(
              open ? 'deliveryOpen' : 'deliveryClose',
              robotId,
              robotName,
              { success: true, message: response.message }
            );
          } else {
            const errorMsg = response.message || 'Delivery control failed';
            setError(errorMsg);
            options?.onError?.(errorMsg);

            // Log failed action
            auditLogger.log({
              event_type: 'command',
              event_action: 'command_sent',
              robot_id: robotId,
              robot_name: robotName,
              event_details: {
                command: 'delivery',
                action: open ? 'open' : 'close',
                success: false,
                error: errorMsg,
                response
              }
            });
          }
        },
        failedCallback: (error?: string) => {
          setIsLoading(false);
          const errorMsg = error || 'Failed to call delivery service';
          setError(errorMsg);
          options?.onError?.(errorMsg);

          // Log error
          auditLogger.log({
            event_type: 'command',
            event_action: 'command_sent',
            robot_id: robotId,
            robot_name: robotName,
            event_details: {
              command: 'delivery',
              action: open ? 'open' : 'close',
              success: false,
              error: errorMsg
            }
          });
        }
      });
    },
    [connection.ros, connection.connectedRobot]
  );

  const openDelivery = useCallback(
    (options?: DeliveryControlOptions) => {
      callDeliveryService(true, options);
    },
    [callDeliveryService]
  );

  const closeDelivery = useCallback(
    (options?: DeliveryControlOptions) => {
      callDeliveryService(false, options);
    },
    [callDeliveryService]
  );

  return {
    openDelivery,
    closeDelivery,
    isOpen,
    isLoading,
    error
  };
}