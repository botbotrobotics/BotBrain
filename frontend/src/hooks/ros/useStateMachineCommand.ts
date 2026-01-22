import { useState, useCallback, useRef } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';
import {
  StateMachineCommand,
  StateMachineCommandResponse,
  type StateMachineServiceRequest,
  type StateMachineServiceResponse
} from '@/types/StateMachine';

export function useStateMachineCommand() {
  const { connection } = useRobotConnection();
  const [isExecuting, setIsExecuting] = useState(false);
  const [lastError, setLastError] = useState<string | null>(null);
  const serviceClientRef = useRef<ROSLIB.Service<unknown, unknown> | null>(null);

  // Initialize service client
  const getServiceClient = useCallback(() => {
    if (!connection.ros || !connection.online) {
      return null;
    }

    if (!serviceClientRef.current) {
      serviceClientRef.current = new ROSLIB.Service({
        ros: connection.ros,
        name: '/state_machine/command',
        serviceType: 'bot_custom_interfaces/srv/StateMachine',
      });
    }

    return serviceClientRef.current;
  }, [connection.ros, connection.online]);

  // Execute command
  const executeCommand = useCallback(
    (nodeName: string, command: StateMachineCommand): Promise<StateMachineServiceResponse> => {
      return new Promise((resolve, reject) => {
        const serviceClient = getServiceClient();

        if (!serviceClient) {
          const error = 'ROS not connected';
          setLastError(error);
          reject(new Error(error));
          return;
        }

        setIsExecuting(true);
        setLastError(null);

        const request: StateMachineServiceRequest = {
          node: nodeName,
          command: command,
        };

        serviceClient.callService(
          request as any,
          (response: any) => {
            setIsExecuting(false);
            const result = response as StateMachineServiceResponse;

            if (result.success) {
              resolve(result);
            } else {
              const errorMsg = getErrorMessage(result.result);
              setLastError(errorMsg);
              reject(new Error(errorMsg));
            }
          },
          (error: string) => {
            setIsExecuting(false);
            const errorMsg = `Service call failed: ${error}`;
            setLastError(errorMsg);
            reject(new Error(errorMsg));
          }
        );
      });
    },
    [getServiceClient]
  );

  // Helper to activate a node
  const activateNode = useCallback(
    (nodeName: string) => executeCommand(nodeName, StateMachineCommand.ACTIVATE_NODE),
    [executeCommand]
  );

  // Helper to deactivate a node
  const deactivateNode = useCallback(
    (nodeName: string) => executeCommand(nodeName, StateMachineCommand.DEACTIVATE_NODE),
    [executeCommand]
  );

  // Helper to restart a node
  const restartNode = useCallback(
    (nodeName: string) => executeCommand(nodeName, StateMachineCommand.RESTART_NODE),
    [executeCommand]
  );

  return {
    executeCommand,
    activateNode,
    deactivateNode,
    restartNode,
    isExecuting,
    lastError,
    isConnected: connection.online,
  };
}

// Helper function to convert response code to error message
function getErrorMessage(result: number): string {
  switch (result) {
    case StateMachineCommandResponse.SUCCESS:
      return 'Command executed successfully';
    case StateMachineCommandResponse.FAILURE:
      return 'Command execution failed';
    case StateMachineCommandResponse.INVALID_NODE:
      return 'Invalid node name';
    case StateMachineCommandResponse.INVALID_COMMAND:
      return 'Invalid command';
    case StateMachineCommandResponse.INVALID_STATE:
      return 'Invalid state for this operation';
    default:
      return 'Unknown error occurred';
  }
}
