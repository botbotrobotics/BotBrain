import { useState, useEffect, useRef } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';
import type { NodeStatus } from '@/types/StateMachine';

export function useStateMachineStatus() {
  const { connection } = useRobotConnection();
  const [nodeStatuses, setNodeStatuses] = useState<NodeStatus[]>([]);
  const [lastUpdate, setLastUpdate] = useState<Date | null>(null);
  const topicRef = useRef<ROSLIB.Topic<unknown> | null>(null);

  useEffect(() => {
    if (!connection.ros || !connection.online) {
      setNodeStatuses([]);
      setLastUpdate(null);
      return;
    }

    // Create topic subscriber
    const statusTopic = new ROSLIB.Topic({
      ros: connection.ros,
      name: '/state_machine/status',
      messageType: 'bot_custom_interfaces/msg/StatusArray',
    });

    // Subscribe to messages
    statusTopic.subscribe((message: any) => {
      // Handle both 'nodes' and 'containers' field names
      const containers = message.containers || message.nodes;
      if (containers && Array.isArray(containers)) {
        // Map containers to NodeStatus format
        const statuses = containers.map((container: any) => ({
          name: container.name || '',
          displayName: container.display_name || container.name || '',
          state: container.status || container.state || 'unknown',
          active: container.status?.toLowerCase() === 'active' ||
                  container.state?.toLowerCase() === 'active' ||
                  false
        }));
        setNodeStatuses(statuses);
        setLastUpdate(new Date());
      }
    });

    topicRef.current = statusTopic;

    // Cleanup on unmount
    return () => {
      if (topicRef.current) {
        topicRef.current.unsubscribe();
        topicRef.current = null;
      }
    };
  }, [connection.ros, connection.online]);

  return {
    nodeStatuses,
    lastUpdate,
    isConnected: connection.online,
  };
}
