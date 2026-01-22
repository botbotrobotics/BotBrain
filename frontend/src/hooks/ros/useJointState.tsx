'use client';

import { useState, useEffect } from 'react';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { JointState } from '@/interfaces/ros/JointState';

export default function useJointState(useDummy: boolean = false): JointState {
  const { connection } = useRobotConnection();

  const [jointState, setJointState] = useState<JointState>({
    header: {
      frame_id: '',
      seq: 0,
      stamp: {
        nanosec: 0,
        sec: 0,
      },
    },
    name: [],
    position: [],
    velocity: [],
    effort: [],
  });

  useEffect(() => {
    if (!connection.ros || !connection.online) return;

    const topicFactory: ROSTopicFactory = new ROSTopicFactory(
      connection.ros,
      useDummy
    );
    
    // Use optimized subscribe with lower latency settings
    const jointStateTopic = topicFactory.createAndSubscribeTopic<JointState>(
      'jointStates',
      (msg) => {
        // Use functional state update to ensure we're always working with latest state
        setJointState(msg);
      }
    );

    // Cleanup: desinscreve do tópico quando o componente é desmontado ou a conexão muda
    return () => {
      jointStateTopic.unsubscribe();
    };
  }, [connection.ros, connection.online]);

  return jointState;
}
