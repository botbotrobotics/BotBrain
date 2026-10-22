import { Database } from '@/types/database.types';
import * as ROSLIB from 'roslib';

type Robot = Database['public']['Tables']['robots']['Row'];

export default interface RobotConnectionData {
  ros?: ROSLIB.Ros;
  /** rosbridge WebSocket URL; its host also serves the camera video (WHEP). */
  url?: string;
  online: boolean;
  connectedRobot?: Robot | null;
}
