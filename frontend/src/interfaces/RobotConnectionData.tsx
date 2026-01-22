import { Database } from '@/types/database.types';
import * as ROSLIB from 'roslib';

type Robot = Database['public']['Tables']['robots']['Row'];

export default interface RobotConnectionData {
  ros?: ROSLIB.Ros;
  online: boolean;
  connectedRobot?: Robot | null;
}
