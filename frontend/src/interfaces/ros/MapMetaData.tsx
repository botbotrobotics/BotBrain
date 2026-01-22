import { Pose } from 'roslib';
import { Time } from './primitives/Time';

export interface MapMetaData {
  map_load_time: Time;
  resolution: number;
  width: number;
  height: number;
  origin: Pose;
}
