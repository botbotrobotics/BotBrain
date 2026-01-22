import { Time } from './primitives/Time';

export interface Header {
  seq: number;
  stamp: Time;
  frame_id: string;
}
