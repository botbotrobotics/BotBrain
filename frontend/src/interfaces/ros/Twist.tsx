import { Vector3 } from 'roslib';

export interface Twist {
  linear: Vector3;
  angular: Vector3;
}
