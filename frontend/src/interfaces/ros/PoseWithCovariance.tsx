import { Pose } from 'roslib';

export interface PoseWithCovariance {
  pose: Pose;
  covariance: number[];
}
