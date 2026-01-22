import { Twist } from './Twist';

export interface TwistWithCovariance {
  twist: Twist;
  covariance: number[];
}
