import { Header } from './Header';
import { PoseWithCovariance } from './PoseWithCovariance';
import { TwistWithCovariance } from './TwistWithCovariance';

export interface Odometry {
  header: Header;
  child_frame_id: string;
  pose: PoseWithCovariance;
  twist: TwistWithCovariance;
}
