import { BatteryState } from './ros/BatteryState';
import { CompressedImage } from './ros/CompressedImage';
import { JointState } from './ros/JointState';
import { LaserScan } from './ros/LaserScan';
import { Temperature } from './ros/Temperature';

export interface RobotSensorData {
  laserScan?: LaserScan;
  jointState?: JointState;
  img?: CompressedImage;
  battery?: BatteryState;
  temperature?: Temperature;
}
