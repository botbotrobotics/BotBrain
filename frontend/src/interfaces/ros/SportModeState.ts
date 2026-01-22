import { Header } from './Header';

export interface SportModeState {
  header: Header;
  gait_type: number;
  foot_raise_height: number;
  position: number[]; // x, y, z in odometry frame
  body_height: number;
  velocity: number[]; // vx, vy, vz in odometry frame
  yaw_speed: number;
  mode: number;
  foot_position_body?: number[]; // Optional: positions of the feet in body frame
  foot_speed_body?: number[]; // Optional: velocities of the feet in body frame
}
