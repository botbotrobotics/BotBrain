import { ServiceType } from '@/types/RobotActionTypes';

export interface RobotTopicMapping {
  velocity: string;
  velocityNipple: string;
  temperature: string;
  battery: string;
  laserScan: string;
  odometry: string;
  listener: string;
  map: string;
  jointStates: string;
  camera: string;
  sportModeState: string;
  robotStatus: string;
  thermal: string;
  rgb: string;
  goalPose: string;
  cancelGoal: string;
  diagnostics: string;
}

export interface RobotServiceMapping {
  prompt: string;
  antiCollision: string;
  light: string;
  stop: string;
  pose: string;
  mode: string;
  talk: string;
  delivery: string;
}

export interface RobotCapabilities {
  hasCamera: boolean;
  hasThermalCamera: boolean;
  hasLidar: boolean;
  hasArm: boolean;
  hasGripper: boolean;
  hasSpeaker: boolean;
  hasMicrophone: boolean;
  hasIMU: boolean;
  hasGPS: boolean;
  hasOdometry: boolean;
  maxLinearVelocity: number; // m/s
  maxAngularVelocity: number; // rad/s
  batteryCapacity?: number; // mAh
  supportedModes?: string[];
}

export interface RobotDimensions {
  length: number; // meters
  width: number; // meters
  height: number; // meters
  weight?: number; // kg
  wheelbase?: number; // meters
}

export interface RobotProfile {
  id: string;
  name: string;
  manufacturer: string;
  model: string;
  description: string;
  imageUrl?: string;
  urdfPath?: string;
  capabilities: RobotCapabilities;
  dimensions: RobotDimensions;
  topics: RobotTopicMapping;
  services: RobotServiceMapping;
  defaultSettings?: {
    linearSpeed?: number;
    angularSpeed?: number;
    cameraQuality?: 'low' | 'medium' | 'high';
    diagnosticsEnabled?: boolean;
  };
  customActions?: Array<{
    id: string;
    name: string;
    icon: string;
    service: ServiceType;
    request?: any;
  }>;
  // Actions to hide for this robot type
  hiddenActions?: string[];
}

export type RobotProfileId = 'go2-r1' | 'tita-r1' | 'g1-r1';

export interface RobotProfileRegistry {
  [key: string]: RobotProfile;
}