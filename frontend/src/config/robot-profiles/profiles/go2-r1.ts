import { RobotProfile } from '../types';

export const Go2R1Profile: RobotProfile = {
  id: 'go2-r1',
  name: 'Go2-R1',
  manufacturer: 'Unitree',
  model: 'Go2 R1',
  description: 'Unitree Go2 quadruped robot with advanced mobility and perception capabilities',
  imageUrl: '/images/robots/go2-r1.png',
  urdfPath: '/models/go2/go2.urdf',
  
  capabilities: {
    hasCamera: true,
    hasThermalCamera: true,
    hasLidar: true,
    hasArm: false,
    hasGripper: false,
    hasSpeaker: true,
    hasMicrophone: true,
    hasIMU: true,
    hasGPS: false,
    hasOdometry: true,
    maxLinearVelocity: 3.5, // m/s
    maxAngularVelocity: 2.0, // rad/s
    batteryCapacity: 8000, // mAh
    supportedModes: ['walk', 'stand', 'sport', 'balance', 'dance'],
  },
  
  dimensions: {
    length: 0.70, // meters
    width: 0.28, // meters
    height: 0.40, // meters
    weight: 15, // kg
    wheelbase: 0.45, // meters
  },
  
  topics: {
    velocity: 'cmd_vel_joy',
    velocityNipple: 'cmd_vel_nipple',
    temperature: 'imu_temp',
    battery: 'battery',
    laserScan: 'scan',
    odometry: 'odom',
    listener: 'listener',
    map: 'map',
    jointStates: 'joint_states',
    camera: 'compressed_camera',
    sportModeState: 'lf/sportmodestate',
    robotStatus: 'robot_status',
    thermal: 'viz/camcam/thermal/compressed_image',
    rgb: 'viz/camcam/rgb/compressed_image',
    goalPose: 'goal_pose',
    cancelGoal: 'cancel_goal',
    diagnostics: 'diagnostic_stats',
  },
  
  services: {
    prompt: 'rosa_prompt',
    antiCollision: 'obstacle_avoidance',
    light: 'light_control',
    stop: 'emergency_stop',
    pose: 'pose',
    mode: 'mode',
    talk: 'talk',
    delivery: 'delivery_control',
  },
  
  defaultSettings: {
    linearSpeed: 1.0,
    angularSpeed: 0.5,
    cameraQuality: 'high',
    diagnosticsEnabled: true,
  },
  
  customActions: [
    {
      id: 'stand',
      name: 'Stand',
      icon: 'ArrowUp',
      service: 'pose',
      request: { pose: 'stand' },
    },
    {
      id: 'sit',
      name: 'Sit',
      icon: 'ArrowDown',
      service: 'pose',
      request: { pose: 'sit' },
    },
    {
      id: 'balance',
      name: 'Balance Mode',
      icon: 'Activity',
      service: 'mode',
      request: { mode: 'balance' },
    },
    {
      id: 'sport',
      name: 'Sport Mode',
      icon: 'Zap',
      service: 'mode',
      request: { mode: 'sport' },
    },
  ],
};