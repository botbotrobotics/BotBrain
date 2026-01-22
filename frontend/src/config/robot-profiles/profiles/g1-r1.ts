import { RobotProfile } from '../types';

export const G1R1Profile: RobotProfile = {
  id: 'g1-r1',
  name: 'G1-R1',
  manufacturer: 'Unitree',
  model: 'G1',
  description: 'Unitree G1 humanoid robot with advanced bipedal locomotion and manipulation capabilities',
  imageUrl: '/images/robots/g1-r1.png',
  urdfPath: '/robot-models/g1/robot.urdf',

  capabilities: {
    hasCamera: true,
    hasThermalCamera: false,
    hasLidar: true,
    hasArm: true,
    hasGripper: true,
    hasSpeaker: true,
    hasMicrophone: true,
    hasIMU: true,
    hasGPS: false,
    hasOdometry: true,
    maxLinearVelocity: 2.0, // m/s (humanoid robots typically move slower than quadrupeds)
    maxAngularVelocity: 1.5, // rad/s
    batteryCapacity: 10000, // mAh (estimated)
    supportedModes: ['stand', 'walk', 'balance', 'manipulation', 'dance'],
  },

  dimensions: {
    length: 0.50, // meters (depth)
    width: 0.45, // meters (shoulder width)
    height: 1.65, // meters (typical humanoid height)
    weight: 45, // kg (estimated for humanoid)
    wheelbase: 0.30, // meters (stance width)
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
    linearSpeed: 0.5, // Slower default for humanoid
    angularSpeed: 0.3,
    cameraQuality: 'high',
    diagnosticsEnabled: true,
  },

  customActions: [
    {
      id: 'damping',
      name: 'Damping',
      icon: 'Waves', // Wave icon representing smooth, dampened movement
      service: 'mode',
      request: { mode: 'damping' },
    },
    {
      id: 'zero-torque',
      name: 'Zero Torque',
      icon: 'CircleX', // X in circle representing zero/disabled torque
      service: 'mode',
      request: { mode: 'zero_torque' },
    },
    {
      id: 'stand',
      name: 'Stand',
      icon: 'User',
      service: 'pose',
      request: { pose: 'stand' },
    },
    {
      id: 'balance',
      name: 'Balance Mode',
      icon: 'Activity',
      service: 'mode',
      request: { mode: 'balance' },
    },
    {
      id: 'manipulation',
      name: 'Manipulation Mode',
      icon: 'Hand',
      service: 'mode',
      request: { mode: 'manipulation' },
    },
    {
      id: 'wave',
      name: 'Wave',
      icon: 'HandMetal',
      service: 'pose',
      request: { pose: 'wave' },
    },
    {
      id: 'dance',
      name: 'Dance',
      icon: 'Music',
      service: 'mode',
      request: { mode: 'dance' },
    },
  ],

  // Hide actions that are not available for G1 humanoid robot
  hiddenActions: ['antiCollisionOn', 'antiCollisionOff', 'lightOn', 'lightOff', 'poseOn', 'poseOff', 'sit'],
};