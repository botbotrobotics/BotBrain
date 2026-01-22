import { RobotProfile } from '../types';

export const TitaR1Profile: RobotProfile = {
  id: 'tita-r1',
  name: 'Tita-R1',
  manufacturer: 'Custom',
  model: 'Tita R1',
  description: 'Custom robot platform with modular capabilities and advanced sensors',
  imageUrl: '/images/robots/tita-r1.png',
  urdfPath: '/models/tita/tita.urdf',
  
  capabilities: {
    hasCamera: true,
    hasThermalCamera: false,
    hasLidar: true,
    hasArm: true,
    hasGripper: true,
    hasSpeaker: true,
    hasMicrophone: true,
    hasIMU: true,
    hasGPS: true,
    hasOdometry: true,
    maxLinearVelocity: 2.0, // m/s
    maxAngularVelocity: 1.5, // rad/s
    batteryCapacity: 12000, // mAh
    supportedModes: ['manual', 'autonomous', 'follow', 'patrol'],
  },
  
  dimensions: {
    length: 0.60, // meters
    width: 0.40, // meters
    height: 0.80, // meters
    weight: 25, // kg
    wheelbase: 0.35, // meters
  },
  
  // These will be mapped later as mentioned by the user
  // For now, using placeholder values that can be updated
  topics: {
    velocity: 'tita/cmd_vel',
    velocityNipple: 'tita/cmd_vel_manual',
    temperature: 'tita/temperature',
    battery: 'tita/battery_state',
    laserScan: 'tita/laser_scan',
    odometry: 'tita/odometry',
    listener: 'tita/audio_listener',
    map: 'tita/map',
    jointStates: 'tita/joint_states',
    camera: 'tita/camera/image_compressed',
    sportModeState: 'tita/mode_state',
    robotStatus: 'tita/status',
    thermal: 'tita/thermal/image_compressed',
    rgb: 'tita/rgb/image_compressed',
    goalPose: 'tita/move_base/goal',
    cancelGoal: 'tita/move_base/cancel',
    diagnostics: 'tita/diagnostics',
  },
  
  services: {
    prompt: 'tita/ai_prompt',
    antiCollision: 'tita/collision_avoidance',
    light: 'tita/led_control',
    stop: 'tita/emergency_stop',
    pose: 'tita/set_pose',
    mode: 'tita/set_mode',
    talk: 'tita/text_to_speech',
    delivery: 'tita/delivery_control',
  },
  
  defaultSettings: {
    linearSpeed: 0.8,
    angularSpeed: 0.4,
    cameraQuality: 'medium',
    diagnosticsEnabled: true,
  },
  
  customActions: [
    {
      id: 'home',
      name: 'Home Position',
      icon: 'Home',
      service: 'pose',
      request: { pose: 'home' },
    },
    {
      id: 'arm_up',
      name: 'Arm Up',
      icon: 'ArrowUp',
      service: 'pose',
      request: { pose: 'arm_up' },
    },
    {
      id: 'arm_down',
      name: 'Arm Down',
      icon: 'ArrowDown',
      service: 'pose',
      request: { pose: 'arm_down' },
    },
    {
      id: 'gripper_open',
      name: 'Open Gripper',
      icon: 'Maximize2',
      service: 'pose',
      request: { pose: 'gripper_open' },
    },
    {
      id: 'gripper_close',
      name: 'Close Gripper',
      icon: 'Minimize2',
      service: 'pose',
      request: { pose: 'gripper_close' },
    },
    {
      id: 'autonomous',
      name: 'Autonomous Mode',
      icon: 'Navigation',
      service: 'mode',
      request: { mode: 'autonomous' },
    },
    {
      id: 'manual',
      name: 'Manual Mode',
      icon: 'Gamepad2',
      service: 'mode',
      request: { mode: 'manual' },
    },
  ],

  // Hide actions that are not available for Tita robot
  hiddenActions: ['sit', 'riseSit', 'lightOn', 'lightOff', 'antiCollisionOn', 'antiCollisionOff', 'poseOn', 'poseOff', 'jointLock', 'balanceStand'],
};