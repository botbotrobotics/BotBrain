import { getRosMsgType, getRosServiceType, rosTypes } from './messages';
import * as ROSLIB from 'roslib';
import { ServiceOptions } from '@/interfaces/ros/ServiceOptions';
import { ServiceType } from '@/types/RobotActionTypes';
import { RobotProfile } from '@/config/robot-profiles';

// Legacy mappings for backward compatibility
export const legacyTopicsMessages = {
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
};

export const legacyServicesMessages: Record<ServiceType, string> = {
  prompt: 'rosa_prompt',
  antiCollision: 'obstacle_avoidance',
  light: 'light_control',
  stop: 'emergency_stop',
  pose: 'pose',
  mode: 'mode',
  talk: 'talk',
  delivery: 'delivery_control',
};

export const ServiceMsgType: Record<ServiceType, keyof typeof rosTypes> = {
  prompt: 'llmPrompt',
  light: 'bool',
  stop: 'bool',
  antiCollision: 'antiCollision',
  mode: 'mode',
  pose: 'pose',
  talk: 'talk',
  delivery: 'bool',
};

export const TopicMsgType: Record<
  keyof typeof legacyTopicsMessages,
  keyof typeof rosTypes
> = {
  velocity: 'twist',
  laserScan: 'laserScan',
  temperature: 'temperature',
  listener: 'string',
  odometry: 'odometry',
  map: 'occupancyGrid',
  jointStates: 'jointState',
  camera: 'image',
  battery: 'battery',
  velocityNipple: 'twist',
  sportModeState: 'sportModeState',
  robotStatus: 'robotStatus',
  thermal: 'image',
  rgb: 'image',
  goalPose: 'poseStamped',
  cancelGoal: 'bool',
  diagnostics: 'diagnosticStats',
};

/**
 * Gets the ROS topic path based on robot profile or falls back to legacy
 * @param typeKey Topic key
 * @param robotProfile Optional robot profile for custom topic mapping
 * @param isDummy Flag for dummy/simulation mode
 * @returns Full topic path
 */
export function getRosTopic(
  typeKey: keyof typeof legacyTopicsMessages,
  robotProfile?: RobotProfile | null,
  isDummy = false
): string {
  let topic: string;
  
  // Use robot profile if available, otherwise fall back to legacy
  if (robotProfile && robotProfile.topics) {
    topic = robotProfile.topics[typeKey] || legacyTopicsMessages[typeKey];
  } else {
    topic = legacyTopicsMessages[typeKey];
  }
  
  return `${isDummy ? '/dummy' : ''}/${topic}`;
}

/**
 * Gets the ROS service path based on robot profile or falls back to legacy
 * @param typeKey Service key
 * @param robotProfile Optional robot profile for custom service mapping
 * @param isDummy Flag for dummy/simulation mode
 * @returns Full service path
 */
export function getRosService(
  typeKey: ServiceType,
  robotProfile?: RobotProfile | null,
  isDummy = false
): string {
  let service: string;
  
  // Use robot profile if available, otherwise fall back to legacy
  if (robotProfile && robotProfile.services) {
    service = robotProfile.services[typeKey] || legacyServicesMessages[typeKey];
  } else {
    service = legacyServicesMessages[typeKey];
  }
  
  return `${isDummy ? '/dummy' : ''}/${service}`;
}

export class ROSBase {
  protected ros: ROSLIB.Ros;
  protected isDummy: boolean = false;
  protected robotProfile?: RobotProfile | null;

  constructor(
    rosCon: ROSLIB.Ros, 
    useDummyData: boolean = false,
    robotProfile?: RobotProfile | null
  ) {
    this.ros = rosCon;
    this.isDummy = useDummyData;
    this.robotProfile = robotProfile;
  }
}

export class ROSTopicFactory extends ROSBase {
  constructor(
    rosCon: ROSLIB.Ros, 
    useDummyData: boolean = false,
    robotProfile?: RobotProfile | null
  ) {
    super(rosCon, useDummyData, robotProfile);
  }

  private createTopic = (
    topicName: keyof typeof legacyTopicsMessages
  ): ROSLIB.Topic<unknown> => {
    return new ROSLIB.Topic({
      ros: this.ros,
      name: getRosTopic(topicName, this.robotProfile, this.isDummy),
      messageType: getRosMsgType(TopicMsgType[topicName]),
      compression: 'cbor',
      throttle_rate: 0,
      queue_size: 1,
    });
  };

  /**
   * Creates and subscribes to a topic with profile-aware topic mapping
   * @param topicKey Topic key
   * @param callback Callback function for messages
   * @returns Created topic
   */
  public createAndSubscribeTopic = <T,>(
    topicKey: keyof typeof legacyTopicsMessages,
    callback: (msg: T) => void
  ): ROSLIB.Topic<unknown> => {
    const topic = this.createTopic(topicKey);

    topic.subscribe((msg: unknown) => {
      callback(msg as T);
    });

    return topic;
  };

  /**
   * Creates and subscribes to a high-frequency topic with profile-aware mapping
   * @param topicKey Topic key
   * @param callback Callback function for messages
   * @returns Created topic
   */
  public createAndSubscribeHighFrequencyTopic = <T,>(
    topicKey: keyof typeof legacyTopicsMessages,
    callback: (msg: T) => void
  ): ROSLIB.Topic<unknown> => {
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: getRosTopic(topicKey, this.robotProfile, this.isDummy),
      messageType: getRosMsgType(TopicMsgType[topicKey]),
      // Settings optimized for real-time streaming with CBOR compression
      compression: 'cbor',
      throttle_rate: 0,
      queue_size: 1,
    });

    // Create a local timestamp to track message processing
    let lastProcessTimestamp = 0;

    topic.subscribe((msg: unknown) => {
      const now = Date.now();
      // Skip processing if we're already processing messages too frequently
      if (now - lastProcessTimestamp < 33) {
        // ~30fps max = 33ms
        return;
      }

      lastProcessTimestamp = now;
      callback(msg as T);
    });

    return topic;
  };
}

export class ROSServiceFactory extends ROSBase {
  constructor(
    rosCon: ROSLIB.Ros, 
    useDummyData: boolean = false,
    robotProfile?: RobotProfile | null
  ) {
    super(rosCon, useDummyData, robotProfile);
  }

  private createService = (serviceName: ServiceType): ROSLIB.Service<unknown, unknown> => {
    const serviceType = getRosServiceType(ServiceMsgType[serviceName]);
    const servicePath = getRosService(serviceName, this.robotProfile, this.isDummy);
    
    console.log(`Creating service: ${serviceName}, type: ${serviceType}, path: ${servicePath}`);
    
    return new ROSLIB.Service({
      ros: this.ros,
      name: servicePath,
      serviceType: serviceType,
    });
  };

  public callServiceWithCallback(options: ServiceOptions) {
    const service = this.createService(options.typeKey);
    const request = {
      ...options.request,
    };

    service.callService(
      request,
      options.callback ?? (() => {}),
      options.failedCallback
    );
  }
}