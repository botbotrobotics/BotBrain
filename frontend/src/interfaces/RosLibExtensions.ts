// Extend the ROSLIB type definitions for roslib 2.x compatibility
declare module 'roslib' {
  interface RosOptions {
    /**
     * The encoding to use for communication with the ROS bridge
     * 'cbor' - Use CBOR encoding for better performance
     */
    encoding?: 'cbor' | 'json';
  }

  interface TopicOptions {
    /**
     * The compression type to use for the topic
     * 'cbor' - Use CBOR compression for better performance
     */
    compression?: 'none' | 'png' | 'cbor';
  }

  interface ServiceOptions {
    /**
     * The compression type to use for the service
     * 'cbor' - Use CBOR compression for better performance
     */
    compression?: 'none' | 'png' | 'cbor';
  }
}

// ============ roslib 2.x Compatibility Types ============
// These were removed in roslib 2.x but are used in many places in this codebase.
// In roslib 2.x, messages are plain JavaScript objects.

/**
 * ROS Message type - represents any ROS message.
 * In roslib 2.x, messages are plain objects, not instances of a Message class.
 */
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type RosMessage = Record<string, any>;

/**
 * Helper function to create a ROS message object (for compatibility with old `new ROSLIB.Message()` calls).
 * Simply returns the input object - roslib 2.x uses plain objects for messages.
 */
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export function createRosMessage<T extends Record<string, any>>(data: T): T {
  return data;
} 