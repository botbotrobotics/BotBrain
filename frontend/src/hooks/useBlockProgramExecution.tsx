'use client';

import { useState, useRef, useCallback } from 'react';
import useRobotVelocityPublisher from './ros/useRobotVelocityPublisher';
import useOdometry from './ros/useOdometry';
import { Vector3 } from 'roslib';
import { Quaternion, Euler } from 'three';
import useRobotActionDispatcher from './ros/useRobotActionDispatcher';
import { ServiceOptions } from '@/interfaces/ros/ServiceOptions';

interface BlockCommand {
  id: string;
  instanceId: string;
  type: 'move' | 'turn_right' | 'turn_left' | 'wait' | 'repeat' | 'forever' | 'if' | 'stand_up' | 'stand_down' | 'lock' | 'unlock';
  value?: number;
  unit?: string;
  direction?: 'forward' | 'backward';
  attachedBlocks?: string[];
  nestedBlocks?: string[];
}

interface ExecutionState {
  isRunning: boolean;
  isPaused: boolean;
  currentBlockIndex: number;
  currentBlockId: string | null;
  progress: number;
  error: string | null;
}

// Constants for robot movement
const LINEAR_VELOCITY = 0.5; // Base velocity for movement
const LINEAR_VELOCITY_SLOW = 0.15; // Slow velocity for final approach
const ANGULAR_VELOCITY = 0.8; // Increased for more responsive turning
const POSITION_TOLERANCE = 0.02; // 2cm tolerance (was 5cm)
const ANGLE_TOLERANCE = 0.087; // ~5 degrees in radians
const UPDATE_INTERVAL = 50; // ms - reduced from 100 to 50 for better precision
const STUCK_THRESHOLD = 0.003; // 3mm movement threshold (was 5mm)
const STUCK_COUNT_MAX = 40; // 2 seconds before declaring stuck (adjusted for new interval)
const DECELERATION_DISTANCE = 0.15; // Start slowing down 15cm before target

// Calibration factors - adjust these if robot consistently over/undershoots
const MOVEMENT_CALIBRATION_FACTOR = 0.95; // Reduced to 0.95 to compensate for overshooting
const ROTATION_CALIBRATION_FACTOR = 0.98; // Slightly reduced for rotation as well

export default function useBlockProgramExecution() {
  const publishVelocity = useRobotVelocityPublisher();
  const odometry = useOdometry();
  const robotActionDispatcher = useRobotActionDispatcher();
  
  // Test velocity publisher on first use
  const hasTestedPublisherRef = useRef(false);
  
  const [executionState, setExecutionState] = useState<ExecutionState>({
    isRunning: false,
    isPaused: false,
    currentBlockIndex: 0,
    currentBlockId: null,
    progress: 0,
    error: null
  });

  const executionRef = useRef<{
    abortController: AbortController | null;
    blockSequence: BlockCommand[];
    loopStack: Array<{ blockId: string; count: number; maxCount: number }>;
  }>({
    abortController: null,
    blockSequence: [],
    loopStack: []
  });

  // Calculate distance between two positions (2D)
  const calculateDistance = (pos1: any, pos2: any): number => {
    const dx = pos2.x - pos1.x;
    const dy = pos2.y - pos1.y;
    // Using 2D distance for ground robots
    return Math.sqrt(dx * dx + dy * dy);
  };
  
  // Calculate 3D distance (for debugging)
  const calculate3DDistance = (pos1: any, pos2: any): number => {
    const dx = pos2.x - pos1.x;
    const dy = pos2.y - pos1.y;
    const dz = pos2.z - pos1.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  };
  
  // Check if odometry is valid
  const isValidOdometry = (): boolean => {
    const pos = odometry?.pose?.pose?.position;
    const orient = odometry?.pose?.pose?.orientation;
    return (
      pos && orient &&
      typeof pos.x === 'number' && !isNaN(pos.x) &&
      typeof pos.y === 'number' && !isNaN(pos.y) &&
      typeof orient.x === 'number' && !isNaN(orient.x) &&
      typeof orient.y === 'number' && !isNaN(orient.y) &&
      typeof orient.z === 'number' && !isNaN(orient.z) &&
      typeof orient.w === 'number' && !isNaN(orient.w)
    );
  };

  // Calculate yaw angle from quaternion
  const getYawFromQuaternion = (orientation: any): number => {
    const quaternion = new Quaternion(
      orientation.x,
      orientation.y,
      orientation.z,
      orientation.w
    );
    const euler = new Euler().setFromQuaternion(quaternion, 'XYZ');
    return euler.z; // Yaw is Z rotation in ROS coordinate system
  };

  // Calculate angle difference (handles wrap-around)
  const calculateAngleDifference = (angle1: number, angle2: number): number => {
    let diff = angle2 - angle1;
    while (diff > Math.PI) diff -= 2 * Math.PI;
    while (diff < -Math.PI) diff += 2 * Math.PI;
    return diff;
  };

  // Execute move command with odometry feedback
  const executeMove = async (distance: number, direction: 'forward' | 'backward' = 'forward'): Promise<void> => {
    // Validate odometry before starting
    if (!isValidOdometry()) {
      throw new Error('Invalid odometry data - cannot execute movement');
    }
    
    const startPosition = { ...odometry.pose.pose.position };
    const baseVelocity = direction === 'forward' ? LINEAR_VELOCITY : -LINEAR_VELOCITY;
    const slowVelocity = direction === 'forward' ? LINEAR_VELOCITY_SLOW : -LINEAR_VELOCITY_SLOW;
    
    // Add minimum execution time to prevent immediate stops
    const minExecutionTime = Math.max(500, (distance / Math.abs(LINEAR_VELOCITY)) * 1000 * 0.8); // 80% of theoretical time
    const maxExecutionTime = (distance / Math.abs(LINEAR_VELOCITY)) * 1000 * 3; // 3x theoretical time as absolute max
    const startTime = Date.now();
    
    // Debug logging
    console.log('Starting move:', {
      distance,
      direction,
      startPosition,
      baseVelocity,
      minExecutionTime,
      maxExecutionTime
    });
    
    // Send initial command to wake up the robot
    console.log('Sending initial velocity command to wake robot...');
    publishVelocity({
      linear: new Vector3({ x: slowVelocity * 0.5, y: 0, z: 0 }),
      angular: new Vector3({ x: 0, y: 0, z: 0 })
    });
    
    // Wait a moment for robot to respond
    await new Promise(resolve => setTimeout(resolve, 100));
    
    // Log initial odometry state
    console.log('Initial odometry state:', {
      position: {
        x: odometry.pose.pose.position.x.toFixed(3),
        y: odometry.pose.pose.position.y.toFixed(3),
        z: odometry.pose.pose.position.z.toFixed(3)
      },
      orientation: {
        x: odometry.pose.pose.orientation.x.toFixed(3),
        y: odometry.pose.pose.orientation.y.toFixed(3),
        z: odometry.pose.pose.orientation.z.toFixed(3),
        w: odometry.pose.pose.orientation.w.toFixed(3)
      },
      twist: {
        linear: {
          x: odometry.twist.twist.linear.x.toFixed(3),
          y: odometry.twist.twist.linear.y.toFixed(3),
          z: odometry.twist.twist.linear.z.toFixed(3)
        }
      }
    });
    
    return new Promise((resolve, reject) => {
      let lastDistanceTraveled = 0;
      let stuckCounter = 0;
      
      let lastValidPosition = { ...startPosition };
      let noUpdateCounter = 0;
      const MAX_NO_UPDATE_COUNT = 10; // Allow 500ms of no odometry updates (10 * 50ms)
      let velocityHistory: number[] = []; // Track velocity history for debugging
      
      const intervalId = setInterval(() => {
        if (executionRef.current.abortController?.signal.aborted) {
          clearInterval(intervalId);
          publishVelocity({
            linear: new Vector3({ x: 0, y: 0, z: 0 }),
            angular: new Vector3({ x: 0, y: 0, z: 0 })
          });
          reject(new Error('Execution aborted'));
          return;
        }

        const currentPosition = odometry.pose.pose.position;
        const elapsedTime = Date.now() - startTime;
        
        // Check if position has updated
        const positionChanged = (
          Math.abs(currentPosition.x - lastValidPosition.x) > 0.0001 ||
          Math.abs(currentPosition.y - lastValidPosition.y) > 0.0001
        );
        
        if (positionChanged) {
          lastValidPosition = { ...currentPosition };
          noUpdateCounter = 0;
        } else {
          noUpdateCounter++;
          if (noUpdateCounter > MAX_NO_UPDATE_COUNT) {
            console.warn(`No odometry updates for ${noUpdateCounter * UPDATE_INTERVAL}ms`);
          }
        }
        
        // Calculate distance traveled
        const distanceTraveled = calculateDistance(startPosition, currentPosition);
        const remainingDistance = distance - distanceTraveled;
        
        // Debug current state
        if (Math.random() < 0.05) { // Log 5% of the time to avoid spam
          console.log('Move progress:', {
            distanceTraveled: distanceTraveled.toFixed(3),
            remainingDistance: remainingDistance.toFixed(3),
            targetDistance: distance,
            velocity: remainingDistance < DECELERATION_DISTANCE ? 'slow' : 'normal',
            elapsedTime
          });
        }
        
        // Update progress
        const progress = Math.min((distanceTraveled / distance) * 100, 100);
        setExecutionState(prev => ({ ...prev, progress }));

        // Check if robot is stuck (not moving) - only if we're getting odometry updates
        const movementDelta = Math.abs(distanceTraveled - lastDistanceTraveled);
        
        if (noUpdateCounter <= MAX_NO_UPDATE_COUNT) {
          // Only check for stuck if we're getting odometry updates
          if (movementDelta < STUCK_THRESHOLD) {
            stuckCounter++;
            if (stuckCounter % 20 === 0) { // Log every second
              console.log(`Stuck counter: ${stuckCounter}/${STUCK_COUNT_MAX}, movement: ${movementDelta.toFixed(4)}m`);
            }
            
            if (stuckCounter > STUCK_COUNT_MAX) {
              clearInterval(intervalId);
              publishVelocity({
                linear: new Vector3({ x: 0, y: 0, z: 0 }),
                angular: new Vector3({ x: 0, y: 0, z: 0 })
              });
              console.error('Robot stuck - no movement detected', {
                lastDistance: lastDistanceTraveled,
                currentDistance: distanceTraveled,
                startPos: startPosition,
                currentPos: currentPosition
              });
              reject(new Error('Robot appears to be stuck - check if robot is powered on and responding to commands'));
              return;
            }
          } else {
            if (stuckCounter > 0) {
              console.log('Movement detected, resetting stuck counter');
            }
            stuckCounter = 0;
          }
        } else {
          // If no odometry updates, reset stuck counter to avoid false positives
          stuckCounter = 0;
        }
        lastDistanceTraveled = distanceTraveled;

        // Check if we've reached the target distance
        const reachedDistance = distanceTraveled >= (distance - POSITION_TOLERANCE); // Within 2cm of target
        const maxTimeExceeded = elapsedTime >= maxExecutionTime;
        
        if (reachedDistance) {
          clearInterval(intervalId);
          
          // Stop the robot with multiple commands to ensure full stop
          const sendStopCommands = async () => {
            for (let i = 0; i < 3; i++) {
              publishVelocity({
                linear: new Vector3({ x: 0, y: 0, z: 0 }),
                angular: new Vector3({ x: 0, y: 0, z: 0 })
              });
              await new Promise(resolve => setTimeout(resolve, 50));
            }
            
            console.log('Move completed successfully:', {
              finalDistance: distanceTraveled.toFixed(3),
              targetDistance: distance,
              error: (distance - distanceTraveled).toFixed(3),
              elapsedTime
            });
            
            // Add delay to ensure robot fully stops before resolving
            setTimeout(() => resolve(), 200); // Increased from 100ms
          };
          
          sendStopCommands();
        } else if (maxTimeExceeded) {
          clearInterval(intervalId);
          
          // Stop the robot
          publishVelocity({
            linear: new Vector3({ x: 0, y: 0, z: 0 }),
            angular: new Vector3({ x: 0, y: 0, z: 0 })
          });
          
          console.warn('Move timed out:', {
            finalDistance: distanceTraveled.toFixed(3),
            targetDistance: distance,
            error: (distance - distanceTraveled).toFixed(3),
            elapsedTime
          });
          
          // Consider it complete if we got reasonably close
          if (distanceTraveled >= distance * 0.9) {
            setTimeout(() => resolve(), 100);
          } else {
            reject(new Error(`Movement timed out - only traveled ${distanceTraveled.toFixed(2)}m of ${distance}m`));
          }
        } else {
          // Continue moving with deceleration when close to target
          let velocity = baseVelocity;
          
          // Implement deceleration when approaching target
          if (remainingDistance < DECELERATION_DISTANCE && remainingDistance > POSITION_TOLERANCE) {
            // Linear deceleration: velocity proportional to remaining distance
            const decelerationFactor = remainingDistance / DECELERATION_DISTANCE;
            velocity = slowVelocity + (baseVelocity - slowVelocity) * decelerationFactor;
            
            if (Math.random() < 0.1) { // Log 10% of deceleration commands
              console.log('Decelerating:', {
                remainingDistance: remainingDistance.toFixed(3),
                velocity: velocity.toFixed(3),
                decelerationFactor: decelerationFactor.toFixed(2)
              });
            }
          }
          
          const velocityCommand = {
            linear: new Vector3({ x: velocity, y: 0, z: 0 }),
            angular: new Vector3({ x: 0, y: 0, z: 0 })
          };
          
          // Send velocity command multiple times to ensure robot receives it
          // Important for Unitree robots that may timeout without continuous commands
          publishVelocity(velocityCommand);
          
          // Log occasionally for debugging
          if (Math.random() < 0.02) { // 2% of the time
            console.log('Velocity command:', {
              velocity: velocity.toFixed(3),
              remainingDistance: remainingDistance.toFixed(3)
            });
          }
        }
      }, UPDATE_INTERVAL);
    });
  };

  // Execute turn command with odometry feedback
  const executeTurn = async (degrees: number, direction: 'left' | 'right'): Promise<void> => {
    // Validate odometry before starting
    if (!isValidOdometry()) {
      throw new Error('Invalid odometry data - cannot execute turn');
    }
    
    const radians = (degrees * Math.PI) / 180;
    const startOrientation = { ...odometry.pose.pose.orientation };
    const startYaw = getYawFromQuaternion(startOrientation);
    const angularVel = direction === 'left' ? ANGULAR_VELOCITY : -ANGULAR_VELOCITY;
    const slowAngularVel = angularVel * 0.3; // 30% speed for final approach
    
    // Add minimum execution time based on expected rotation speed
    const minExecutionTime = Math.max(500, (radians / Math.abs(ANGULAR_VELOCITY)) * 1000 * 0.8); // 80% of theoretical time
    const maxExecutionTime = (radians / Math.abs(ANGULAR_VELOCITY)) * 1000 * 3; // 3x theoretical time as absolute max
    const startTime = Date.now();
    
    // Debug logging
    console.log('Starting turn:', {
      degrees,
      radians,
      direction,
      startYaw,
      angularVel,
      minExecutionTime,
      maxExecutionTime
    });
    
    return new Promise((resolve, reject) => {
      let lastYaw = startYaw;
      let totalAngleTurned = 0;
      let stuckCounter = 0;
      
      const intervalId = setInterval(() => {
        if (executionRef.current.abortController?.signal.aborted) {
          clearInterval(intervalId);
          publishVelocity({
            linear: new Vector3({ x: 0, y: 0, z: 0 }),
            angular: new Vector3({ x: 0, y: 0, z: 0 })
          });
          reject(new Error('Execution aborted'));
          return;
        }

        const currentOrientation = odometry.pose.pose.orientation;
        const currentYaw = getYawFromQuaternion(currentOrientation);
        const elapsedTime = Date.now() - startTime;
        
        // Calculate angle turned in this iteration
        const deltaAngle = calculateAngleDifference(lastYaw, currentYaw);
        
        // Accumulate total angle turned (handling wrap-around)
        if (direction === 'left' && deltaAngle > 0) {
          totalAngleTurned += deltaAngle;
        } else if (direction === 'right' && deltaAngle < 0) {
          totalAngleTurned += Math.abs(deltaAngle);
        }
        
        const remainingAngle = radians - totalAngleTurned;
        
        // Debug current state
        if (Math.random() < 0.05) { // Log 5% of the time
          console.log('Turn progress:', {
            currentYaw: currentYaw.toFixed(3),
            totalAngleTurned: totalAngleTurned.toFixed(3),
            remainingAngle: remainingAngle.toFixed(3),
            targetAngle: radians.toFixed(3),
            elapsedTime
          });
        }
        
        // Update progress
        const progress = Math.min((totalAngleTurned / radians) * 100, 100);
        setExecutionState(prev => ({ ...prev, progress }));

        // Check if robot is stuck (not turning)
        const ANGLE_STUCK_THRESHOLD = 0.001; // ~0.057 degrees (reduced from 0.002)
        if (Math.abs(deltaAngle) < ANGLE_STUCK_THRESHOLD) {
          stuckCounter++;
          if (stuckCounter % 20 === 0) { // Log every second
            console.log(`Turn stuck counter: ${stuckCounter}/${STUCK_COUNT_MAX}, delta angle: ${deltaAngle.toFixed(4)} rad`);
          }
          
          if (stuckCounter > STUCK_COUNT_MAX) {
            clearInterval(intervalId);
            publishVelocity({
              linear: new Vector3({ x: 0, y: 0, z: 0 }),
              angular: new Vector3({ x: 0, y: 0, z: 0 })
            });
            console.error('Robot stuck while turning', {
              startYaw,
              currentYaw,
              deltaAngle,
              totalAngleTurned
            });
            reject(new Error('Robot appears to be stuck while turning - check if robot is powered on and responding to commands'));
            return;
          }
        } else {
          if (stuckCounter > 0) {
            console.log('Rotation detected, resetting stuck counter');
          }
          stuckCounter = 0;
        }
        lastYaw = currentYaw;

        // Check if we've reached the target angle
        const reachedAngle = totalAngleTurned >= (radians - ANGLE_TOLERANCE / 2); // Within half the tolerance
        const maxTimeExceeded = elapsedTime >= maxExecutionTime;
        
        if (reachedAngle) {
          clearInterval(intervalId);
          
          // Stop the robot with multiple commands to ensure full stop
          const sendStopCommands = async () => {
            for (let i = 0; i < 3; i++) {
              publishVelocity({
                linear: new Vector3({ x: 0, y: 0, z: 0 }),
                angular: new Vector3({ x: 0, y: 0, z: 0 })
              });
              await new Promise(resolve => setTimeout(resolve, 50));
            }
            
            console.log('Turn completed successfully:', {
              finalAngle: totalAngleTurned.toFixed(3),
              targetAngle: radians.toFixed(3),
              degreesturned: (totalAngleTurned * 180 / Math.PI).toFixed(1),
              targetDegrees: degrees,
              error: ((radians - totalAngleTurned) * 180 / Math.PI).toFixed(1),
              elapsedTime
            });
            
            // Add delay to ensure robot fully stops before resolving
            setTimeout(() => resolve(), 200); // Increased from 100ms
          };
          
          sendStopCommands();
        } else if (maxTimeExceeded) {
          clearInterval(intervalId);
          
          // Stop the robot
          publishVelocity({
            linear: new Vector3({ x: 0, y: 0, z: 0 }),
            angular: new Vector3({ x: 0, y: 0, z: 0 })
          });
          
          console.warn('Turn timed out:', {
            finalAngle: totalAngleTurned.toFixed(3),
            targetAngle: radians.toFixed(3),
            degreesturned: (totalAngleTurned * 180 / Math.PI).toFixed(1),
            elapsedTime
          });
          
          // Consider it complete if we got reasonably close
          if (totalAngleTurned >= radians * 0.9) {
            setTimeout(() => resolve(), 100);
          } else {
            reject(new Error(`Turn timed out - only turned ${(totalAngleTurned * 180 / Math.PI).toFixed(0)}° of ${degrees}°`));
          }
        } else {
          // Continue turning with deceleration when close to target
          let velocity = angularVel;
          
          // Implement deceleration when approaching target (within 15 degrees)
          const DECELERATION_ANGLE = 0.26; // ~15 degrees in radians
          if (remainingAngle < DECELERATION_ANGLE && remainingAngle > ANGLE_TOLERANCE) {
            // Linear deceleration: velocity proportional to remaining angle
            const decelerationFactor = remainingAngle / DECELERATION_ANGLE;
            velocity = slowAngularVel + (angularVel - slowAngularVel) * decelerationFactor;
            
            if (Math.random() < 0.1) { // Log 10% of deceleration commands
              console.log('Turn decelerating:', {
                remainingAngle: remainingAngle.toFixed(3),
                remainingDegrees: (remainingAngle * 180 / Math.PI).toFixed(1),
                velocity: velocity.toFixed(3),
                decelerationFactor: decelerationFactor.toFixed(2)
              });
            }
          }
          
          const turnCommand = {
            linear: new Vector3({ x: 0, y: 0, z: 0 }),
            angular: new Vector3({ x: 0, y: 0, z: velocity })
          };
          
          // Send velocity command multiple times to ensure robot receives it
          // Important for Unitree robots that may timeout without continuous commands
          publishVelocity(turnCommand);
          
          // Log occasionally for debugging
          if (Math.random() < 0.02) { // 2% of the time
            console.log('Velocity command:', {
              velocity: velocity.toFixed(3),
              remainingAngle: remainingAngle.toFixed(3)
            });
          }
        }
      }, UPDATE_INTERVAL);
    });
  };

  // Execute wait command
  const executeWait = async (seconds: number): Promise<void> => {
    const startTime = Date.now();
    
    return new Promise((resolve, reject) => {
      const intervalId = setInterval(() => {
        if (executionRef.current.abortController?.signal.aborted) {
          clearInterval(intervalId);
          reject(new Error('Execution aborted'));
          return;
        }

        const elapsed = (Date.now() - startTime) / 1000;
        const progress = Math.min((elapsed / seconds) * 100, 100);
        setExecutionState(prev => ({ ...prev, progress }));

        if (elapsed >= seconds) {
          clearInterval(intervalId);
          resolve();
        }
      }, UPDATE_INTERVAL);
    });
  };

  // Execute robot action command (for stand up/down, lock/unlock)
  const executeRobotAction = async (
    serviceType: 'mode' | 'pose' | 'light' | 'antiCollision',
    request: any,
    actionName: string
  ): Promise<void> => {
    console.log(`Executing robot action: ${actionName}`, request);
    
    return new Promise((resolve, reject) => {
      const serviceOptions: ServiceOptions = {
        typeKey: serviceType as any,
        request,
        callback: (response: any) => {
          console.log(`${actionName} completed successfully:`, response);
          
          // Set progress to 100% when action completes
          setExecutionState(prev => ({ ...prev, progress: 100 }));
          
          // Add a small delay to ensure robot completes the action
          setTimeout(() => resolve(), 500);
        },
        failedCallback: (error?: string) => {
          console.error(`${actionName} failed:`, error);
          reject(new Error(error || `${actionName} failed`));
        }
      };
      
      // Set initial progress
      setExecutionState(prev => ({ ...prev, progress: 50 }));
      
      // Dispatch the action
      robotActionDispatcher(serviceOptions);
    });
  };

  // Execute a single block
  const executeBlock = async (block: BlockCommand): Promise<void> => {
    setExecutionState(prev => ({
      ...prev,
      currentBlockId: block.instanceId,
      progress: 0
    }));

    switch (block.type) {
      case 'move':
        const distance = block.value || 0;
        const distanceInMeters = block.unit === 'cm' ? distance / 100 : distance;
        // Apply calibration factor to compensate for systematic errors
        const calibratedDistance = distanceInMeters * MOVEMENT_CALIBRATION_FACTOR;
        console.log(`Move block: ${distance}${block.unit || 'm'} -> ${calibratedDistance.toFixed(3)}m (calibration: ${MOVEMENT_CALIBRATION_FACTOR})`);
        await executeMove(calibratedDistance, block.direction || 'forward');
        break;

      case 'turn_left':
      case 'turn_right':
        const degrees = block.value || 0;
        // Apply calibration factor to compensate for systematic errors
        const calibratedDegrees = degrees * ROTATION_CALIBRATION_FACTOR;
        console.log(`Turn block: ${degrees}° -> ${calibratedDegrees.toFixed(1)}° (calibration: ${ROTATION_CALIBRATION_FACTOR})`);
        await executeTurn(calibratedDegrees, block.type === 'turn_left' ? 'left' : 'right');
        break;

      case 'wait':
        await executeWait(block.value || 1);
        break;

      case 'stand_up':
        console.log('Executing stand up command');
        await executeRobotAction('mode', { mode: 'stand_up' }, 'Stand up');
        break;

      case 'stand_down':
        console.log('Executing stand down command');
        await executeRobotAction('mode', { mode: 'stand_down' }, 'Stand down');
        break;

      case 'lock':
        console.log('Executing joint lock command');
        await executeRobotAction('mode', { mode: 'joint_lock' }, 'Joint lock');
        break;

      case 'unlock':
        console.log('Executing balance stand (unlock) command');
        await executeRobotAction('mode', { mode: 'balance_stand' }, 'Balance stand');
        break;

      default:
        console.warn(`Unknown block type: ${block.type}`);
    }
  };

  // Build flat sequence of blocks to execute
  const buildBlockSequence = (blocks: BlockCommand[], startBlockId?: string): BlockCommand[] => {
    const sequence: BlockCommand[] = [];
    const visited = new Set<string>();

    const addBlockAndChildren = (blockId: string) => {
      if (visited.has(blockId)) {
        console.log(`Block ${blockId} already visited, skipping`);
        return;
      }
      visited.add(blockId);

      const block = blocks.find(b => b.instanceId === blockId);
      if (!block) {
        console.warn(`Block ${blockId} not found`);
        return;
      }

      console.log(`Adding block to sequence: ${block.type} (${block.instanceId})`);
      // Add the block itself
      sequence.push(block);

      // Add attached blocks (connected below)
      if (block.attachedBlocks && block.attachedBlocks.length > 0) {
        console.log(`Block has ${block.attachedBlocks.length} attached blocks:`, block.attachedBlocks);
        block.attachedBlocks.forEach(attachedId => addBlockAndChildren(attachedId));
      }
    };

    // Start from specified block or first block without parent
    if (startBlockId) {
      console.log(`Building sequence from specified start block: ${startBlockId}`);
      addBlockAndChildren(startBlockId);
    } else {
      // Find top-level blocks (blocks that are not attached to any other block)
      const topLevelBlocks = blocks.filter(b => {
        // A block is top-level if no other block has it in their attachedBlocks array
        const isAttached = blocks.some(other => 
          other.attachedBlocks?.includes(b.instanceId)
        );
        return !isAttached;
      });
      
      console.log(`Found ${topLevelBlocks.length} top-level blocks:`, topLevelBlocks.map(b => ({ 
        type: b.type, 
        instanceId: b.instanceId,
        attachedBlocks: b.attachedBlocks 
      })));
      
      // Debug: show all blocks structure
      console.log('All blocks structure:', blocks.map(b => ({
        type: b.type,
        instanceId: b.instanceId,
        attachedBlocks: b.attachedBlocks || []
      })));
      
      topLevelBlocks.forEach(block => addBlockAndChildren(block.instanceId));
    }

    console.log(`Final sequence has ${sequence.length} blocks`);
    return sequence;
  };

  // Main execution function
  const executeProgram = useCallback(async (blocks: BlockCommand[]) => {
    console.log('Starting program execution with blocks:', blocks);
    
    // Build execution sequence
    const sequence = buildBlockSequence(blocks);
    console.log('Built execution sequence:', sequence.map(b => ({ id: b.instanceId, type: b.type })));
    
    if (sequence.length === 0) {
      console.warn('No blocks to execute');
      return;
    }

    // Initialize execution
    executionRef.current.abortController = new AbortController();
    executionRef.current.blockSequence = sequence;
    
    setExecutionState({
      isRunning: true,
      isPaused: false,
      currentBlockIndex: 0,
      currentBlockId: null,
      progress: 0,
      error: null
    });
    
    // Test pulse - send a small movement command to verify robot is responding
    if (!hasTestedPublisherRef.current) {
      hasTestedPublisherRef.current = true;
      console.log('Testing velocity publisher with small pulse...');
      publishVelocity({
        linear: new Vector3({ x: 0.1, y: 0, z: 0 }),
        angular: new Vector3({ x: 0, y: 0, z: 0 })
      });
      await new Promise(resolve => setTimeout(resolve, 200));
      publishVelocity({
        linear: new Vector3({ x: 0, y: 0, z: 0 }),
        angular: new Vector3({ x: 0, y: 0, z: 0 })
      });
      await new Promise(resolve => setTimeout(resolve, 300));
    }

    try {
      // Execute blocks in sequence
      for (let i = 0; i < sequence.length; i++) {
        if (executionRef.current.abortController.signal.aborted) {
          console.log('Execution aborted at block', i);
          break;
        }

        // Handle pause - check the ref instead of state
        while (executionRef.current.abortController && !executionRef.current.abortController.signal.aborted) {
          // Check current state by using a callback
          const isPaused = await new Promise<boolean>(resolve => {
            setExecutionState(prev => {
              resolve(prev.isPaused);
              return prev;
            });
          });
          
          if (!isPaused) break;
          await new Promise(resolve => setTimeout(resolve, 100));
        }

        console.log(`Executing block ${i+1}/${sequence.length}:`, sequence[i].type);
        setExecutionState(prev => ({ ...prev, currentBlockIndex: i }));
        
        try {
          await executeBlock(sequence[i]);
          console.log(`Block ${i+1} completed successfully`);
          
          // Add a small delay between blocks to ensure clean transitions
          if (i < sequence.length - 1) {
            await new Promise(resolve => setTimeout(resolve, 200));
          }
        } catch (blockError) {
          console.error(`Error executing block ${i+1}:`, blockError);
          throw blockError; // Re-throw to handle in outer catch
        }
      }
      
      console.log('All blocks executed successfully');
    } catch (error) {
      console.error('Block execution error:', error);
      setExecutionState(prev => ({
        ...prev,
        error: error instanceof Error ? error.message : 'Unknown error'
      }));
    } finally {
      // Clean up
      console.log('Cleaning up execution state');
      setExecutionState(prev => ({
        ...prev,
        isRunning: false,
        currentBlockId: null,
        progress: 0
      }));
    }
  }, [publishVelocity, robotActionDispatcher]);

  // Control functions
  const play = useCallback((blocks: BlockCommand[]) => {
    executeProgram(blocks);
  }, [executeProgram]);

  const pause = useCallback(() => {
    setExecutionState(prev => ({ ...prev, isPaused: true }));
  }, []);

  const resume = useCallback(() => {
    setExecutionState(prev => ({ ...prev, isPaused: false }));
  }, []);

  const stop = useCallback(() => {
    executionRef.current.abortController?.abort();
    
    // Stop robot immediately
    publishVelocity({
      linear: new Vector3({ x: 0, y: 0, z: 0 }),
      angular: new Vector3({ x: 0, y: 0, z: 0 })
    });
    
    setExecutionState({
      isRunning: false,
      isPaused: false,
      currentBlockIndex: 0,
      currentBlockId: null,
      progress: 0,
      error: null
    });
  }, [publishVelocity]);

  return {
    executionState,
    play,
    pause,
    resume,
    stop
  };
} 