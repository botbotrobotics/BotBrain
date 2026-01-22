// State Machine Command enum
export enum StateMachineCommand {
  ACTIVATE_NODE = 1,
  DEACTIVATE_NODE = 2,
  RESTART_NODE = 3,
}

// State Machine Command Response enum
export enum StateMachineCommandResponse {
  SUCCESS = 1,
  FAILURE = 2,
  INVALID_NODE = 3,
  INVALID_COMMAND = 4,
  INVALID_STATE = 5,
}

// Service Request Type
export interface StateMachineServiceRequest {
  node: string;
  command: number;
}

// Service Response Type
export interface StateMachineServiceResponse {
  result: number;
  success: boolean;
}

// Individual Node Status
export interface NodeStatus {
  name: string;        // Container/node ID (e.g., "robot_read_node")
  displayName: string; // Human-readable module name (e.g., "Robot Reader")
  state: string;
  active: boolean;
}

// Status Array Message Type (from topic)
export interface StatusArrayMessage {
  nodes: NodeStatus[];
}

// Display state for UI
export type NodeStatusSignal = 'green' | 'yellow' | 'red';

export interface NodeStatusDisplay extends NodeStatus {
  signal: NodeStatusSignal;
}
