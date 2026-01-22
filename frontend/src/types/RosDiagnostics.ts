// ROS2 Diagnostics message types
// Based on diagnostic_msgs/DiagnosticArray and diagnostic_msgs/DiagnosticStatus

export interface KeyValue {
  key: string;
  value: string;
}

export interface DiagnosticStatus {
  // Level of operation
  level: number; // 0 = OK, 1 = WARN, 2 = ERROR, 3 = STALE
  name: string; // Description of the component
  message: string; // Description of the status
  hardware_id: string; // Hardware ID
  values: KeyValue[]; // Additional diagnostic values
}

export interface DiagnosticArray {
  header: {
    stamp: {
      sec: number;
      nanosec: number;
    };
    frame_id: string;
  };
  status: DiagnosticStatus[];
}

// Level constants
export const DiagnosticLevel = {
  OK: 0,
  WARN: 1,
  ERROR: 2,
  STALE: 3,
} as const;

export type DiagnosticLevelType = typeof DiagnosticLevel[keyof typeof DiagnosticLevel];

// Helper function to get level name
export function getDiagnosticLevelName(level: number): string {
  switch (level) {
    case DiagnosticLevel.OK:
      return 'OK';
    case DiagnosticLevel.WARN:
      return 'Warning';
    case DiagnosticLevel.ERROR:
      return 'Error';
    case DiagnosticLevel.STALE:
      return 'Stale';
    default:
      return 'Unknown';
  }
}

// Helper function to get level color
export function getDiagnosticLevelColor(level: number): string {
  switch (level) {
    case DiagnosticLevel.OK:
      return 'green';
    case DiagnosticLevel.WARN:
      return 'yellow';
    case DiagnosticLevel.ERROR:
      return 'red';
    case DiagnosticLevel.STALE:
      return 'gray';
    default:
      return 'gray';
  }
}