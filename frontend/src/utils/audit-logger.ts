import { createSupabaseClient } from './supabase-client';
import { Database } from '@/types/database.types';

export type EventType = 'auth' | 'robot' | 'command' | 'system' | 'data' | 'mission' | 'navigation' | 'audio' | 'camera' | 'safety' | 'export';

export type EventAction =
  // Auth events
  | 'login'
  | 'logout'
  | 'signup'
  | 'session_expired'
  | 'password_reset'
  // Robot events
  | 'robot_connect'
  | 'robot_disconnect'
  | 'robot_connection_failed'
  | 'robot_battery_low'
  | 'robot_battery_critical'
  // Command events
  | 'command_sent'
  | 'button_pressed'
  | 'mode_changed'
  | 'joystick_control'
  | 'keyboard_control'
  | 'gamepad_control'
  // Safety events
  | 'emergency_stop'
  | 'collision_avoided'
  | 'safety_override'
  | 'speed_limit_changed'
  // System events
  | 'settings_updated'
  | 'profile_updated'
  | 'robot_added'
  | 'robot_updated'
  | 'robot_deleted'
  // Data events
  | 'sensor_data_collected'
  | 'diagnostic_run'
  // Mission events
  | 'mission_created'
  | 'mission_updated'
  | 'mission_deleted'
  | 'mission_started'
  | 'mission_stopped'
  | 'mission_completed'
  | 'mission_failed'
  | 'waypoint_added'
  | 'waypoints_updated'
  // Navigation events
  | 'navigation_goal_set'
  | 'navigation_started'
  | 'navigation_cancelled'
  | 'navigation_completed'
  | 'navigation_failed'
  | 'map_loaded'
  | 'map_saved'
  | 'localization_reset'
  // Audio events
  | 'audio_played'
  | 'audio_stopped'
  | 'voice_command_sent'
  | 'tts_generated'
  | 'volume_changed'
  // Camera events
  | 'camera_viewed'
  | 'camera_switched'
  | 'screenshot_taken'
  | 'recording_started'
  | 'recording_stopped'
  | 'stream_quality_changed'
  // Export events
  | 'data_exported'
  | 'logs_downloaded'
  | 'report_generated'
  | 'config_exported'
  | 'map_exported';

export interface AuditLogEvent {
  event_type: EventType;
  event_action: EventAction;
  event_details?: Record<string, any>;
  robot_id?: string;
  robot_name?: string;
}

export class AuditLogger {
  private static instance: AuditLogger;
  private supabase;
  private debugMode = false;
  
  private constructor() {
    this.supabase = createSupabaseClient();
    // Enable debug mode in development
    this.debugMode = process.env.NODE_ENV === 'development';
  }
  
  static getInstance(): AuditLogger {
    if (!AuditLogger.instance) {
      AuditLogger.instance = new AuditLogger();
    }
    return AuditLogger.instance;
  }
  
  async log(event: AuditLogEvent): Promise<void> {
    try {
      // Get current user
      const { data: { session } } = await this.supabase.auth.getSession();

      if (!session?.user) {
        if (this.debugMode) {
          console.warn('[AuditLogger] No user session found, skipping log:', event);
        }
        return;
      }

      // Check if user has audit logging enabled
      const { data: profile } = await this.supabase
        .from('user_profiles')
        .select('audit_logging_enabled')
        .eq('user_id', session.user.id)
        .single();

      // If audit logging is explicitly disabled, skip logging
      if (profile && profile.audit_logging_enabled === false) {
        if (this.debugMode) {
          console.log('[AuditLogger] Audit logging disabled for user, skipping log:', event);
        }
        return;
      }

      if (this.debugMode) {
        console.log('[AuditLogger] Logging event:', event);
      }

      // Insert the log entry
      const { data, error } = await this.supabase
        .from('audit_logs')
        .insert({
          user_id: session.user.id,
          event_type: event.event_type,
          event_action: event.event_action,
          event_details: event.event_details || {},
          robot_id: event.robot_id || null,
          robot_name: event.robot_name || null,
        })
        .select()
        .single();
        
      if (error) {
        console.error('[AuditLogger] Failed to log audit event:', error);
        console.error('[AuditLogger] Event that failed:', event);
        
        // Log specific error details in debug mode
        if (this.debugMode && error.code === '42501') {
          console.error('[AuditLogger] Permission denied. Check RLS policies.');
        }
      } else if (this.debugMode && data) {
        console.log('[AuditLogger] Successfully logged event:', data);
      }
    } catch (error) {
      console.error('[AuditLogger] Unexpected error in audit logging:', error);
    }
  }
  
  // Convenience methods for common events
  async logLogin(details?: Record<string, any>) {
    await this.log({
      event_type: 'auth',
      event_action: 'login',
      event_details: details,
    });
  }
  
  async logLogout() {
    await this.log({
      event_type: 'auth',
      event_action: 'logout',
    });
  }
  
  async logRobotConnect(robotId: string, robotName: string, address: string) {
    await this.log({
      event_type: 'robot',
      event_action: 'robot_connect',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { address },
    });
  }
  
  async logRobotDisconnect(robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'robot',
      event_action: 'robot_disconnect',
      robot_id: robotId,
      robot_name: robotName,
    });
  }
  
  async logRobotConnectionFailed(address: string, error: string) {
    await this.log({
      event_type: 'robot',
      event_action: 'robot_connection_failed',
      event_details: { address, error },
    });
  }
  
  async logCommand(command: string, robotId?: string, robotName?: string, details?: Record<string, any>) {
    await this.log({
      event_type: 'command',
      event_action: 'command_sent',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { command, ...details },
    });
  }
  
  async logButtonPress(button: string, robotId?: string, robotName?: string, details?: Record<string, any>) {
    await this.log({
      event_type: 'command',
      event_action: 'button_pressed',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { button, ...details },
    });
  }
  
  async logModeChange(mode: string, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'command',
      event_action: 'mode_changed',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { mode },
    });
  }
  
  async logEmergencyStop(robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'command',
      event_action: 'emergency_stop',
      robot_id: robotId,
      robot_name: robotName,
    });
  }
  
  async logSettingsUpdate(setting: string, changes: Record<string, any>) {
    await this.log({
      event_type: 'system',
      event_action: 'settings_updated',
      event_details: { setting, changes },
    });
  }
  
  async logProfileUpdate(changes: Record<string, any>) {
    await this.log({
      event_type: 'system',
      event_action: 'profile_updated',
      event_details: { changes },
    });
  }

  // Mission-related events
  async logMissionCreated(missionId: string, missionName: string, robotId?: string) {
    await this.log({
      event_type: 'mission',
      event_action: 'mission_created',
      robot_id: robotId,
      event_details: { mission_id: missionId, mission_name: missionName },
    });
  }
  
  async logMissionUpdated(missionId: string, missionName: string, changes: Record<string, any>) {
    await this.log({
      event_type: 'mission',
      event_action: 'mission_updated',
      event_details: { mission_id: missionId, mission_name: missionName, changes },
    });
  }
  
  async logMissionDeleted(missionId: string, missionName: string) {
    await this.log({
      event_type: 'mission',
      event_action: 'mission_deleted',
      event_details: { mission_id: missionId, mission_name: missionName },
    });
  }
  
  async logMissionStarted(missionId: string, missionName: string, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'mission',
      event_action: 'mission_started',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { mission_id: missionId, mission_name: missionName },
    });
  }
  
  async logMissionStopped(missionId: string, missionName: string, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'mission',
      event_action: 'mission_stopped',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { mission_id: missionId, mission_name: missionName },
    });
  }
  
  async logMissionCompleted(missionId: string, missionName: string, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'mission',
      event_action: 'mission_completed',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { mission_id: missionId, mission_name: missionName },
    });
  }
  
  async logWaypointsUpdated(missionId: string, missionName: string, waypointCount: number) {
    await this.log({
      event_type: 'mission',
      event_action: 'waypoints_updated',
      event_details: { mission_id: missionId, mission_name: missionName, waypoint_count: waypointCount },
    });
  }

  // Navigation events
  async logNavigationGoalSet(goal: { x: number; y: number; theta?: number }, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'navigation',
      event_action: 'navigation_goal_set',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { goal },
    });
  }

  async logNavigationStarted(robotId?: string, robotName?: string, details?: Record<string, any>) {
    await this.log({
      event_type: 'navigation',
      event_action: 'navigation_started',
      robot_id: robotId,
      robot_name: robotName,
      event_details: details,
    });
  }

  async logNavigationCompleted(robotId?: string, robotName?: string, duration?: number) {
    await this.log({
      event_type: 'navigation',
      event_action: 'navigation_completed',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { duration_seconds: duration },
    });
  }

  async logNavigationCancelled(robotId?: string, robotName?: string, reason?: string) {
    await this.log({
      event_type: 'navigation',
      event_action: 'navigation_cancelled',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { reason },
    });
  }

  async logNavigationFailed(robotId?: string, robotName?: string, error?: string) {
    await this.log({
      event_type: 'navigation',
      event_action: 'navigation_failed',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { error },
    });
  }

  // Audio events
  async logAudioPlayed(audioFile: string, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'audio',
      event_action: 'audio_played',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { file: audioFile },
    });
  }

  async logVoiceCommand(command: string, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'audio',
      event_action: 'voice_command_sent',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { command },
    });
  }

  async logTTSGenerated(text: string, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'audio',
      event_action: 'tts_generated',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { text: text.substring(0, 200) }, // Limit text length
    });
  }

  // Camera events
  async logCameraViewed(cameraId: string, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'camera',
      event_action: 'camera_viewed',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { camera_id: cameraId },
    });
  }

  async logScreenshotTaken(cameraId: string, filename: string, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'camera',
      event_action: 'screenshot_taken',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { camera_id: cameraId, filename },
    });
  }

  async logRecordingStarted(cameraId: string, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'camera',
      event_action: 'recording_started',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { camera_id: cameraId },
    });
  }

  async logRecordingStopped(cameraId: string, duration: number, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'camera',
      event_action: 'recording_stopped',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { camera_id: cameraId, duration_seconds: duration },
    });
  }

  // Safety events
  async logCollisionAvoided(details: Record<string, any>, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'safety',
      event_action: 'collision_avoided',
      robot_id: robotId,
      robot_name: robotName,
      event_details: details,
    });
  }

  async logSafetyOverride(reason: string, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'safety',
      event_action: 'safety_override',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { reason },
    });
  }

  async logSpeedLimitChanged(newLimit: number, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'safety',
      event_action: 'speed_limit_changed',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { new_limit: newLimit },
    });
  }

  // Export events
  async logDataExported(dataType: string, format: string, recordCount?: number) {
    await this.log({
      event_type: 'export',
      event_action: 'data_exported',
      event_details: { data_type: dataType, format, record_count: recordCount },
    });
  }

  async logLogsDownloaded(logType: string, dateRange?: { from: string; to: string }) {
    await this.log({
      event_type: 'export',
      event_action: 'logs_downloaded',
      event_details: { log_type: logType, date_range: dateRange },
    });
  }

  async logReportGenerated(reportType: string, parameters?: Record<string, any>) {
    await this.log({
      event_type: 'export',
      event_action: 'report_generated',
      event_details: { report_type: reportType, parameters },
    });
  }

  async logConfigExported(configType: string) {
    await this.log({
      event_type: 'export',
      event_action: 'config_exported',
      event_details: { config_type: configType },
    });
  }

  async logMapExported(mapName: string, format: string) {
    await this.log({
      event_type: 'export',
      event_action: 'map_exported',
      event_details: { map_name: mapName, format },
    });
  }

  // Control input events
  async logJoystickControl(details: { x: number; y: number; angular?: number }, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'command',
      event_action: 'joystick_control',
      robot_id: robotId,
      robot_name: robotName,
      event_details: details,
    });
  }

  async logKeyboardControl(key: string, action: string, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'command',
      event_action: 'keyboard_control',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { key, action },
    });
  }

  async logGamepadControl(button: string, value: number, robotId?: string, robotName?: string) {
    await this.log({
      event_type: 'command',
      event_action: 'gamepad_control',
      robot_id: robotId,
      robot_name: robotName,
      event_details: { button, value },
    });
  }
}

export const auditLogger = AuditLogger.getInstance(); 