export type SpeedMode = 'beginner' | 'normal' | 'insane';

export const SPEED_MODE_MULTIPLIERS: Record<SpeedMode, number> = {
  beginner: 0.2,
  normal: 0.8,
  insane: 1.0,
};

export const SPEED_MODE_LABELS: Record<SpeedMode, string> = {
  beginner: 'Beginner',
  normal: 'Normal',
  insane: 'Insane',
};

export const SPEED_MODE_DESCRIPTIONS: Record<SpeedMode, string> = {
  beginner: '20% speed - Safe for learning',
  normal: '80% speed - Balanced performance',
  insane: '100% speed - Maximum velocity',
};

export const DEFAULT_SPEED_MODE: SpeedMode = 'normal';
