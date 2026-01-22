// Simple configuration storage utility using localStorage

interface ConfigValues {
  robotAddress: string;
  videoFormat: 'mp4' | 'webm';
  overlayColor: 'white' | 'black' | 'red' | 'purple' | 'blue' | 'green';
  gamepadMappings: Record<number, string>;
  gamepadAxisMappings: Record<string, string>;
  joystickVisualizationOnly: boolean;
  // Additional config values can be added here in the future
}

// Default gamepad button mappings
const DEFAULT_GAMEPAD_MAPPINGS: Record<number, string> = {
  0: 'getUp',       // A - Stand up
  1: 'getDown',     // B - Stand down
  2: 'stopMove',    // X - Stop
  3: 'hello',       // Y - Hello gesture
  4: 'lightOff',    // LB - Light off
  5: 'lightOn',     // RB - Light on
  6: 'antiCollisionOff', // LT - Anti-collision off
  7: 'antiCollisionOn',  // RT - Anti-collision on
  8: 'balanceStand',     // Back - Balance stand
  9: 'emergencyOn',      // Start - Emergency stop
};

// Default gamepad axis mappings
const DEFAULT_GAMEPAD_AXIS_MAPPINGS: Record<string, string> = {
  'left': 'movement',   // Left stick controls movement
  'right': 'rotation',  // Right stick controls rotation
};

// Default config values
const DEFAULT_CONFIG: ConfigValues = {
  robotAddress: 'ws://192.168.1.95:9090',
  videoFormat: 'mp4',
  overlayColor: 'white',
  gamepadMappings: DEFAULT_GAMEPAD_MAPPINGS,
  gamepadAxisMappings: DEFAULT_GAMEPAD_AXIS_MAPPINGS,
  joystickVisualizationOnly: false,
};

/**
 * Gets a configuration value from localStorage
 * @param key The key of the config value to get
 * @returns The stored value, or the default if not found
 */
export function getConfig<K extends keyof ConfigValues>(key: K): ConfigValues[K] {
  if (typeof window === 'undefined') return DEFAULT_CONFIG[key];
  
  try {
    const storedValue = localStorage.getItem(`robotics_config_${key}`);
    if (storedValue === null) return DEFAULT_CONFIG[key];
    
    // Parse the stored value
    const parsed = JSON.parse(storedValue);
    
    // Handle each config key separately with proper type checking
    switch (key) {
      case 'videoFormat':
        // Check if the value is a valid video format
        if (parsed === 'mp4' || parsed === 'webm') {
          return parsed as ConfigValues[K];
        }
        return DEFAULT_CONFIG.videoFormat as ConfigValues[K];
        
      case 'robotAddress':
        // Basic validation for robot address
        if (typeof parsed === 'string' && (
          parsed.startsWith('ws://') || 
          parsed.startsWith('wss://') || 
          parsed.startsWith('http://') || 
          parsed.startsWith('https://') ||
          /^[0-9\.]+$/.test(parsed) || // IP without protocol
          parsed.includes('.') // domain without protocol
        )) {
          return parsed as ConfigValues[K];
        }
        return DEFAULT_CONFIG.robotAddress as ConfigValues[K];
        
      case 'overlayColor':
        // Check if the value is a valid overlay color
        if (
          parsed === 'white' ||
          parsed === 'black' ||
          parsed === 'red' ||
          parsed === 'purple' ||
          parsed === 'blue' ||
          parsed === 'green'
        ) {
          return parsed as ConfigValues[K];
        }
        return DEFAULT_CONFIG.overlayColor as ConfigValues[K];

      case 'gamepadMappings':
        // Validate gamepad mappings object
        if (typeof parsed === 'object' && parsed !== null) {
          return parsed as ConfigValues[K];
        }
        return DEFAULT_CONFIG.gamepadMappings as ConfigValues[K];

      case 'gamepadAxisMappings':
        // Validate gamepad axis mappings object
        if (typeof parsed === 'object' && parsed !== null) {
          return parsed as ConfigValues[K];
        }
        return DEFAULT_CONFIG.gamepadAxisMappings as ConfigValues[K];

      case 'joystickVisualizationOnly':
        // Validate boolean value
        if (typeof parsed === 'boolean') {
          return parsed as ConfigValues[K];
        }
        return DEFAULT_CONFIG.joystickVisualizationOnly as ConfigValues[K];

      default:
        return parsed as ConfigValues[K];
    }
  } catch (error) {
    console.error(`Error retrieving config ${key}:`, error);
    return DEFAULT_CONFIG[key];
  }
}

/**
 * Sets a configuration value in localStorage
 * @param key The key of the config value to set
 * @param value The value to store
 */
export function setConfig<K extends keyof ConfigValues>(key: K, value: ConfigValues[K]): void {
  if (typeof window === 'undefined') return;
  
  try {
    localStorage.setItem(`robotics_config_${key}`, JSON.stringify(value));
  } catch (error) {
    console.error(`Error saving config ${key}:`, error);
  }
}

/**
 * Gets all configuration values
 * @returns All config values, with defaults applied for missing values
 */
export function getAllConfig(): ConfigValues {
  if (typeof window === 'undefined') return DEFAULT_CONFIG;

  const config: ConfigValues = {
    robotAddress: getConfig('robotAddress'),
    videoFormat: getConfig('videoFormat'),
    overlayColor: getConfig('overlayColor'),
    gamepadMappings: getConfig('gamepadMappings'),
    gamepadAxisMappings: getConfig('gamepadAxisMappings'),
    joystickVisualizationOnly: getConfig('joystickVisualizationOnly'),
  };

  return config;
} 