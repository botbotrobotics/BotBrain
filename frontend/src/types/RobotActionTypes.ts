export interface RobotActionCallbackOptions {
  input?: any;
  request?: any;
  callback: (result: any) => void;
  failedCallback?: (error?: string) => void;
}

export interface ActionType {
  label: string;
  icon: string | React.ComponentType<{ className?: string; size?: number }>;
  action: (options?: RobotActionCallbackOptions) => void;
}

export type MenuActionTypeName =
  | 'fleet'
  | 'dashboard'
  | 'home'
  | 'chart'
  | 'chatMenu'
  | 'chatNew'
  | 'user'
  | 'settings'
  | 'help'
  | 'mapEdit'
  | 'labs'
  | 'ai'
  | 'maps'
  | 'soundboard'
  | 'dPad'
  | 'fullScreen'
  | 'expandContainer'
  | 'menu'
  | 'extras'
  | 'myUi'
  | 'health';

export interface MenuActionType extends ActionType {
  iconDefaultSize?: number;
}

export type RobotActionTypeName =
  | 'getUp'
  | 'getDown'
  | 'balanceStand'
  | 'jointLock'
  | 'lightOn'
  | 'lightOff'
  | 'antiCollisionOn'
  | 'antiCollisionOff'
  | 'emergencyOn'
  | 'emergencyOff'
  | 'poseOn'
  | 'poseOff'
  | 'hello'
  | 'stopMove'
  | 'sit'
  | 'riseSit'
  | 'stretch'
  | 'dance'
  | 'prompt'
  | 'damping'
  | 'zeroTorque';

export type ServiceType =
  | 'prompt'
  | 'antiCollision'
  | 'mode'
  | 'light'
  | 'stop'
  | 'light'
  | 'pose'
  | 'talk'
  | 'delivery';

export interface RobotActionType extends ActionType {
  name: RobotActionTypeName;
}
