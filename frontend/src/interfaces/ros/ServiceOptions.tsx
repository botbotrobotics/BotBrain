import { ServiceType } from '@/types/RobotActionTypes';

export interface ServiceOptions {
  typeKey: ServiceType;
  request?: any;
  callback?: (result: any) => void;
  failedCallback?: (error?: string) => void;
}
