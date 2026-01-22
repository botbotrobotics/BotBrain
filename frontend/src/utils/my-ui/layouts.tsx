import { generateId, Widget } from '@/contexts/DashboardContext';
import { CameraType, RobotCameraType } from '@/types/CameraType';
import { RobotActionTypeName } from '@/types/RobotActionTypes';
import { StickType } from '@/types/StickType';

export type LayoutType =
  | 'Default'
  | 'Command Input'
  | 'Information'
  | 'Image Visualization';

const layouts: Record<LayoutType, Widget[]> = {
  Default: [
    {
      type: 'info',
      id: generateId(),
      size: { width: 400, height: 250 },
      position: { x: 0, y: 0 },
    },
    {
      type: 'chat',
      id: generateId(),
      size: { width: 400, height: 250 },
      position: { x: 0, y: 260 },
    },
    {
      type: 'camera',
      id: generateId(),
      size: { width: 425, height: 250 },
      position: { x: 400, y: 0 },
    },
    {
      type: 'camera',
      id: generateId(),
      size: { width: 425, height: 250 },
      position: { x: 825, y: 0 },
    },
    {
      type: 'visualization3d',
      id: generateId(),
      size: { width: 850, height: 250 },
      position: { x: 400, y: 260 },
    },
    {
      type: 'buttonGroup',
      id: generateId(),
      size: { width: 200, height: 510 },
      position: { x: 1250, y: 0 },
    },
  ],
  Information: [
    {
      type: 'info',
      id: generateId(),
      size: { width: 400, height: 250 },
      position: { x: 0, y: 0 },
    },
    {
      type: 'info',
      id: generateId(),
      size: { width: 400, height: 250 },
      position: { x: 0, y: 260 },
    },
    {
      type: 'sidewaysgauge',
      id: generateId(),
      size: { width: 425, height: 250 },
      position: { x: 400, y: 0 },
    },
    {
      type: 'sidewaysgauge',
      id: generateId(),
      size: { width: 425, height: 250 },
      position: { x: 825, y: 0 },
      props: { topic: '/temperature' },
    },
    {
      type: 'visualization3d',
      id: generateId(),
      size: { width: 850, height: 250 },
      position: { x: 400, y: 260 },
    },
    {
      type: 'gauge',
      id: generateId(),
      size: { width: 400, height: 510 },
      position: { x: 1250, y: 0 },
    },
  ],
  'Command Input': [
    {
      type: 'chat',
      id: generateId(),
      size: { width: 400, height: 500 },
      position: { x: 0, y: 0 },
    },
    {
      type: 'camera',
      id: generateId(),
      size: { width: 425, height: 250 },
      position: { x: 400, y: 0 },
    },
    {
      type: 'camera',
      id: generateId(),
      size: { width: 425, height: 250 },
      position: { x: 825, y: 0 },
    },
    {
      type: 'joystick',
      id: generateId(),
      size: { width: 425, height: 250 },
      position: { x: 400, y: 260 },
      props: {
        joystickSide: 'left' as StickType,
      },
    },
    {
      type: 'joystick',
      id: generateId(),
      size: { width: 425, height: 250 },
      position: { x: 825, y: 260 },
      props: {
        joystickSide: 'right' as StickType,
      },
    },
    {
      type: 'button',
      id: generateId(),
      size: { width: 200, height: 150 },
      position: { x: 1290, y: 0 },
      props: {
        buttonAction: 'getDown' as RobotActionTypeName,
      },
    },
    {
      type: 'button',
      id: generateId(),
      size: { width: 200, height: 150 },
      position: { x: 1290, y: 150 },
      props: {
        buttonAction: 'antiCollisionOn' as RobotActionTypeName,
      },
    },
    {
      type: 'button',
      id: generateId(),
      size: { width: 200, height: 150 },
      position: { x: 1290, y: 300 },
      props: {
        buttonAction: 'sit' as RobotActionTypeName,
      },
    },
    {
      type: 'button',
      id: generateId(),
      size: { width: 200, height: 150 },
      position: { x: 1290, y: 450 },
      props: {
        buttonAction: 'lightOn' as RobotActionTypeName,
      },
    },
    {
      type: 'button',
      id: generateId(),
      size: { width: 200, height: 150 },
      position: { x: 1490, y: 0 },
      props: {
        buttonAction: 'jointLock' as RobotActionTypeName,
      },
    },
    {
      type: 'button',
      id: generateId(),
      size: { width: 200, height: 150 },
      position: { x: 1490, y: 150 },
      props: {
        buttonAction: 'balanceStand' as RobotActionTypeName,
      },
    },
    {
      type: 'button',
      id: generateId(),
      size: { width: 200, height: 150 },
      position: { x: 1490, y: 300 },
      props: {
        buttonAction: 'hello' as RobotActionTypeName,
      },
    },
    {
      type: 'button',
      id: generateId(),
      size: { width: 200, height: 150 },
      position: { x: 1490, y: 450 },
      props: {
        buttonAction: 'poseOn' as RobotActionTypeName,
      },
    },
  ],
  'Image Visualization': [
    {
      type: 'visualization3d',
      id: generateId(),
      size: { width: 650, height: 700 },
      position: { x: 0, y: 0 },
    },
    {
      type: 'camera',
      id: generateId(),
      size: { width: 400, height: 700 },
      position: { x: 650, y: 0 },
    },
    {
      type: 'camera',
      id: generateId(),
      size: { width: 400, height: 700 },
      position: { x: 1050, y: 0 },
      props: {
        robotCamera: 'sideCam' as RobotCameraType,
        cameraType: 'rgb' as CameraType,
      },
    },
    {
      type: 'camera',
      id: generateId(),
      size: { width: 400, height: 700 },
      position: { x: 1450, y: 0 },
      props: {
        robotCamera: 'sideCam' as RobotCameraType,
        cameraType: 'thermal' as CameraType,
      },
    },
  ],
};

export default layouts;
