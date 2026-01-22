declare module 'ros2d' {
  export interface ViewerOptions {
    divID: string;
    width: number;
    height: number;
    background?: string;
  }

  export class Viewer {
    constructor(options: ViewerOptions);
    scene: any;
    width: number;
    height: number;
    scaleToDimensions(width: number, height: number): void;
    shift(x: number, y: number): void;
    resize(width: number, height: number): void;
  }

  export interface OccupancyGridClientOptions {
    ros: any;
    topic?: string;
    rootObject: any;
    continuous?: boolean;
  }

  export class OccupancyGridClient {
    constructor(options: OccupancyGridClientOptions);
    currentGrid: any;
    on(event: string, callback: () => void): void;
    rootObject: any;
  }

  export interface NavigationArrowOptions {
    size?: number;
    strokeSize?: number;
    fillColor?: any;
    strokeColor?: any;
    pulse?: boolean;
  }

  export class NavigationArrow {
    constructor(options: NavigationArrowOptions);
    x: number;
    y: number;
    rotation: number;
  }
} 