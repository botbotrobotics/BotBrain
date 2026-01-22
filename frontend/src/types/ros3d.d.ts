declare module 'ros3d/build/ros3d.js' {
  export class Viewer {
    constructor(options: any);
    setSize(width: number, height: number): void;
    dispose(): void;
    updateLidarPoints(scan: any): void;
  }

  export class UrdfClient {
    constructor(options: any);
  }

  // Se houver outras classes/métodos que você usar, adicione-os aqui.
}
