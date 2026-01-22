// Type definitions for Jetson diagnostic data
export interface JetsonBoardStatus {
  powerMode: string;
  powerModeId: number;
  jetsonClocks: boolean;
  jetsonClocksOnBoot: boolean;
  uptime: string;
  interval: number;
  jtopVersion: string;
  model: string;
  partNumber: string;
  pNumber: string;
  module: string;
  soc: string;
  cudaArchBin: string;
  serialNumber: string;
  l4t: string;
  jetpack: string;
  libCuda: string;
  libOpenCV: string;
  libOpenCVCuda: boolean;
  libCuDNN: string;
  libTensorRT: string;
  libVPI: string;
}

export interface JetsonCpu {
  id: number;
  usage: number;
  freq: number;
  unit: string;
  governor: string;
  model: string;
}

export interface JetsonGpu {
  usage: number;
  freq: {
    cur: number;
    max: number;
    min: number;
    governor: string;
    gpc: number[];
  };
  unit: string;
}

export interface JetsonMemory {
  ram: {
    used: number;
    shared: number;
    total: number;
    unit: string;
    lfbNblock: number;
    lfbSize: number;
    lfbUnit: string;
  };
  swap: {
    used: number;
    total: number;
    unit: string;
    cachedSize: number;
    cachedUnit: string;
  };
  emc: {
    usage: number;
    freq: number;
    unit: string;
  };
}

export interface JetsonTemperatures {
  cpu: number;
  cv0: number;
  cv1: number;
  cv2: number;
  gpu: number;
  soc0: number;
  soc1: number;
  soc2: number;
  tj: number;
}

export interface JetsonPowerRail {
  name: string;
  currentPower: number;
  averagePower: number;
}

export interface JetsonPower {
  currentTotal: number;
  averageTotal: number;
  rails: JetsonPowerRail[];
}

export interface JetsonFan {
  mode: string;
  speed: number[];
  control: string;
}

export interface JetsonDisk {
  used: number;
  total: number;
  unit: string;
}

export interface JetsonDiagnosticsData {
  board: JetsonBoardStatus;
  cpus: JetsonCpu[];
  gpu: JetsonGpu;
  memory: JetsonMemory;
  temperatures: JetsonTemperatures;
  power: JetsonPower;
  fan: JetsonFan;
  disk: JetsonDisk;
}

// ROS2 diagnostic message interface
export interface DiagnosticKeyValue {
  key: string;
  value: string;
}

export interface DiagnosticStatus {
  level: number;
  name: string;
  message: string;
  hardware_id: string;
  values: DiagnosticKeyValue[];
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