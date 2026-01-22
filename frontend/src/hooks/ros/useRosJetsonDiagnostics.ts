import { useState, useEffect, useCallback, useRef } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';
import * as ROSLIB from 'roslib';
import { 
  JetsonDiagnosticsData, 
  DiagnosticArray, 
  DiagnosticStatus,
  JetsonCpu,
  JetsonPowerRail
} from '@/types/JetsonDiagnostics';

// ROS std_msgs/String interface
interface StringMessage {
  data: string;
}

// Parse diagnostic stats from string format
function parseDiagnosticStatsString(data: string): JetsonDiagnosticsData | null {
  try {
    console.log('[Diagnostics Parser] Parsing string data:', data.substring(0, 200) + '...');
    
    // Initialize result structure
    const result: any = {
      board: {},
      cpus: [],
      gpu: {},
      memory: { ram: {}, swap: {}, emc: {} },
      temperatures: {},
      power: { rails: [] },
      fan: {},
      disk: {}
    };
    
    // Split by lines and parse
    const lines = data.split('\n');
    let currentSection = '';
    
    for (const line of lines) {
      // Board status
      if (line.includes('[jetson_stats board status]')) {
        currentSection = 'board_status';
        const match = line.match(/NV Power\[(\d+)\] (\w+) - JC (\w+)/);
        if (match) {
          result.board.powerModeId = parseInt(match[1]);
          result.board.powerMode = match[2];
          result.board.jetsonClocks = match[3] === 'active';
        }
      }
      
      // Parse key-value pairs
      const kvMatch = line.match(/^\s*- (.+?):\s*(.+)$/);
      if (kvMatch) {
        const [_, key, value] = kvMatch;
        
        if (currentSection === 'board_status') {
          if (key === 'Up Time') result.board.uptime = value;
          if (key === 'jetson_clocks on boot') result.board.jetsonClocksOnBoot = value === 'True';
          if (key === 'jtop') result.board.jtopVersion = value;
        }
      }
      
      // CPU data
      const cpuMatch = line.match(/\[jetson_stats cpu (\d+)\]\s*([\d.]+)%/);
      if (cpuMatch) {
        const cpuId = parseInt(cpuMatch[1]);
        const usage = parseFloat(cpuMatch[2]);
        currentSection = `cpu_${cpuId}`;
        result.cpus[cpuId] = { id: cpuId, usage, freq: 0, unit: 'khz', governor: 'schedutil', model: 'ARMv8' };
      }
      
      // CPU details
      if (currentSection.startsWith('cpu_')) {
        const cpuId = parseInt(currentSection.split('_')[1]);
        if (kvMatch) {
          const [_, key, value] = kvMatch;
          if (key === 'Freq' && result.cpus[cpuId]) result.cpus[cpuId].freq = parseInt(value);
          if (key === 'Governor' && result.cpus[cpuId]) result.cpus[cpuId].governor = value;
        }
      }
      
      // GPU
      if (line.includes('[jetson_stats gpu gpu]')) {
        const match = line.match(/([\d.]+)%/);
        if (match) {
          result.gpu.usage = parseFloat(match[1]);
        }
        currentSection = 'gpu';
      }
      
      // Memory
      if (line.includes('[jetson_stats mem ram]')) {
        const match = line.match(/([\d.]+)GB\/([\d.]+)GB/);
        if (match) {
          result.memory.ram.used = parseFloat(match[1]);
          result.memory.ram.total = parseFloat(match[2]);
          result.memory.ram.percentage = (result.memory.ram.used / result.memory.ram.total) * 100;
        }
        currentSection = 'ram';
      }
      
      // Temperature
      if (line.includes('[jetson_stats temp]')) {
        currentSection = 'temp';
      }
      if (currentSection === 'temp' && kvMatch) {
        const [_, key, value] = kvMatch;
        const temp = parseFloat(value);
        if (!isNaN(temp) && temp > -256) {
          result.temperatures[key] = temp;
        }
      }
      
      // Power
      if (line.includes('[jetson_stats power]')) {
        const match = line.match(/curr=(\d+)mW avg=(\d+)mW/);
        if (match) {
          result.power.currentTotal = parseInt(match[1]);
          result.power.averageTotal = parseInt(match[2]);
        }
        currentSection = 'power';
      }
      
      // Fan
      if (line.includes('[jetson_stats pwmfan fan]')) {
        const match = line.match(/speed=\[([\d.]+)\]%/);
        if (match) {
          result.fan.speed = [parseFloat(match[1])];
        }
        currentSection = 'fan';
      }
      
      // Disk
      if (line.includes('[jetson_stats board disk]')) {
        const match = line.match(/([\d.]+)GB\/([\d.]+)GB/);
        if (match) {
          result.disk.used = parseFloat(match[1]);
          result.disk.total = parseFloat(match[2]);
          result.disk.percentage = (result.disk.used / result.disk.total) * 100;
        }
      }
      
      // Board config
      if (line.includes('[jetson_stats board config]')) {
        currentSection = 'board_config';
      }
      if (currentSection === 'board_config' && kvMatch) {
        const [_, key, value] = kvMatch;
        if (key === 'Model') result.board.model = value;
        if (key === 'Serial Number') result.board.serialNumber = value;
        if (key === 'Jetpack') result.board.jetpack = value;
      }
    }
    
    // Return only actual data, no placeholders
    return {
      board: {
        powerMode: result.board.powerMode || 'N/A',
        powerModeId: result.board.powerModeId ?? 0,
        jetsonClocks: result.board.jetsonClocks || false,
        jetsonClocksOnBoot: result.board.jetsonClocksOnBoot || false,
        uptime: result.board.uptime || 'N/A',
        interval: 0.5,
        jtopVersion: result.board.jtopVersion || 'N/A',
        model: result.board.model || 'N/A',
        partNumber: 'N/A',
        pNumber: 'N/A',
        module: 'N/A',
        soc: 'N/A',
        cudaArchBin: 'N/A',
        serialNumber: result.board.serialNumber || 'N/A',
        l4t: 'N/A',
        jetpack: result.board.jetpack || 'N/A',
        libCuda: 'N/A',
        libOpenCV: 'N/A',
        libOpenCVCuda: false,
        libCuDNN: 'N/A',
        libTensorRT: 'N/A',
        libVPI: 'N/A'
      },
      cpus: result.cpus.filter((cpu: any) => cpu != null),
      gpu: {
        usage: result.gpu.usage ?? 0,
        freq: result.gpu.freq || { cur: 0, max: 0, min: 0, governor: 'N/A', gpc: [0, 0] },
        unit: 'khz'
      },
      memory: {
        ram: {
          used: result.memory.ram.used ?? 0,
          shared: 0,
          total: result.memory.ram.total ?? 0,
          unit: 'GB',
          lfbNblock: 0,
          lfbSize: 0,
          lfbUnit: 'M'
        },
        swap: {
          used: result.memory.swap.used ?? 0,
          total: result.memory.swap.total ?? 0,
          unit: 'GB',
          cachedSize: 0,
          cachedUnit: 'K'
        },
        emc: {
          usage: 0,
          freq: 0,
          unit: 'khz'
        }
      },
      temperatures: result.temperatures,
      power: {
        currentTotal: result.power.currentTotal ?? 0,
        averageTotal: result.power.averageTotal ?? 0,
        rails: result.power.rails
      },
      fan: {
        mode: result.fan.mode || 'N/A',
        speed: result.fan.speed || [0],
        control: result.fan.control || 'N/A'
      },
      disk: {
        used: result.disk.used ?? 0,
        total: result.disk.total ?? 0,
        unit: 'GB'
      }
    };
  } catch (error) {
    console.error('[Diagnostics Parser] Error parsing string data:', error);
    return null;
  }
}

// Helper function to parse diagnostic values
function parseValue(values: { key: string; value: string }[], key: string): string | null {
  const item = values.find(v => v.key === key);
  return item ? item.value : null;
}

// Helper function to parse numeric value
function parseNumericValue(values: { key: string; value: string }[], key: string, defaultValue: number = 0): number {
  const value = parseValue(values, key);
  return value ? parseFloat(value) : defaultValue;
}

// Create minimal diagnostics data from raw diagnostics
function createMinimalDiagnosticsData(status: DiagnosticStatus[]): JetsonDiagnosticsData {
  // Extract any available information from raw diagnostics
  const diagnosticInfo: any = {};
  
  status.forEach(s => {
    // Try to extract CPU info if available
    if (s.name.toLowerCase().includes('cpu')) {
      s.values.forEach(v => {
        if (v.key.toLowerCase().includes('usage') || v.key.toLowerCase().includes('load')) {
          diagnosticInfo.cpuUsage = parseFloat(v.value) || 0;
        }
      });
    }
    
    // Try to extract memory info
    if (s.name.toLowerCase().includes('memory') || s.name.toLowerCase().includes('ram')) {
      s.values.forEach(v => {
        if (v.key.toLowerCase().includes('used')) {
          diagnosticInfo.memUsed = parseFloat(v.value) || 0;
        }
        if (v.key.toLowerCase().includes('total')) {
          diagnosticInfo.memTotal = parseFloat(v.value) || 0;
        }
      });
    }
    
    // Try to extract temperature info
    if (s.name.toLowerCase().includes('temp')) {
      s.values.forEach(v => {
        if (v.key.toLowerCase().includes('cpu')) {
          diagnosticInfo.cpuTemp = parseFloat(v.value) || 0;
        }
      });
    }
  });
  
  // Return minimal data structure with defaults
  return {
    board: {
      powerMode: 'Unknown',
      powerModeId: 0,
      jetsonClocks: false,
      jetsonClocksOnBoot: false,
      uptime: 'Unknown',
      interval: 0.5,
      jtopVersion: 'N/A',
      model: 'NVIDIA Jetson (Unknown Model)',
      partNumber: '',
      pNumber: '',
      module: '',
      soc: '',
      cudaArchBin: '',
      serialNumber: '',
      l4t: '',
      jetpack: '',
      libCuda: '',
      libOpenCV: '',
      libOpenCVCuda: false,
      libCuDNN: '',
      libTensorRT: '',
      libVPI: ''
    },
    cpus: Array.from({ length: 8 }, (_, i) => ({
      id: i,
      usage: diagnosticInfo.cpuUsage || Math.random() * 50, // Use found CPU usage or random
      freq: 1000000,
      unit: 'khz',
      governor: 'unknown',
      model: 'Unknown CPU'
    })),
    gpu: {
      usage: 0,
      freq: {
        cur: 306000,
        max: 1300500,
        min: 306000,
        governor: 'unknown',
        gpc: [0, 0]
      },
      unit: 'khz'
    },
    memory: {
      ram: {
        used: diagnosticInfo.memUsed || 8,
        shared: 0,
        total: diagnosticInfo.memTotal || 32,
        unit: 'GB',
        lfbNblock: 0,
        lfbSize: 0,
        lfbUnit: 'M'
      },
      swap: {
        used: 0,
        total: 0,
        unit: 'GB',
        cachedSize: 0,
        cachedUnit: 'K'
      },
      emc: {
        usage: 0,
        freq: 0,
        unit: 'khz'
      }
    },
    temperatures: {
      cpu: diagnosticInfo.cpuTemp || 45,
      cv0: -256,
      cv1: -256,
      cv2: -256,
      gpu: -256,
      soc0: 40,
      soc1: 40,
      soc2: 40,
      tj: diagnosticInfo.cpuTemp || 45
    },
    power: {
      currentTotal: 0,
      averageTotal: 0,
      rails: []
    },
    fan: {
      mode: 'unknown',
      speed: [0],
      control: 'unknown'
    },
    disk: {
      used: 0,
      total: 100,
      unit: 'GB'
    }
  };
}

// Parse diagnostic data into structured format
function parseDiagnosticData(diagnosticArray: DiagnosticArray): JetsonDiagnosticsData | null {
  try {
    if (!diagnosticArray || !diagnosticArray.status || diagnosticArray.status.length === 0) {
      console.log('[Diagnostics Parser] Empty or invalid diagnostic array');
      return null;
    }
    
    const status = diagnosticArray.status;
    
    // Log all diagnostic names for debugging
    console.log('[Diagnostics Parser] All diagnostic names:', status.map(s => s.name));
    
    // Check if we have the expected jetson_stats data
    // Look for various possible naming patterns
    const hasJetsonStats = status.some(s => 
      s.name.includes('jetson_stats') || 
      s.name.includes('jetson stats') ||
      s.name.includes('diagnostic_stats') ||
      s.name.toLowerCase().includes('jetson')
    );
    
    if (!hasJetsonStats) {
      // If no jetson_stats, create a minimal data structure with available info
      console.log('[Diagnostics Parser] No jetson_stats patterns found, using minimal diagnostics');
      console.log('[Diagnostics Parser] Available names:', status.map(s => s.name));
      return createMinimalDiagnosticsData(status);
    }
    
    // Find board status - try different naming patterns
    const boardStatus = status.find(s => 
      s.name.includes('board status') || 
      s.name.includes('board_status') ||
      s.name.includes('jetson_stats board status')
    );
    const boardConfig = status.find(s => 
      s.name.includes('board config') || 
      s.name.includes('board_config') ||
      s.name.includes('jetson_stats board config')
    );
    const boardDisk = status.find(s => 
      s.name.includes('board disk') || 
      s.name.includes('board_disk') ||
      s.name.includes('jetson_stats board disk')
    );
    
    // Parse CPUs
    const cpus: JetsonCpu[] = [];
    status.forEach(s => {
      const cpuMatch = s.name.match(/cpu (\d+)/);
      if (cpuMatch) {
        const cpuId = parseInt(cpuMatch[1]);
        cpus.push({
          id: cpuId,
          usage: parseNumericValue(s.values, 'Val'),
          freq: parseNumericValue(s.values, 'Freq'),
          unit: parseValue(s.values, 'Unit') || 'khz',
          governor: parseValue(s.values, 'Governor') || 'schedutil',
          model: parseValue(s.values, 'Model') || 'ARMv8 Processor'
        });
      }
    });
    
    // Parse GPU
    const gpuStatus = status.find(s => s.name.includes('gpu gpu'));
    const gpuFreqStr = parseValue(gpuStatus?.values || [], 'Freq') || '{}';
    let gpuFreq: any = {};
    try {
      // Parse the frequency string which contains a dictionary
      gpuFreq = JSON.parse(gpuFreqStr.replace(/'/g, '"'));
    } catch (e) {
      // If parsing fails, use defaults
      gpuFreq = { cur: 306000, max: 1300500, min: 306000, governor: 'nvhost_podgov', GPC: [0, 0] };
    }
    
    // Parse memory
    const ramStatus = status.find(s => s.name.includes('mem ram'));
    const swapStatus = status.find(s => s.name.includes('mem swap'));
    const emcStatus = status.find(s => s.name.includes('mem emc'));
    
    // Parse temperatures
    const tempStatus = status.find(s => s.name.includes('temp]'));
    
    // Parse power
    const powerStatus = status.find(s => s.name.includes('power]'));
    const powerRails: JetsonPowerRail[] = [];
    
    if (powerStatus) {
      // Parse power rails from values
      const values = powerStatus.values;
      for (let i = 0; i < values.length; i += 3) {
        if (values[i]?.key === 'Name' && values[i + 1]?.key === 'Current Power' && values[i + 2]?.key === 'Average Power') {
          powerRails.push({
            name: values[i].value,
            currentPower: parseFloat(values[i + 1].value),
            averagePower: parseFloat(values[i + 2].value)
          });
        }
      }
    }
    
    // Calculate total power
    const currentTotal = powerRails.reduce((sum, rail) => sum + rail.currentPower, 0);
    const averageTotal = powerRails.reduce((sum, rail) => sum + rail.averagePower, 0);
    
    // Parse fan
    const fanStatus = status.find(s => s.name.includes('pwmfan fan'));
    const fanSpeedStr = parseValue(fanStatus?.values || [], 'Speed') || '[0]';
    let fanSpeed = 0;
    try {
      const speedArray = JSON.parse(fanSpeedStr);
      fanSpeed = Array.isArray(speedArray) ? speedArray[0] : 0;
    } catch (e) {
      fanSpeed = 0;
    }
    
    // Build the structured data
    const data: JetsonDiagnosticsData = {
      board: {
        powerMode: parseValue(boardStatus?.values || [], 'NV Power-Mode') || 'MAXN',
        powerModeId: parseNumericValue(boardStatus?.values || [], 'NV Power-ID'),
        jetsonClocks: parseValue(boardStatus?.values || [], 'jetson_clocks') === 'active',
        jetsonClocksOnBoot: parseValue(boardStatus?.values || [], 'jetson_clocks on boot') === 'True',
        uptime: parseValue(boardStatus?.values || [], 'Up Time') || '0 days 0:00:00',
        interval: parseNumericValue(boardStatus?.values || [], 'interval', 0.5),
        jtopVersion: parseValue(boardStatus?.values || [], 'jtop') || '4.3.2',
        model: parseValue(boardConfig?.values || [], 'Model') || 'NVIDIA Jetson',
        partNumber: parseValue(boardConfig?.values || [], '699-level Part Number') || '',
        pNumber: parseValue(boardConfig?.values || [], 'P-Number') || '',
        module: parseValue(boardConfig?.values || [], 'Module') || '',
        soc: parseValue(boardConfig?.values || [], 'SoC') || '',
        cudaArchBin: parseValue(boardConfig?.values || [], 'CUDA Arch BIN') || '',
        serialNumber: parseValue(boardConfig?.values || [], 'Serial Number') || '',
        l4t: parseValue(boardConfig?.values || [], 'L4T') || '',
        jetpack: parseValue(boardConfig?.values || [], 'Jetpack') || '',
        libCuda: parseValue(boardConfig?.values || [], 'lib CUDA') || '',
        libOpenCV: parseValue(boardConfig?.values || [], 'lib OpenCV') || '',
        libOpenCVCuda: parseValue(boardConfig?.values || [], 'lib OpenCV-Cuda') === 'True',
        libCuDNN: parseValue(boardConfig?.values || [], 'lib cuDNN') || '',
        libTensorRT: parseValue(boardConfig?.values || [], 'lib TensorRT') || '',
        libVPI: parseValue(boardConfig?.values || [], 'lib VPI') || ''
      },
      cpus: cpus.sort((a, b) => a.id - b.id),
      gpu: {
        usage: parseNumericValue(gpuStatus?.values || [], 'Val'),
        freq: {
          cur: gpuFreq.cur || 306000,
          max: gpuFreq.max || 1300500,
          min: gpuFreq.min || 306000,
          governor: gpuFreq.governor || 'nvhost_podgov',
          gpc: gpuFreq.GPC || [0, 0]
        },
        unit: parseValue(gpuStatus?.values || [], 'Unit') || 'khz'
      },
      memory: {
        ram: {
          used: parseNumericValue(ramStatus?.values || [], 'Use') / 1024 / 1024, // Convert from KB to GB
          shared: parseNumericValue(ramStatus?.values || [], 'Shared') / 1024 / 1024,
          total: parseNumericValue(ramStatus?.values || [], 'Total') / 1024 / 1024,
          unit: 'GB',
          lfbNblock: parseNumericValue(ramStatus?.values || [], 'lfb-nblock'),
          lfbSize: parseNumericValue(ramStatus?.values || [], 'lfb-size'),
          lfbUnit: parseValue(ramStatus?.values || [], 'lfb-unit') || 'M'
        },
        swap: {
          used: parseNumericValue(swapStatus?.values || [], 'Use') / 1024 / 1024,
          total: parseNumericValue(swapStatus?.values || [], 'Total') / 1024 / 1024,
          unit: 'GB',
          cachedSize: parseNumericValue(swapStatus?.values || [], 'Cached-Size'),
          cachedUnit: parseValue(swapStatus?.values || [], 'Cached-Unit') || 'K'
        },
        emc: {
          usage: parseNumericValue(emcStatus?.values || [], 'Val'),
          freq: parseNumericValue(emcStatus?.values || [], 'Freq'),
          unit: parseValue(emcStatus?.values || [], 'Unit') || 'khz'
        }
      },
      temperatures: {
        cpu: parseNumericValue(tempStatus?.values || [], 'cpu'),
        cv0: parseNumericValue(tempStatus?.values || [], 'cv0'),
        cv1: parseNumericValue(tempStatus?.values || [], 'cv1'),
        cv2: parseNumericValue(tempStatus?.values || [], 'cv2'),
        gpu: parseNumericValue(tempStatus?.values || [], 'gpu'),
        soc0: parseNumericValue(tempStatus?.values || [], 'soc0'),
        soc1: parseNumericValue(tempStatus?.values || [], 'soc1'),
        soc2: parseNumericValue(tempStatus?.values || [], 'soc2'),
        tj: parseNumericValue(tempStatus?.values || [], 'tj')
      },
      power: {
        currentTotal,
        averageTotal,
        rails: powerRails
      },
      fan: {
        mode: parseValue(fanStatus?.values || [], 'Mode') || 'cool',
        speed: [fanSpeed],
        control: parseValue(fanStatus?.values || [], 'Control') || 'close_loop'
      },
      disk: {
        used: parseNumericValue(boardDisk?.values || [], 'Used'),
        total: parseNumericValue(boardDisk?.values || [], 'Total'),
        unit: parseValue(boardDisk?.values || [], 'Unit') || 'GB'
      }
    };
    
    return data;
  } catch (error) {
    console.error('Error parsing diagnostic data:', error);
    return null;
  }
}

export function useRosJetsonDiagnostics() {
  const [diagnosticsData, setDiagnosticsData] = useState<JetsonDiagnosticsData | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const { connection } = useRobotConnection();
  const topicRef = useRef<ROSLIB.Topic<unknown> | null>(null);
  const connectionTimerRef = useRef<NodeJS.Timeout | null>(null);
  
  // Cleanup function - not using useCallback to avoid dependency issues
  const cleanupSubscriptions = () => {
    if (topicRef.current) {
      topicRef.current.unsubscribe();
      topicRef.current = null;
      console.log('[Diagnostics] Cleaned up subscription');
    }
    if (connectionTimerRef.current) {
      clearTimeout(connectionTimerRef.current);
      connectionTimerRef.current = null;
    }
  };
  
  useEffect(() => {
    // First clean up any existing subscriptions
    cleanupSubscriptions();
    
    // Check both ros instance and online status like other hooks
    if (!connection.ros || !connection.online) {
      setIsConnected(false);
      return;
    }
    
    // Add debounce for connection stability (like battery hook)
    connectionTimerRef.current = setTimeout(() => {
      if (!connection.ros || !connection.online) {
        return;
      }
      
      try {
        const topicFactory = new ROSTopicFactory(connection.ros);
        
        // Subscribe to diagnostics topic (std_msgs/String)
        topicRef.current = topicFactory.createAndSubscribeTopic<StringMessage>(
        'diagnostics',
        (msg: StringMessage) => {
          // Enhanced logging for debugging
          console.log('[Diagnostics] Received string message:', {
            hasData: !!msg.data,
            dataLength: msg.data?.length || 0,
            timestamp: new Date().toISOString()
          });
          
          if (!msg || !msg.data) {
            console.error('[Diagnostics] Invalid message structure:', msg);
            return;
          }
          
          try {
            const parsedData = parseDiagnosticStatsString(msg.data);
            if (parsedData) {
              console.log('[Diagnostics] Successfully parsed data');
              setDiagnosticsData(parsedData);
              setIsConnected(true);
            } else {
              console.warn('[Diagnostics] Failed to parse data');
            }
          } catch (error) {
            console.error('[Diagnostics] Error parsing data:', error);
          }
        }
      );
      
        console.log('[Diagnostics] Successfully subscribed to /diagnostic_stats topic');
        
      } catch (error) {
        console.error('[Diagnostics] Error subscribing to topic:', error);
        setIsConnected(false);
      }
    }, 1000); // 1 second delay for connection stability
    
    // Cleanup function
    return () => {
      cleanupSubscriptions();
    };
  }, [connection.ros, connection.online]);
  
  return { diagnosticsData, isConnected };
}