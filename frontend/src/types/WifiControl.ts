// WiFi Control Type Definitions

export enum WifiAuthType {
  OPEN = 'open',
  PSK = 'psk',
  WPA2_ENTERPRISE = 'wpa2-enterprise'
}

export enum NetworkMode {
  WIFI = 'wifi',
  FOURGE = '4g',
  HOTSPOT = 'hotspot',
  OFFLINE = 'offline'
}

export interface WifiNetwork {
  ssid: string;
  signal: number; // Signal strength in dBm
  security: WifiAuthType;
  inUse?: boolean; // Currently connected
  rate?: string; // Connection rate (e.g., "54 Mbit/s")
  channel?: number;
  bars?: number; // Signal bars (1-5)
}

// Service request/response types

// GetAvailableNetworks service
export interface GetAvailableNetworksRequest {
  rescan?: boolean;
  interface?: string;
}

export interface GetAvailableNetworksResponse {
  success: boolean;
  message: string; // This contains the raw nmcli output
  networks?: string; // Fallback for backwards compatibility
}

// ConnectWifi service
export interface ConnectWifiRequest {
  ssid: string;
  auth_type: WifiAuthType;
  psk?: string; // For PSK networks
  identity?: string; // For enterprise networks
  password?: string; // For enterprise networks
  save_profile?: boolean;
}

export interface ConnectWifiResponse {
  success: boolean;
  message?: string;
}

// SetBool service (for wifi_radio)
export interface SetBoolRequest {
  data: boolean;
}

export interface SetBoolResponse {
  success: boolean;
  message: string;
}

// Trigger service (for saved_networks, check_4g, update_network_status)
export interface TriggerRequest {}

export interface TriggerResponse {
  success: boolean;
  message: string;
}

// Watchdog parameters
export interface WifiWatchdogParams {
  watchdog_enabled: boolean;
  check_interval: number; // seconds
  wifi_ssid?: string;
  wifi_pass?: string;
}

// UI State types
export interface WifiConnectionState {
  isConnected: boolean;
  currentNetwork?: string;
  signal?: number;
  ipAddress?: string;
  mode: NetworkMode;
}

export interface WifiControlState {
  isScanning: boolean;
  isConnecting: boolean;
  availableNetworks: WifiNetwork[];
  savedNetworks: string[];
  connectionState: WifiConnectionState;
  radioEnabled: boolean;
  fourGStatus?: string;
  watchdogParams?: WifiWatchdogParams;
}

// Helper function to parse signal strength to bars
export function signalToBars(signal: number): number {
  if (signal >= -50) return 5;
  if (signal >= -60) return 4;
  if (signal >= -70) return 3;
  if (signal >= -80) return 2;
  return 1;
}

// Helper function to parse nmcli output
export function parseNetworkList(nmcliOutput: string): WifiNetwork[] {
  if (!nmcliOutput || nmcliOutput.trim() === '') {
    return [];
  }

  // Check if this is just a simple success message
  const trimmedOutput = nmcliOutput.trim().toLowerCase();
  if (trimmedOutput === 'ok' || trimmedOutput === 'success' || trimmedOutput === 'true') {
    console.log('Received success message instead of network data');
    return [];
  }

  const lines = nmcliOutput.split('\n').filter(line => line.trim());
  const networks: WifiNetwork[] = [];

  // Debug log to see what we're parsing
  console.log('Parsing nmcli output:', nmcliOutput);

  // Skip header line if present (contains SSID, BSSID, etc.)
  let startIndex = 0;
  if (lines[0] && (
    lines[0].includes('SSID') ||
    lines[0].includes('BSSID') ||
    lines[0].includes('MODE') ||
    lines[0].includes('CHAN')
  )) {
    startIndex = 1;
  }

  for (let i = startIndex; i < lines.length; i++) {
    const line = lines[i].trim();
    if (!line) continue;

    // nmcli output format can vary, but typical format is:
    // IN-USE  BSSID              SSID            MODE   CHAN  RATE        SIGNAL  BARS  SECURITY
    // *       XX:XX:XX:XX:XX:XX  MyNetwork       Infra  6     195 Mbit/s  100     ████  WPA2
    //         YY:YY:YY:YY:YY:YY  OtherNetwork    Infra  11    130 Mbit/s  70      ███   WPA2 WPA3

    // Split by multiple spaces (at least 2) to handle SSIDs with single spaces
    const parts = line.split(/\s{2,}/).map(p => p.trim()).filter(p => p);

    // Also try splitting by single spaces if we didn't get enough parts
    if (parts.length < 5) {
      const altParts = line.split(/\s+/).filter(p => p);
      if (altParts.length > parts.length) {
        // Use alternative parsing
        const inUse = altParts[0] === '*';
        const startIdx = inUse ? 1 : 0;

        // Skip BSSID (MAC address format XX:XX:XX:XX:XX:XX)
        let ssidIdx = startIdx;
        if (altParts[startIdx] && altParts[startIdx].includes(':')) {
          ssidIdx = startIdx + 1;
        }

        const ssid = altParts[ssidIdx];
        if (!ssid || ssid === '--' || ssid.includes(':')) continue;

        // Default values
        let signal = -80;
        let bars = 2;
        let security = WifiAuthType.OPEN;

        // Look for signal strength (number between 0-100)
        for (let j = ssidIdx + 1; j < altParts.length; j++) {
          if (/^\d+$/.test(altParts[j])) {
            const num = parseInt(altParts[j]);
            if (num <= 100) {
              signal = -100 + num; // Convert percentage to dBm approximation
              bars = signalToBars(signal);
              break;
            }
          }
        }

        // Look for security type
        const lineUpper = line.toUpperCase();
        if (lineUpper.includes('WPA3') || lineUpper.includes('WPA2') || lineUpper.includes('WPA')) {
          security = lineUpper.includes('ENTERPRISE') ? WifiAuthType.WPA2_ENTERPRISE : WifiAuthType.PSK;
        } else if (lineUpper.includes('WEP')) {
          security = WifiAuthType.PSK;
        }

        networks.push({
          ssid,
          signal,
          security,
          inUse,
          bars
        });
        continue;
      }
    }

    // Parse with the properly split parts
    const inUse = parts[0] === '*';
    let ssidIdx = inUse ? 2 : 1;

    // Check if the first part after in-use indicator is BSSID
    if (parts[inUse ? 1 : 0] && parts[inUse ? 1 : 0].includes(':')) {
      // This is BSSID, SSID is next
      ssidIdx = inUse ? 2 : 1;
    } else {
      // No BSSID, this might be SSID
      ssidIdx = inUse ? 1 : 0;
    }

    const ssid = parts[ssidIdx];
    if (!ssid || ssid === '--' || ssid.includes(':')) continue;

    // Find signal and security in remaining parts
    let signal = -80; // Default
    let bars = 2; // Default
    let security = WifiAuthType.OPEN;

    for (let j = ssidIdx + 1; j < parts.length; j++) {
      const part = parts[j];

      // Check for signal (number)
      if (/^\d+$/.test(part)) {
        const num = parseInt(part);
        if (num <= 100) {
          signal = -100 + num;
          bars = signalToBars(signal);
        }
      }

      // Check for bars visualization
      if (part.includes('▂') || part.includes('▄') || part.includes('▆') || part.includes('█') || part.includes('_')) {
        bars = Math.min(5, Math.max(1, part.replace(/_/g, '').length));
      }

      // Check for security
      if (part.toUpperCase().includes('WPA') || part.toUpperCase().includes('WEP')) {
        security = part.toUpperCase().includes('ENTERPRISE') ? WifiAuthType.WPA2_ENTERPRISE : WifiAuthType.PSK;
      }
    }

    networks.push({
      ssid,
      signal,
      security,
      inUse,
      bars
    });
  }

  // Remove duplicates and sort by signal strength
  const uniqueNetworks = networks.filter((network, index, self) =>
    index === self.findIndex(n => n.ssid === network.ssid)
  );

  return uniqueNetworks.sort((a, b) => b.signal - a.signal);
}