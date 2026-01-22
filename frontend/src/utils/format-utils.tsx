/**
 * Format bytes to human-readable format with appropriate units
 * @param bytes Number of bytes
 * @returns Formatted string with appropriate units (B, KB, MB, GB)
 */
export function formatBytes(bytes: number, decimals: number = 1): string {
  if (bytes === 0) return '0 B';
  
  const k = 1024;
  const sizes = ['B', 'KB', 'MB', 'GB'];
  
  const i = Math.floor(Math.log(bytes) / Math.log(k));
  const value = bytes / Math.pow(k, i);
  
  return `${value.toFixed(decimals)} ${sizes[i]}`;
}

/**
 * Format latency to human-readable format with appropriate units
 * @param ms Latency in milliseconds
 * @returns Formatted string with appropriate units (ms or s)
 */
export function formatLatency(ms: number): string {
  if (ms < 0) return 'N/A';
  if (ms < 1000) return `${Math.round(ms)} ms`;
  return `${(ms / 1000).toFixed(2)} s`;
}

/**
 * Format data rate to human-readable format
 * @param bytesPerSec Data rate in bytes per second
 * @returns Formatted string with appropriate units (Bps, KBps, MBps)
 */
export function formatDataRate(bytesPerSec: number): string {
  return `${formatBytes(bytesPerSec)}/s`;
} 