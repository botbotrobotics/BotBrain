import JSZip from 'jszip';

export interface MapMetadata {
  image: string;
  resolution: number;
  origin: [number, number, number];
  negate: number;
  occupied_thresh: number;
  free_thresh: number;
  [key: string]: any; // For preserving additional keys
}

export interface MapData {
  width: number;
  height: number;
  pixels: Uint8Array;
  metadata: MapMetadata;
}

// Parse PGM file format
export function parsePGM(buffer: ArrayBuffer): { width: number; height: number; pixels: Uint8Array } {
  const view = new DataView(buffer);
  const decoder = new TextDecoder('ascii');
  
  let pos = 0;
  let width = 0;
  let height = 0;
  let maxVal = 0;
  
  // Read header
  const header: string[] = [];
  let inComment = false;
  let currentToken = '';
  
  while (pos < buffer.byteLength && header.length < 4) {
    const byte = view.getUint8(pos++);
    const char = String.fromCharCode(byte);
    
    if (char === '#') {
      inComment = true;
    } else if (char === '\n') {
      if (!inComment && currentToken) {
        header.push(currentToken);
        currentToken = '';
      }
      inComment = false;
    } else if (!inComment) {
      if (char === ' ' || char === '\t' || char === '\r') {
        if (currentToken) {
          header.push(currentToken);
          currentToken = '';
        }
      } else {
        currentToken += char;
      }
    }
  }
  
  if (currentToken) {
    header.push(currentToken);
  }
  
  // Parse header values
  if (header[0] !== 'P5') {
    throw new Error('Invalid PGM file format. Expected P5.');
  }
  
  width = parseInt(header[1]);
  height = parseInt(header[2]);
  maxVal = parseInt(header[3]);
  
  if (maxVal !== 255) {
    throw new Error('Only 8-bit PGM files are supported.');
  }
  
  // Read pixel data
  const pixelCount = width * height;
  const pixels = new Uint8Array(pixelCount);
  
  for (let i = 0; i < pixelCount; i++) {
    pixels[i] = view.getUint8(pos + i);
  }
  
  return { width, height, pixels };
}

// Parse YAML file
export function parseYAML(content: string): MapMetadata {
  const metadata: any = {};
  const lines = content.split('\n');
  
  for (const line of lines) {
    const trimmed = line.trim();
    if (!trimmed || trimmed.startsWith('#')) continue;
    
    const colonIndex = trimmed.indexOf(':');
    if (colonIndex === -1) continue;
    
    const key = trimmed.substring(0, colonIndex).trim();
    const value = trimmed.substring(colonIndex + 1).trim();
    
    // Parse origin array
    if (key === 'origin') {
      const matches = value.match(/\[([^\]]+)\]/);
      if (matches) {
        metadata.origin = matches[1].split(',').map(v => parseFloat(v.trim()));
      }
    } else if (key === 'resolution' || key === 'occupied_thresh' || key === 'free_thresh') {
      metadata[key] = parseFloat(value);
    } else if (key === 'negate') {
      metadata[key] = parseInt(value);
    } else {
      metadata[key] = value;
    }
  }
  
  return metadata as MapMetadata;
}

// Generate YAML content
export function generateYAML(metadata: MapMetadata): string {
  const lines: string[] = [];
  
  // Required fields first
  lines.push(`image: ${metadata.image}`);
  lines.push(`resolution: ${metadata.resolution}`);
  lines.push(`origin: [${metadata.origin.join(', ')}]`);
  lines.push(`negate: ${metadata.negate}`);
  lines.push(`occupied_thresh: ${metadata.occupied_thresh}`);
  lines.push(`free_thresh: ${metadata.free_thresh}`);
  
  // Add any additional fields
  for (const [key, value] of Object.entries(metadata)) {
    if (!['image', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'].includes(key)) {
      lines.push(`${key}: ${value}`);
    }
  }
  
  return lines.join('\n');
}

// Convert canvas to PGM format
export function canvasToPGM(canvas: HTMLCanvasElement): ArrayBuffer {
  const ctx = canvas.getContext('2d')!;
  const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
  const { width, height, data } = imageData;
  
  // Create PGM header
  const header = `P5\n${width} ${height}\n255\n`;
  const headerBytes = new TextEncoder().encode(header);
  
  // Create buffer for header + pixel data
  const buffer = new ArrayBuffer(headerBytes.length + width * height);
  const view = new Uint8Array(buffer);
  
  // Write header
  view.set(headerBytes, 0);
  
  // Write pixel data (convert RGBA to grayscale)
  let pixelOffset = headerBytes.length;
  for (let i = 0; i < data.length; i += 4) {
    // Use red channel as grayscale value (since we're using exact colors)
    view[pixelOffset++] = data[i];
  }
  
  return buffer;
}

// Quantize color to Nav2 palette
export function quantizeColor(r: number, g: number, b: number, occupiedThresh: number, freeThresh: number): [number, number, number] {
  const gray = Math.round(0.299 * r + 0.587 * g + 0.114 * b);
  const normalized = gray / 255;
  
  if (normalized < occupiedThresh) {
    return [0, 0, 0]; // Occupied (black)
  } else if (normalized > freeThresh) {
    return [255, 255, 255]; // Free (white)
  } else {
    // Unknown (mid-grey between thresholds)
    const unknownValue = Math.round(((occupiedThresh + freeThresh) / 2) * 255);
    return [unknownValue, unknownValue, unknownValue];
  }
}

// Create a zip file containing map files
export async function createMapZip(pgmBuffer: ArrayBuffer, yamlContent: string, additionalFiles?: { name: string; content: ArrayBuffer }[]): Promise<Blob> {
  const zip = new JSZip();
  
  // Add PGM file
  zip.file('map.pgm', pgmBuffer);
  
  // Add YAML file
  zip.file('map.yaml', yamlContent);
  
  // Add additional files if any
  if (additionalFiles) {
    for (const file of additionalFiles) {
      zip.file(file.name, file.content);
    }
  }
  
  // Generate zip file
  const blob = await zip.generateAsync({ type: 'blob' });
  return blob;
} 