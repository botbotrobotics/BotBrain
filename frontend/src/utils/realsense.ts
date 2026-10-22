import type { CameraType } from '@/types/CameraType';

/** bot_realsense node names; each serves `<node>/color` and `<node>/depth` through MediaMTX. */
export type RealsenseNode = 'front_camera' | 'back_camera';
export type RealsenseStream = 'color' | 'depth';

/** The node's depth_min_m / depth_max_m: the distances its grey-depth luma 16 and 235 stand for. */
export interface DepthRange {
  minM: number;
  maxM: number;
}

export interface RealsenseSource {
  node: RealsenseNode;
  /** Full ROS name of the node, namespace included, e.g. `/front_camera` or `/robot1/front_camera`. */
  nodeName: string;
  depthRange: DepthRange;
}

export const REALSENSE_NODES: Partial<Record<CameraType, RealsenseNode>> = {
  camera: 'front_camera',
  back: 'back_camera',
};

/** Where the browser reaches MediaMTX's WHEP endpoints. */
export type WebrtcServer =
  /** A fixed base URL, e.g. a TLS reverse proxy: `<url>/<node>/<stream>/whep`. */
  | { kind: 'url'; url: string }
  /** The host the frontend reaches rosbridge at, on this port, over http (https if rosbridge is wss). */
  | { kind: 'port'; port: number };

/**
 * NEXT_PUBLIC_WEBRTC_URL wins over NEXT_PUBLIC_WEBRTC_PORT (default 8889). Both are build-time
 * config, so a bad value throws at load rather than quietly failing every stream.
 */
export function parseWebrtcServer(url: string | undefined, port: string | undefined): WebrtcServer {
  if (url) {
    let parsed: URL | null = null;
    try {
      parsed = new URL(url);
    } catch {
      // reported below
    }
    if (!parsed || (parsed.protocol !== 'http:' && parsed.protocol !== 'https:') || parsed.search || parsed.hash) {
      throw new Error(`NEXT_PUBLIC_WEBRTC_URL must be an http(s) base URL without query or fragment, got "${url}"`);
    }
    return { kind: 'url', url: parsed.href.replace(/\/+$/, '') };
  }
  if (port === undefined || port === '') return { kind: 'port', port: 8889 };
  const value = Number(port);
  if (!Number.isInteger(value) || value < 1 || value > 65535) {
    throw new Error(`NEXT_PUBLIC_WEBRTC_PORT must be a TCP port, got "${port}"`);
  }
  return { kind: 'port', port: value };
}

// Literal process.env reads: Next.js inlines NEXT_PUBLIC_* only where they are spelled out.
const WEBRTC_SERVER = parseWebrtcServer(process.env.NEXT_PUBLIC_WEBRTC_URL, process.env.NEXT_PUBLIC_WEBRTC_PORT);

export function whepUrl(
  rosbridgeUrl: string,
  node: RealsenseNode,
  stream: RealsenseStream,
  server: WebrtcServer = WEBRTC_SERVER
): string {
  let base: string;
  if (server.kind === 'url') {
    base = server.url;
  } else {
    const { protocol, hostname } = new URL(rosbridgeUrl);
    base = `${protocol === 'wss:' ? 'https' : 'http'}://${hostname}:${server.port}`;
  }
  return `${base}/${node}/${stream}/whep`;
}

/** Luma 16..235 spans the range; codec noise outside it saturates. */
function lumaToT(y: number): number {
  return Math.min(Math.max((y - 16) / 219, 0), 1);
}

/**
 * Metres for a decoded depth luma value, or null where the camera saw no depth (Y < 8).
 * Inverse of the node's t = (zmin^-0.5 - z^-0.5) / (zmin^-0.5 - zmax^-0.5), Y = 16 + 219 t.
 */
export function depthFromLuma(y: number, { minM, maxM }: DepthRange): number | null {
  if (y < 8) return null;
  return (minM ** -0.5 - lumaToT(y) * (minM ** -0.5 - maxM ** -0.5)) ** -2;
}

/** Google's polynomial fit of the Turbo colormap, t in [0, 1] -> 0..255 RGB. */
function turbo(t: number): [number, number, number] {
  const r = 0.13572138 + t * (4.6153926 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))));
  const g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))));
  const b = 0.1066733 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))));
  const byte = (c: number) => Math.round(255 * Math.min(Math.max(c, 0), 1));
  return [byte(r), byte(g), byte(b)];
}

/**
 * Opaque colour per luma value, near = red and far = blue, no depth = black.
 * One 32-bit word per entry in ImageData byte order, so a frame colours with one lookup per pixel.
 */
export const DEPTH_LUT: Uint32Array = (() => {
  const rgba = new Uint8ClampedArray(256 * 4);
  for (let y = 0; y < 256; y++) {
    // Turbo below ~0.1 is near-black and would read as "no depth", so far maps to 0.1.
    if (y >= 8) rgba.set(turbo(1 - 0.9 * lumaToT(y)), y * 4);
    rgba[y * 4 + 3] = 255;
  }
  return new Uint32Array(rgba.buffer);
})();

/**
 * Colours a grey depth picture in place and keeps its luma. The stream is grey and flagged full
 * range, so after the browser's YUV to RGB conversion every pixel's R equals the decoded Y.
 */
export function colorizeDepth(rgba: Uint8ClampedArray, luma: Uint8Array): void {
  const out = new Uint32Array(rgba.buffer, rgba.byteOffset, luma.length);
  for (let i = 0; i < luma.length; i++) {
    const y = rgba[i * 4];
    luma[i] = y;
    out[i] = DEPTH_LUT[y];
  }
}

/** Where a srcWidth x srcHeight picture lands when fitted object-fit: contain into a dstWidth x dstHeight box. */
export function containRect(srcWidth: number, srcHeight: number, dstWidth: number, dstHeight: number) {
  const srcRatio = srcWidth / srcHeight;
  if (srcRatio > dstWidth / dstHeight) {
    const height = dstWidth / srcRatio;
    return { offsetX: 0, offsetY: (dstHeight - height) / 2, width: dstWidth, height };
  }
  const width = dstHeight * srcRatio;
  return { offsetX: (dstWidth - width) / 2, offsetY: 0, width, height: dstHeight };
}

/**
 * Index into a width x height frame shown object-fit: contain in a box, for a point in box
 * coordinates; null on the letterbox bars or outside.
 */
export function framePixelAt(
  width: number,
  height: number,
  boxWidth: number,
  boxHeight: number,
  x: number,
  y: number
): number | null {
  const bounds = containRect(width, height, boxWidth, boxHeight);
  const px = Math.floor(((x - bounds.offsetX) * width) / bounds.width);
  const py = Math.floor(((y - bounds.offsetY) * height) / bounds.height);
  if (px < 0 || px >= width || py < 0 || py >= height) return null;
  return py * width + px;
}
