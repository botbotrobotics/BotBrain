'use client';

import { ComponentProps, useCallback, useEffect, useRef, useState } from 'react';
import * as ROSLIB from 'roslib';
import { Layers, Loader, Maximize2, Minimize2, VideoOff } from 'lucide-react';
import { useLanguage } from '@/contexts/LanguageContext';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import useWhepStream from '@/hooks/useWhepStream';
import {
  DepthRange,
  RealsenseSource,
  RealsenseStream,
  colorizeDepth,
  depthFromLuma,
  framePixelAt,
  whepUrl,
} from '@/utils/realsense';
import RosCameraImg from './ros-camera-img';

/** RosCameraImg's props, which every live camera view takes. */
export type CameraViewProps = ComponentProps<typeof RosCameraImg>;

export function CameraLoading() {
  const { t } = useLanguage();
  return (
    <span className="w-full h-full border-2 border-gray-200 dark:border-black rounded-lg flex flex-col items-center justify-center text-center">
      <Loader className="w-5 h-5 animate-spin" />
      {t('robotCams', 'initializingCamera')}
    </span>
  );
}

const FAILURES_BEFORE_MESSAGE = 3;
const LIFECYCLE_POLL_MS = 3000;
const LIFECYCLE_TIMEOUT_MS = 5000;

/** The camera node's lifecycle as far as video is concerned: `on` also covers "could not tell". */
type NodePower = 'checking' | 'on' | { off: string };

function lifecycleLabel(ros: ROSLIB.Ros, nodeName: string): Promise<string | null> {
  return new Promise((resolve) => {
    window.setTimeout(() => resolve(null), LIFECYCLE_TIMEOUT_MS);
    new ROSLIB.Service({ ros, name: `${nodeName}/get_state`, serviceType: 'lifecycle_msgs/srv/GetState' }).callService(
      {},
      (response: unknown) => {
        const state = typeof response === 'object' && response && 'current_state' in response ? response.current_state : null;
        resolve(typeof state === 'object' && state && 'label' in state && typeof state.label === 'string' ? state.label : null);
      },
      () => resolve(null)
    );
  });
}

/** An inactive node still answers rosapi, so detection alone cannot tell whether it is active. */
function useNodePower(ros: ROSLIB.Ros | undefined, nodeName: string): [NodePower, () => void] {
  const [power, setPower] = useState<NodePower>('checking');
  const [asks, setAsks] = useState(0);
  const recheck = useCallback(() => setAsks((n) => n + 1), []);

  useEffect(() => {
    if (!ros) return;
    let cancelled = false;
    let timer: number | undefined;
    const ask = async () => {
      const label = await lifecycleLabel(ros, nodeName);
      if (cancelled) return;
      if (label === null || label === 'active') {
        setPower('on');
        return;
      }
      setPower((previous) => (typeof previous === 'object' && previous.off === label ? previous : { off: label }));
      timer = window.setTimeout(ask, LIFECYCLE_POLL_MS);
    };
    ask();
    return () => {
      cancelled = true;
      window.clearTimeout(timer);
    };
  }, [ros, nodeName, asks]);

  return [power, recheck];
}

export default function RealsenseCamera({
  source,
  rosbridgeUrl,
  ...props
}: CameraViewProps & { source: RealsenseSource; rosbridgeUrl: string }) {
  const { cameraType } = props;
  const { t } = useLanguage();
  const { ros } = useRobotConnection().connection;
  const [view, setView] = useState<RealsenseStream>('color');
  const containerRef = useRef<HTMLDivElement>(null);
  const [laidOut, setLaidOut] = useState(false);
  const url = whepUrl(rosbridgeUrl, source.node, view);
  // Pages may mount a copy per breakpoint and hide the others with CSS; hidden copies stay idle.
  // Video waits while the node is off, so retries neither spin nor wake MediaMTX's on-demand pull.
  const [power, recheckPower] = useNodePower(laidOut ? ros : undefined, source.nodeName);
  const { stream, failure, attempts } = useWhepStream(laidOut && power === 'on' ? url : null);

  useEffect(() => {
    if (failure) recheckPower();
  }, [failure, recheckPower]);

  const [isFullScreen, setIsFullScreen] = useState(false);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    const observer = new ResizeObserver(([entry]) =>
      setLaidOut(entry.contentRect.width > 0 && entry.contentRect.height > 0)
    );
    observer.observe(container);
    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    const onChange = () => setIsFullScreen(document.fullscreenElement === containerRef.current);
    document.addEventListener('fullscreenchange', onChange);
    return () => document.removeEventListener('fullscreenchange', onChange);
  }, []);

  const toggleFullScreen = () => {
    const change = document.fullscreenElement ? document.exitFullscreen() : containerRef.current?.requestFullscreen();
    change?.catch((err) => console.error('Error toggling full-screen mode:', err));
  };

  const color = stream && view === 'color';
  const fullScreenLabel = isFullScreen ? t('robotCams', 'exitFullscreen') : t('robotCams', 'enterFullscreen');
  const viewLabel = view === 'color' ? t('robotCams', 'depthView') : t('robotCams', 'colorView');
  const rejected = failure?.kind === 'http' && failure.status >= 400 && failure.status < 500;
  let notice: string | null = null;
  if (typeof power === 'object') {
    notice = t('robotCams', 'cameraOff').replace('{state}', power.off);
  } else if (failure && (failure.kind === 'stalled' || rejected || attempts >= FAILURES_BEFORE_MESSAGE)) {
    notice = {
      unreachable: t('robotCams', 'videoUnreachable'),
      http: t('robotCams', 'videoRefused'),
      media: t('robotCams', 'videoMediaFailed'),
      stalled: t('robotCams', 'cameraPaused'),
    }[failure.kind]
      .replace('{server}', new URL(url).host)
      .replace('{status}', failure.kind === 'http' ? String(failure.status) : '');
  }

  return (
    <div
      className={`w-full h-full relative flex items-center justify-center rounded-lg ${isFullScreen ? 'fullscreen-camera' : ''}`}
      ref={containerRef}
      data-realsense-view={cameraType}
    >
      {color ? (
        <RosCameraImg {...props} stream={stream} />
      ) : stream ? (
        <div className={`relative w-full h-full overflow-hidden ${isFullScreen ? '' : 'rounded-lg border-2 border-gray-200 dark:border-black'}`}>
          <DepthView
            stream={stream}
            canvasId={`camera-canvas-${cameraType}`}
            depthRange={source.depthRange}
            noDepthLabel={t('robotCams', 'noDepth')}
          />
        </div>
      ) : notice ? (
        <span className="w-full h-full border-2 border-gray-200 dark:border-black rounded-lg flex flex-col items-center justify-center gap-1 px-4 text-center text-sm">
          <VideoOff className="w-5 h-5" />
          {notice}
        </span>
      ) : (
        <CameraLoading />
      )}
      {/* RosCameraImg has its own full-screen button; the view toggle sits next to it. */}
      <div className={`absolute top-3 flex gap-2 ${color ? 'left-12' : 'left-3'}`}>
        {!color && (
          <button
            className="bg-black/50 hover:bg-black/70 text-white rounded-lg p-1.5 transition-colors"
            onClick={toggleFullScreen}
            aria-label={fullScreenLabel}
            title={fullScreenLabel}
          >
            {isFullScreen ? <Minimize2 className="w-4 h-4" /> : <Maximize2 className="w-4 h-4" />}
          </button>
        )}
        <button
          className={`${view === 'depth' ? 'bg-cyan-600/80 hover:bg-cyan-600' : 'bg-black/50 hover:bg-black/70'} text-white rounded-lg p-1.5 transition-colors`}
          onClick={() => setView(view === 'color' ? 'depth' : 'color')}
          aria-pressed={view === 'depth'}
          aria-label={viewLabel}
          title={viewLabel}
        >
          <Layers className="w-4 h-4" />
        </button>
      </div>
    </div>
  );
}

function DepthView({
  stream,
  canvasId,
  depthRange,
  noDepthLabel,
}: {
  stream: MediaStream;
  canvasId: string;
  depthRange: DepthRange;
  noDepthLabel: string;
}) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const videoRef = useRef<HTMLVideoElement>(null);
  const readoutRef = useRef<HTMLSpanElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    const video = videoRef.current;
    const readout = readoutRef.current;
    const ctx = canvas?.getContext('2d', { willReadFrequently: true });
    if (!canvas || !video || !readout || !ctx) return;
    const frameCallbacks = 'requestVideoFrameCallback' in video;
    let luma = new Uint8Array(0);
    let pointer: { x: number; y: number } | null = null;
    let pending = 0;

    const showDepth = () => {
      let text = '';
      if (pointer && luma.length) {
        const index = framePixelAt(canvas.width, canvas.height, canvas.clientWidth, canvas.clientHeight, pointer.x, pointer.y);
        if (index !== null) {
          const metres = depthFromLuma(luma[index], depthRange);
          text = metres === null ? noDepthLabel : `${metres.toFixed(2)} m`;
        }
      }
      readout.textContent = text;
    };

    const draw = () => {
      const { videoWidth: width, videoHeight: height } = video;
      if (width && height) {
        if (canvas.width !== width || canvas.height !== height) {
          canvas.width = width;
          canvas.height = height;
        }
        if (luma.length !== width * height) luma = new Uint8Array(width * height);
        ctx.drawImage(video, 0, 0, width, height);
        const image = ctx.getImageData(0, 0, width, height);
        colorizeDepth(image.data, luma);
        ctx.putImageData(image, 0, 0);
        showDepth();
      }
      pending = frameCallbacks ? video.requestVideoFrameCallback(draw) : requestAnimationFrame(draw);
    };

    const onMove = (event: PointerEvent) => {
      pointer = { x: event.offsetX, y: event.offsetY };
      showDepth();
    };
    const onLeave = () => {
      pointer = null;
      showDepth();
    };
    canvas.addEventListener('pointermove', onMove);
    canvas.addEventListener('pointerleave', onLeave);
    video.srcObject = stream;
    video.play().catch((error) => console.warn('Depth video did not start:', error));
    draw();
    return () => {
      if (frameCallbacks) video.cancelVideoFrameCallback(pending);
      else cancelAnimationFrame(pending);
      video.pause();
      video.srcObject = null;
      canvas.removeEventListener('pointermove', onMove);
      canvas.removeEventListener('pointerleave', onLeave);
    };
  }, [stream, depthRange, noDepthLabel]);

  // The <video> stays laid out under the canvas: some browsers skip frame callbacks for detached media.
  return (
    <>
      <video ref={videoRef} className="absolute inset-0 w-full h-full object-contain" muted playsInline />
      <canvas ref={canvasRef} id={canvasId} className="relative w-full h-full object-contain cursor-crosshair" />
      <span
        ref={readoutRef}
        className="absolute bottom-3 left-3 rounded-md bg-black/60 px-2 py-1 font-mono text-xs text-white pointer-events-none empty:hidden"
      />
    </>
  );
}
