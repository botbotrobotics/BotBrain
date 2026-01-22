'use client';

import type React from 'react';
import { useEffect, useRef, useState } from 'react';
import { Maximize, Minimize } from 'lucide-react';

interface CameraViewProps {
  label: string;
  className?: string;
  width?: number;
  height?: number;
}

export default function CameraView({
  label,
  className,
  width,
  height,
}: CameraViewProps) {
  const [isFullscreen, setIsFullscreen] = useState(false);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const requestRef = useRef<number>(0);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const createVideoFrame = () => {
      ctx.fillStyle = '#000000';
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      const time = Date.now() / 1000;

      ctx.fillStyle = '#8A2BE2';
      ctx.beginPath();
      ctx.arc(
        canvas.width / 2 + Math.cos(time) * 50,
        canvas.height / 2 + Math.sin(time) * 50,
        20,
        0,
        Math.PI * 2
      );
      ctx.fill();

      ctx.strokeStyle = '#B388FF';
      ctx.lineWidth = 2;
      for (let i = 0; i < 3; i++) {
        const x = Math.sin(time * (i + 1)) * 100 + canvas.width / 2;
        const y = Math.cos(time * (i + 1)) * 100 + canvas.height / 2;
        ctx.strokeRect(x - 15, y - 15, 30, 30);
      }

      ctx.fillStyle = '#ffffff';
      ctx.font = '12px monospace';
      ctx.fillText(label, 10, 20);
      ctx.fillText(new Date().toLocaleTimeString(), 10, canvas.height - 10);

      ctx.fillStyle = '#ff0000';
      ctx.beginPath();
      ctx.arc(canvas.width - 20, 20, 5, 0, Math.PI * 2);
      ctx.fill();

      requestRef.current = requestAnimationFrame(createVideoFrame);
    };

    createVideoFrame();

    return () => {
      if (requestRef.current) {
        cancelAnimationFrame(requestRef.current);
      }
    };
  }, [label]);

  const toggleFullscreen = (e: React.MouseEvent) => {
    e.stopPropagation();

    const container = e.currentTarget.closest('.camera-container');
    if (!container) return;

    if (!isFullscreen) {
      container.classList.add('fullscreen');
    } else {
      container.classList.remove('fullscreen');
    }

    setIsFullscreen(!isFullscreen);
  };

  return (
    <div
      className={`relative overflow-hidden rounded-lg camera-container bg-black ${className}`}
    >
      <canvas
        ref={canvasRef}
        width={width ?? 640}
        height={height ?? 480}
        className="w-full h-full object-cover"
      />

      <div className="absolute inset-0 bg-gradient-to-t from-black/50 to-transparent pointer-events-none">
        <div className="absolute top-2 left-2 flex items-center space-x-2">
          <div className="w-2 h-2 bg-red-500 rounded-full animate-pulse" />
          <span className="text-xs text-white">LIVE</span>
        </div>
      </div>

      <button
        onClick={toggleFullscreen}
        className="absolute top-2 right-2 bg-black/50 text-white p-1 rounded-full hover:bg-primary z-10"
      >
        {isFullscreen ? (
          <Minimize className="w-3 h-3" />
        ) : (
          <Maximize className="w-3 h-3" />
        )}
      </button>
    </div>
  );
}
