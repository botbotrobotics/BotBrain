'use client';

import React, { useEffect, useRef, useState } from 'react';
import RosCameraImg from '@/components/ros-camera-img';
import { MoondreamBoundingBox, MoondreamPoint } from '@/hooks/ros/useMoondreamServices';

interface RosRgbdCameraProps {
  topicName: string;
  offlineMsg?: string;
  boundingBoxes?: MoondreamBoundingBox[];
  points?: MoondreamPoint[];
}

export default function RosRgbdCamera({
  topicName,
  offlineMsg = 'Camera stream offline',
  boundingBoxes = [],
  points = []
}: RosRgbdCameraProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const [dimensions, setDimensions] = useState({ width: 0, height: 0 });

  // Update dimensions when container changes
  useEffect(() => {
    const updateDimensions = () => {
      if (containerRef.current) {
        const rect = containerRef.current.getBoundingClientRect();
        setDimensions({ width: rect.width, height: rect.height });
      }
    };

    updateDimensions();
    window.addEventListener('resize', updateDimensions);

    // Use ResizeObserver for more accurate container size tracking
    const resizeObserver = new ResizeObserver(updateDimensions);
    if (containerRef.current) {
      resizeObserver.observe(containerRef.current);
    }

    return () => {
      window.removeEventListener('resize', updateDimensions);
      resizeObserver.disconnect();
    };
  }, []);

  useEffect(() => {
    const drawOverlays = () => {
      const canvas = canvasRef.current;
      if (!canvas) return;

      const ctx = canvas.getContext('2d');
      if (!ctx) return;

      // Set canvas size to match container
      canvas.width = dimensions.width;
      canvas.height = dimensions.height;

      // Clear canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Get the actual image element to determine aspect ratio
      const imgElement = containerRef.current?.querySelector('canvas[id^="camera-canvas"]') as HTMLCanvasElement;
      let drawWidth = canvas.width;
      let drawHeight = canvas.height;
      let offsetX = 0;
      let offsetY = 0;

      // Common camera aspect ratios
      // Most webcams and security cameras use 4:3 or 16:9
      // Based on your image, it looks like a wide angle camera, likely 16:9 or wider
      let imgAspectRatio = 16 / 9; // Default to 16:9 for modern cameras

      if (imgElement && imgElement.width > 0 && imgElement.height > 0) {
        // Get the actual source image dimensions
        const sourceWidth = imgElement.width;
        const sourceHeight = imgElement.height;

        // Only use these dimensions if they seem valid (not stretched to container)
        if (sourceWidth !== canvas.width || sourceHeight !== canvas.height) {
          imgAspectRatio = sourceWidth / sourceHeight;
        }
      }

      // The actual camera feed appears to be wide (landscape) format
      // even when displayed in a portrait container
      // Based on the image, the video occupies approximately 60-70% of the vertical space
      const containerAspectRatio = canvas.width / canvas.height;

      // For portrait containers showing landscape video
      if (containerAspectRatio < 1) {
        // The video appears to use about 60% of the container height
        // This is typical for a 16:9 or wider video in a portrait container
        drawWidth = canvas.width;
        drawHeight = canvas.width / imgAspectRatio;

        // Center the video vertically
        offsetY = (canvas.height - drawHeight) / 2;
        offsetX = 0;

        // If calculated height exceeds container, fit to container width instead
        if (drawHeight > canvas.height * 0.7) {
          // Video shouldn't take more than 70% of vertical space based on observation
          drawHeight = canvas.height * 0.6;
          drawWidth = drawHeight * imgAspectRatio;
          offsetX = (canvas.width - drawWidth) / 2;
          offsetY = (canvas.height - drawHeight) / 2;
        }
      } else {
        // Landscape container - standard calculation
        if (imgAspectRatio > containerAspectRatio) {
          // Image is wider - fit to width
          drawWidth = canvas.width;
          drawHeight = canvas.width / imgAspectRatio;
          offsetY = Math.max(0, (canvas.height - drawHeight) / 2);
        } else {
          // Image is taller - fit to height
          drawHeight = canvas.height;
          drawWidth = canvas.height * imgAspectRatio;
          offsetX = Math.max(0, (canvas.width - drawWidth) / 2);
        }
      }

      // Final safety checks
      drawWidth = Math.min(drawWidth, canvas.width);
      drawHeight = Math.min(drawHeight, canvas.height);
      offsetX = Math.max(0, offsetX);
      offsetY = Math.max(0, offsetY);

      // Create clipping region to ensure overlays stay within video bounds
      ctx.save();
      ctx.beginPath();
      ctx.rect(offsetX, offsetY, drawWidth, drawHeight);
      ctx.clip();

      // Draw bounding boxes
      boundingBoxes.forEach((box) => {
        // Ensure box coordinates are within valid range (0-1)
        const clampedBox = {
          x_min: Math.max(0, Math.min(1, box.x_min)),
          y_min: Math.max(0, Math.min(1, box.y_min)),
          x_max: Math.max(0, Math.min(1, box.x_max)),
          y_max: Math.max(0, Math.min(1, box.y_max))
        };

        // Calculate actual positions based on image dimensions
        const x = offsetX + (clampedBox.x_min * drawWidth);
        const y = offsetY + (clampedBox.y_min * drawHeight);
        const width = (clampedBox.x_max - clampedBox.x_min) * drawWidth;
        const height = (clampedBox.y_max - clampedBox.y_min) * drawHeight;

        // Purple bounding box with gradient effect
        ctx.strokeStyle = '#A855F7'; // Purple color
        ctx.lineWidth = 3;
        ctx.strokeRect(x, y, width, height);

        // Add corner accents for a modern look
        const cornerSize = Math.min(15, width * 0.1, height * 0.1);
        ctx.lineWidth = 4;

        // Top-left corner
        ctx.beginPath();
        ctx.moveTo(x, y + cornerSize);
        ctx.lineTo(x, y);
        ctx.lineTo(x + cornerSize, y);
        ctx.stroke();

        // Top-right corner
        ctx.beginPath();
        ctx.moveTo(x + width - cornerSize, y);
        ctx.lineTo(x + width, y);
        ctx.lineTo(x + width, y + cornerSize);
        ctx.stroke();

        // Bottom-left corner
        ctx.beginPath();
        ctx.moveTo(x, y + height - cornerSize);
        ctx.lineTo(x, y + height);
        ctx.lineTo(x + cornerSize, y + height);
        ctx.stroke();

        // Bottom-right corner
        ctx.beginPath();
        ctx.moveTo(x + width - cornerSize, y + height);
        ctx.lineTo(x + width, y + height);
        ctx.lineTo(x + width, y + height - cornerSize);
        ctx.stroke();

        // Add a semi-transparent fill
        ctx.fillStyle = 'rgba(168, 85, 247, 0.1)'; // Purple with 10% opacity
        ctx.fillRect(x, y, width, height);
      });

      // Draw points
      points.forEach((point) => {
        // Ensure point coordinates are within valid range (0-1)
        const clampedPoint = {
          x: Math.max(0, Math.min(1, point.x)),
          y: Math.max(0, Math.min(1, point.y))
        };

        // Calculate actual positions based on image dimensions
        const x = offsetX + (clampedPoint.x * drawWidth);
        const y = offsetY + (clampedPoint.y * drawHeight);

        // Draw crosshair
        ctx.strokeStyle = '#FF6B6B'; // Red color for points
        ctx.lineWidth = 2;

        // Horizontal line
        ctx.beginPath();
        ctx.moveTo(x - 20, y);
        ctx.lineTo(x + 20, y);
        ctx.stroke();

        // Vertical line
        ctx.beginPath();
        ctx.moveTo(x, y - 20);
        ctx.lineTo(x, y + 20);
        ctx.stroke();

        // Draw center dot
        ctx.fillStyle = '#FF6B6B';
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, 2 * Math.PI);
        ctx.fill();

        // Add pulsing effect with outer ring
        ctx.strokeStyle = 'rgba(255, 107, 107, 0.4)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(x, y, 15, 0, 2 * Math.PI);
        ctx.stroke();
      });

      // Restore context to remove clipping
      ctx.restore();
    };

    // Use requestAnimationFrame for smooth updates
    const animationFrame = requestAnimationFrame(drawOverlays);

    return () => {
      cancelAnimationFrame(animationFrame);
    };
  }, [boundingBoxes, points, dimensions]);

  return (
    <div ref={containerRef} className="relative w-full h-full">
      <RosCameraImg
        cameraType="moondream"
        topicName={topicName}
        offlineMsg={offlineMsg}
      />
      <canvas
        ref={canvasRef}
        className="absolute top-0 left-0 w-full h-full pointer-events-none"
        style={{ zIndex: 10 }}
      />
    </div>
  );
}