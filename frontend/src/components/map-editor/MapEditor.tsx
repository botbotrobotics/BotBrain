'use client';

import React, { useRef, useEffect, useState, useCallback } from 'react';
import { 
  Brush, 
  Square, 
  Circle, 
  Minus, 
  Download, 
  Upload, 
  Eye, 
  EyeOff,
  Eraser,
  Maximize2,
  ZoomIn,
  ZoomOut,
  RotateCcw,
  Save,
  Layers,
  Hexagon,
  Undo,
  Redo
} from 'lucide-react';
import { parsePGM, parseYAML, canvasToPGM, generateYAML, quantizeColor, createMapZip, MapMetadata } from '@/utils/ros/mapFileUtils';
import { setupWebGL } from '@/utils/webgl/mapShaders';
import { cn } from '@/utils/cn';

export type DrawingTool = 'brush' | 'eraser' | 'line' | 'rectangle' | 'circle' | 'polygon';
export type MapRegion = 'occupied' | 'free' | 'unknown';

interface MapEditorProps {
  className?: string;
}

export function MapEditor({ className }: MapEditorProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const previewCanvasRef = useRef<HTMLCanvasElement>(null);
  const toolPreviewCanvasRef = useRef<HTMLCanvasElement>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const yamlInputRef = useRef<HTMLInputElement>(null);
  
  const [isDrawing, setIsDrawing] = useState(false);
  const [currentTool, setCurrentTool] = useState<DrawingTool>('brush');
  const [currentRegion, setCurrentRegion] = useState<MapRegion>('occupied');
  const [brushSize, setBrushSize] = useState(5);
  const [showThreshold, setShowThreshold] = useState(false);
  const [showInflation, setShowInflation] = useState(false);
  const [zoom, setZoom] = useState(1);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [mousePos, setMousePos] = useState<{ x: number; y: number } | null>(null);
  const [polygonPoints, setPolygonPoints] = useState<{ x: number; y: number }[]>([]);
  const [mapMetadata, setMapMetadata] = useState<MapMetadata>({
    image: 'map.pgm',
    resolution: 0.05,
    origin: [-10.0, -10.0, 0.0],
    negate: 0,
    occupied_thresh: 0.65,
    free_thresh: 0.196
  });
  const [robotRadius, setRobotRadius] = useState(0.3); // meters
  
  const [startPoint, setStartPoint] = useState<{ x: number; y: number } | null>(null);
  const [webglContext, setWebglContext] = useState<ReturnType<typeof setupWebGL> | null>(null);
  const [canvasVersion, setCanvasVersion] = useState(0); // Track canvas changes
  
  // Undo/Redo history management
  const [history, setHistory] = useState<ImageData[]>([]);
  const [historyIndex, setHistoryIndex] = useState(-1);
  
  // Get color for region based on thresholds
  const getRegionColor = useCallback((region: MapRegion): string => {
    const occupiedThresh = mapMetadata.occupied_thresh;
    const freeThresh = mapMetadata.free_thresh;
    const unknownValue = Math.round(((occupiedThresh + freeThresh) / 2) * 255);
    
    switch (region) {
      case 'occupied': return '#000000';
      case 'free': return '#FFFFFF';
      case 'unknown': return `rgb(${unknownValue},${unknownValue},${unknownValue})`;
    }
  }, [mapMetadata]);
  
  // Save current canvas state to history AFTER drawing is complete
  const saveToHistory = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d', { willReadFrequently: true });
    if (!ctx) return;
    
    const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
    
    // Remove any states after current index if we're not at the end
    const newHistory = history.slice(0, historyIndex + 1);
    
    // Add new state
    newHistory.push(imageData);
    
    // Keep only last 5 states as per requirement
    if (newHistory.length > 5) {
      newHistory.shift();
    }
    
    setHistory(newHistory);
    setHistoryIndex(newHistory.length - 1);
  }, [history, historyIndex]);
  
  // Perform undo operation
  const performUndo = useCallback(() => {
    if (historyIndex <= 0) return; // No more states to undo to
    
    const canvas = canvasRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d', { willReadFrequently: true });
    if (!ctx) return;
    
    const newIndex = historyIndex - 1;
    const imageData = history[newIndex];
    
    // Restore the previous state
    ctx.putImageData(imageData, 0, 0);
    setHistoryIndex(newIndex);
    
    // Trigger WebGL update
    setCanvasVersion(v => v + 1);
  }, [history, historyIndex]);
  
  // Perform redo operation
  const performRedo = useCallback(() => {
    if (historyIndex >= history.length - 1) return; // No more states to redo to
    
    const canvas = canvasRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d', { willReadFrequently: true });
    if (!ctx) return;
    
    const newIndex = historyIndex + 1;
    const imageData = history[newIndex];
    
    // Restore the next state
    ctx.putImageData(imageData, 0, 0);
    setHistoryIndex(newIndex);
    
    // Trigger WebGL update
    setCanvasVersion(v => v + 1);
  }, [history, historyIndex]);
  
  // Draw tool preview
  const drawToolPreview = useCallback(() => {
    const canvas = toolPreviewCanvasRef.current;
    if (!canvas || !mousePos) return;
    
    const ctx = canvas.getContext('2d', { willReadFrequently: true });
    if (!ctx) return;
    
    // Clear previous preview
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Get the color for the current region
    const region = currentTool === 'eraser' ? 'free' : currentRegion;
    const color = getRegionColor(region);
    
    // Set styles with semi-transparency for preview
    ctx.globalAlpha = 0.5;
    ctx.fillStyle = color;
    ctx.strokeStyle = color;
    ctx.lineWidth = brushSize;
    ctx.lineCap = 'round';
    
    switch (currentTool) {
      case 'brush':
      case 'eraser':
        // Draw circle cursor
        ctx.beginPath();
        ctx.arc(mousePos.x, mousePos.y, brushSize / 2, 0, Math.PI * 2);
        ctx.fill();
        
        // Draw outline for better visibility
        ctx.globalAlpha = 0.8;
        ctx.strokeStyle = region === 'free' ? '#666' : '#ccc';
        ctx.lineWidth = 1;
        ctx.stroke();
        break;
        
      case 'line':
        if (isDrawing && startPoint) {
          // Draw preview line
          ctx.beginPath();
          ctx.moveTo(startPoint.x, startPoint.y);
          ctx.lineTo(mousePos.x, mousePos.y);
          ctx.stroke();
        } else {
          // Show a small crosshair
          ctx.globalAlpha = 0.8;
          ctx.strokeStyle = '#666';
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.moveTo(mousePos.x - 10, mousePos.y);
          ctx.lineTo(mousePos.x + 10, mousePos.y);
          ctx.moveTo(mousePos.x, mousePos.y - 10);
          ctx.lineTo(mousePos.x, mousePos.y + 10);
          ctx.stroke();
        }
        break;
        
      case 'rectangle':
        if (isDrawing && startPoint) {
          // Draw preview rectangle
          ctx.fillRect(
            Math.min(startPoint.x, mousePos.x),
            Math.min(startPoint.y, mousePos.y),
            Math.abs(mousePos.x - startPoint.x),
            Math.abs(mousePos.y - startPoint.y)
          );
          
          // Draw outline
          ctx.globalAlpha = 0.8;
          ctx.strokeStyle = region === 'free' ? '#666' : '#ccc';
          ctx.lineWidth = 1;
          ctx.strokeRect(
            Math.min(startPoint.x, mousePos.x),
            Math.min(startPoint.y, mousePos.y),
            Math.abs(mousePos.x - startPoint.x),
            Math.abs(mousePos.y - startPoint.y)
          );
        } else {
          // Show a small square cursor
          ctx.globalAlpha = 0.8;
          ctx.strokeStyle = '#666';
          ctx.lineWidth = 1;
          ctx.strokeRect(mousePos.x - 5, mousePos.y - 5, 10, 10);
        }
        break;
        
      case 'circle':
        if (isDrawing && startPoint) {
          // Draw preview circle
          const radius = Math.sqrt(
            Math.pow(mousePos.x - startPoint.x, 2) + 
            Math.pow(mousePos.y - startPoint.y, 2)
          );
          ctx.beginPath();
          ctx.arc(startPoint.x, startPoint.y, radius, 0, Math.PI * 2);
          ctx.fill();
          
          // Draw outline
          ctx.globalAlpha = 0.8;
          ctx.strokeStyle = region === 'free' ? '#666' : '#ccc';
          ctx.lineWidth = 1;
          ctx.stroke();
        } else {
          // Show a small circle cursor
          ctx.globalAlpha = 0.8;
          ctx.strokeStyle = '#666';
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.arc(mousePos.x, mousePos.y, 5, 0, Math.PI * 2);
          ctx.stroke();
        }
        break;
        
      case 'polygon':
        // Draw existing polygon points and lines
        if (polygonPoints.length > 0) {
          ctx.globalAlpha = 0.5;
          ctx.beginPath();
          ctx.moveTo(polygonPoints[0].x, polygonPoints[0].y);
          
          // Draw lines between existing points
          for (let i = 1; i < polygonPoints.length; i++) {
            ctx.lineTo(polygonPoints[i].x, polygonPoints[i].y);
          }
          
          // Draw line to current mouse position
          ctx.lineTo(mousePos.x, mousePos.y);
          
          // Check if we're close to the first point (to close the polygon)
          const closeDistance = 10;
          const firstPoint = polygonPoints[0];
          const distance = Math.sqrt(
            Math.pow(mousePos.x - firstPoint.x, 2) + 
            Math.pow(mousePos.y - firstPoint.y, 2)
          );
          
          if (distance < closeDistance && polygonPoints.length > 2) {
            // Show preview of closed polygon
            ctx.closePath();
            ctx.fill();
            ctx.globalAlpha = 0.8;
            ctx.strokeStyle = region === 'free' ? '#666' : '#ccc';
            ctx.lineWidth = 2;
            ctx.stroke();
            
            // Highlight the first point to indicate closing
            ctx.fillStyle = '#4CAF50';
            ctx.beginPath();
            ctx.arc(firstPoint.x, firstPoint.y, 5, 0, Math.PI * 2);
            ctx.fill();
          } else {
            // Just stroke the path
            ctx.stroke();
            
            // Draw points
            ctx.fillStyle = '#666';
            for (const point of polygonPoints) {
              ctx.beginPath();
              ctx.arc(point.x, point.y, 3, 0, Math.PI * 2);
              ctx.fill();
            }
          }
        }
        
        // Show cursor
        ctx.globalAlpha = 0.8;
        ctx.strokeStyle = '#666';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(mousePos.x - 5, mousePos.y);
        ctx.lineTo(mousePos.x + 5, mousePos.y);
        ctx.moveTo(mousePos.x, mousePos.y - 5);
        ctx.lineTo(mousePos.x, mousePos.y + 5);
        ctx.stroke();
        break;
    }
    
    // Reset global alpha
    ctx.globalAlpha = 1;
  }, [currentTool, currentRegion, brushSize, mousePos, isDrawing, startPoint, getRegionColor, polygonPoints]);
  
  // Update tool preview when relevant states change
  useEffect(() => {
    drawToolPreview();
  }, [drawToolPreview]);
  
  // Clear polygon points when switching tools
  useEffect(() => {
    if (currentTool !== 'polygon') {
      setPolygonPoints([]);
    }
  }, [currentTool]);
  
  // Handle keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && currentTool === 'polygon') {
        setPolygonPoints([]);
      }
      
      // Handle Ctrl+Z for undo
      if ((e.ctrlKey || e.metaKey) && e.key === 'z' && !e.shiftKey) {
        e.preventDefault();
        performUndo();
      }
      
      // Handle Ctrl+Y or Ctrl+Shift+Z for redo
      if ((e.ctrlKey || e.metaKey) && (e.key === 'y' || (e.key === 'z' && e.shiftKey))) {
        e.preventDefault();
        performRedo();
      }
    };
    
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [currentTool, performUndo, performRedo]);
  
  // Initialize WebGL context and canvas
  useEffect(() => {
    if (!previewCanvasRef.current) return;
    
    const context = setupWebGL(previewCanvasRef.current);
    if (context) {
      console.log('WebGL context initialized successfully');
      setWebglContext(context);
    } else {
      console.error('Failed to initialize WebGL context');
    }
    
    // Initialize canvas with a default map
    const canvas = canvasRef.current;
    if (canvas) {
      const ctx = canvas.getContext('2d', { willReadFrequently: true });
      if (ctx) {
        ctx.imageSmoothingEnabled = false;
        ctx.fillStyle = '#FFFFFF'; // Free space
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        
        // Add some example obstacles for testing
        ctx.fillStyle = '#000000'; // Occupied
        ctx.fillRect(100, 100, 50, 50);
        ctx.fillRect(300, 200, 80, 30);
        ctx.fillRect(500, 150, 30, 100);
        
        // Add some unknown areas
        const unknownValue = Math.round(((0.65 + 0.196) / 2) * 255);
        ctx.fillStyle = `rgb(${unknownValue},${unknownValue},${unknownValue})`;
        ctx.fillRect(200, 300, 100, 60);
        
        setCanvasVersion(v => v + 1);
        
        // Save initial state to history
        const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
        setHistory([imageData]);
        setHistoryIndex(0);
      }
    }
  }, []);
  
  // Add canvas-based inflation fallback
  const drawInflationCanvas = useCallback(() => {
    if (!canvasRef.current || !previewCanvasRef.current || !showInflation) return;
    
    const sourceCanvas = canvasRef.current;
    const previewCanvas = previewCanvasRef.current;
    
    // Ensure canvases match size
    if (previewCanvas.width !== sourceCanvas.width || previewCanvas.height !== sourceCanvas.height) {
      previewCanvas.width = sourceCanvas.width;
      previewCanvas.height = sourceCanvas.height;
    }
    
    const ctx = previewCanvas.getContext('2d', { willReadFrequently: true });
    if (!ctx) return;
    
    const sourceCtx = sourceCanvas.getContext('2d', { willReadFrequently: true });
    if (!sourceCtx) return;
    
    // Get source image data
    const sourceData = sourceCtx.getImageData(0, 0, sourceCanvas.width, sourceCanvas.height);
    const pixels = sourceData.data;
    
    // Create output image data
    const outputData = ctx.createImageData(sourceCanvas.width, sourceCanvas.height);
    const output = outputData.data;
    
    const inflationPixels = Math.ceil(robotRadius / mapMetadata.resolution);
    const width = sourceCanvas.width;
    const height = sourceCanvas.height;
    
    // Process each pixel
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const idx = (y * width + x) * 4;
        const gray = pixels[idx];
        
        // Check if this is a free space pixel
        if (gray > 230) { // Free space (white)
          // Check if any occupied pixel is within inflation radius
          let inflated = false;
          
          for (let dy = -inflationPixels; dy <= inflationPixels && !inflated; dy++) {
            for (let dx = -inflationPixels; dx <= inflationPixels && !inflated; dx++) {
              const dist = Math.sqrt(dx * dx + dy * dy);
              if (dist <= inflationPixels) {
                const nx = x + dx;
                const ny = y + dy;
                
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                  const nidx = (ny * width + nx) * 4;
                  if (pixels[nidx] < 25) { // Occupied (black)
                    inflated = true;
                  }
                }
              }
            }
          }
          
          if (inflated) {
            // Inflated area - red
            output[idx] = 255;
            output[idx + 1] = 0;
            output[idx + 2] = 0;
            output[idx + 3] = 180;
          } else {
            // Non-inflated free space - transparent
            output[idx] = 0;
            output[idx + 1] = 0;
            output[idx + 2] = 0;
            output[idx + 3] = 0;
          }
        } else if (gray < 25) {
          // Occupied space - blue
          output[idx] = 0;
          output[idx + 1] = 0;
          output[idx + 2] = 200;
          output[idx + 3] = 180;
        } else {
          // Unknown space - yellow
          output[idx] = 255;
          output[idx + 1] = 255;
          output[idx + 2] = 0;
          output[idx + 3] = 100;
        }
      }
    }
    
    // Clear and draw the result
    ctx.clearRect(0, 0, previewCanvas.width, previewCanvas.height);
    ctx.putImageData(outputData, 0, 0);
  }, [showInflation, robotRadius, mapMetadata.resolution]);

  // Update WebGL preview with fallback
  useEffect(() => {
    if (!canvasRef.current || !(showThreshold || showInflation)) return;
    
    // Try WebGL first
    if (webglContext) {
      let animationId: number;
      let isRunning = true;
      
      const renderFrame = () => {
        if (!isRunning) return;
        
        const { gl, uniforms } = webglContext;
        const canvas = canvasRef.current;
        const previewCanvas = previewCanvasRef.current;
        
        if (!canvas || !previewCanvas) return;
        
        // Sync canvas sizes
        if (previewCanvas.width !== canvas.width || previewCanvas.height !== canvas.height) {
          previewCanvas.width = canvas.width;
          previewCanvas.height = canvas.height;
          gl.viewport(0, 0, canvas.width, canvas.height);
        }
        
        try {
          // Create texture from canvas
          const texture = gl.createTexture();
          gl.bindTexture(gl.TEXTURE_2D, texture);
          gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, canvas);
          gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
          gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
          gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
          gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
          
          // Activate texture unit
          gl.activeTexture(gl.TEXTURE0);
          gl.bindTexture(gl.TEXTURE_2D, texture);
          
          // Ensure we're using the correct program
          gl.useProgram(webglContext.program);
          
          // Set uniforms - check for null uniform locations
          if (uniforms.u_image !== null) gl.uniform1i(uniforms.u_image, 0);
          if (uniforms.u_occupiedThresh !== null) gl.uniform1f(uniforms.u_occupiedThresh, mapMetadata.occupied_thresh);
          if (uniforms.u_freeThresh !== null) gl.uniform1f(uniforms.u_freeThresh, mapMetadata.free_thresh);
          if (uniforms.u_showThreshold !== null) gl.uniform1f(uniforms.u_showThreshold, showThreshold ? 1.0 : 0.0);
          if (uniforms.u_showInflation !== null) gl.uniform1f(uniforms.u_showInflation, showInflation ? 1.0 : 0.0);
          if (uniforms.u_inflationRadius !== null) gl.uniform1f(uniforms.u_inflationRadius, robotRadius / mapMetadata.resolution);
          if (uniforms.u_textureSize !== null) gl.uniform2f(uniforms.u_textureSize, canvas.width, canvas.height);
          
          // Debug uniform values when inflation is enabled
          if (showInflation && Math.random() < 0.01) { // Log occasionally
            console.log('WebGL uniforms:', {
              showInflation: showInflation ? 1.0 : 0.0,
              inflationRadius: robotRadius / mapMetadata.resolution,
              textureSize: [canvas.width, canvas.height]
            });
          }
          
          // Enable alpha blending
          gl.enable(gl.BLEND);
          gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
          
          // Render
          gl.clearColor(0, 0, 0, 0);
          gl.clear(gl.COLOR_BUFFER_BIT);
          gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
          
          // Clean up
          gl.deleteTexture(texture);
        } catch (error) {
          console.error('WebGL rendering error:', error);
          // Fall back to canvas rendering
          isRunning = false;
          drawInflationCanvas();
        }
        
        // Continue animation
        if (isRunning) {
          animationId = requestAnimationFrame(renderFrame);
        }
      };
      
      // Start rendering
      renderFrame();
      
      // Cleanup function
      return () => {
        isRunning = false;
        if (animationId) {
          cancelAnimationFrame(animationId);
        }
      };
    } else {
      // No WebGL, use canvas fallback
      console.log('WebGL not available, using canvas fallback for inflation visualization');
      drawInflationCanvas();
    }
  }, [webglContext, mapMetadata, showThreshold, showInflation, robotRadius, canvasVersion, drawInflationCanvas]);
  
  // Handle file upload
  const handleFileUpload = useCallback(async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    
    if (file.name.endsWith('.pgm')) {
      const buffer = await file.arrayBuffer();
      try {
        const { width, height, pixels } = parsePGM(buffer);
        
        // Draw to canvas
        const canvas = canvasRef.current;
        if (!canvas) return;
        
        canvas.width = width;
        canvas.height = height;
        const ctx = canvas.getContext('2d', { willReadFrequently: true })!;
        ctx.imageSmoothingEnabled = false;
        
        const imageData = ctx.createImageData(width, height);
        for (let i = 0; i < pixels.length; i++) {
          const value = pixels[i];
          imageData.data[i * 4] = value;
          imageData.data[i * 4 + 1] = value;
          imageData.data[i * 4 + 2] = value;
          imageData.data[i * 4 + 3] = 255;
        }
        
        ctx.putImageData(imageData, 0, 0);
        
        // Trigger WebGL update
        setCanvasVersion(v => v + 1);
        
        // Save loaded map as initial state in history
        const loadedImageData = ctx.getImageData(0, 0, width, height);
        setHistory([loadedImageData]);
        setHistoryIndex(0);
      } catch (error) {
        console.error('Error parsing PGM file:', error);
        alert('Failed to parse PGM file. Please ensure it\'s a valid P5 format.');
      }
    } else if (file.name.endsWith('.png')) {
      // Handle PNG files
      const canvas = canvasRef.current;
      if (!canvas) return;
      
      const ctx = canvas.getContext('2d', { willReadFrequently: true });
      if (!ctx) return;
      
      const img = new Image();
      const url = URL.createObjectURL(file);
      
      img.onload = () => {
        // Set canvas dimensions to match image
        canvas.width = img.width;
        canvas.height = img.height;
        
        // Draw image to canvas
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(img, 0, 0);
        
        // Convert to grayscale and apply thresholding
        const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
        const data = imageData.data;
        
        // Convert to grayscale using occupancy map conventions
        // ROS maps typically use grayscale values where:
        // - Black (0) = occupied space
        // - White (255) = free space  
        // - Gray (128) = unknown space
        for (let i = 0; i < data.length; i += 4) {
          // Calculate grayscale value (using standard RGB to grayscale conversion)
          const gray = 0.299 * data[i] + 0.587 * data[i + 1] + 0.114 * data[i + 2];
          
          // Normalize to 0-255 range
          const normalizedGray = Math.round(gray);
          
          // Apply to all channels
          data[i] = normalizedGray;
          data[i + 1] = normalizedGray;
          data[i + 2] = normalizedGray;
          // Keep alpha as is
        }
        
        // Put the processed image back
        ctx.putImageData(imageData, 0, 0);
        
        // Trigger WebGL update
        setCanvasVersion(v => v + 1);
        
        // Save loaded map as initial state in history
        const loadedImageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
        setHistory([loadedImageData]);
        setHistoryIndex(0);
        
        // Clean up
        URL.revokeObjectURL(url);
      };
      
      img.onerror = () => {
        console.error('Error loading PNG file');
        alert('Failed to load PNG file. Please ensure it\'s a valid PNG image.');
        URL.revokeObjectURL(url);
      };
      
      img.src = url;
    } else {
      alert('Please select a PGM or PNG file.');
    }
  }, []);
  
  const handleYamlUpload = useCallback(async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    
    const content = await file.text();
    try {
      const metadata = parseYAML(content);
      setMapMetadata(metadata);
    } catch (error) {
      console.error('Error parsing YAML file:', error);
      alert('Failed to parse YAML file.');
    }
  }, []);
  
  // Drawing functions
  const getMousePos = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return { x: 0, y: 0 };
    
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    
    return {
      x: (e.clientX - rect.left) * scaleX,
      y: (e.clientY - rect.top) * scaleY
    };
  }, []);
  
  const drawPixel = useCallback((ctx: CanvasRenderingContext2D, x: number, y: number) => {
    const region = currentTool === 'eraser' ? 'free' : currentRegion;
    const color = getRegionColor(region);
    ctx.fillStyle = color;
    
    if (currentTool === 'brush' || currentTool === 'eraser') {
      ctx.beginPath();
      ctx.arc(x, y, brushSize / 2, 0, Math.PI * 2);
      ctx.fill();
    }
  }, [currentRegion, currentTool, brushSize, getRegionColor]);
  
  const drawLine = useCallback((ctx: CanvasRenderingContext2D, x1: number, y1: number, x2: number, y2: number) => {
    const region = currentTool === 'eraser' ? 'free' : currentRegion;
    const color = getRegionColor(region);
    ctx.strokeStyle = color;
    ctx.lineWidth = brushSize;
    ctx.lineCap = 'round';
    
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  }, [currentRegion, currentTool, brushSize, getRegionColor]);
  
  const drawRectangle = useCallback((ctx: CanvasRenderingContext2D, x1: number, y1: number, x2: number, y2: number) => {
    const color = getRegionColor(currentRegion);
    ctx.fillStyle = color;
    ctx.fillRect(
      Math.min(x1, x2),
      Math.min(y1, y2),
      Math.abs(x2 - x1),
      Math.abs(y2 - y1)
    );
  }, [currentRegion, getRegionColor]);
  
  const drawCircle = useCallback((ctx: CanvasRenderingContext2D, centerX: number, centerY: number, endX: number, endY: number) => {
    const radius = Math.sqrt(Math.pow(endX - centerX, 2) + Math.pow(endY - centerY, 2));
    const color = getRegionColor(currentRegion);
    ctx.fillStyle = color;
    
    ctx.beginPath();
    ctx.arc(centerX, centerY, radius, 0, Math.PI * 2);
    ctx.fill();
  }, [currentRegion, getRegionColor]);
  
  const drawPolygon = useCallback((ctx: CanvasRenderingContext2D, points: { x: number; y: number }[]) => {
    if (points.length < 3) return;
    
    const color = getRegionColor(currentRegion);
    ctx.fillStyle = color;
    
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    
    for (let i = 1; i < points.length; i++) {
      ctx.lineTo(points[i].x, points[i].y);
    }
    
    ctx.closePath();
    ctx.fill();
  }, [currentRegion, getRegionColor]);
  
  // Mouse event handlers
  const handleMouseDown = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    const pos = getMousePos(e);
    
    if (currentTool === 'polygon') {
      // Check if we're closing the polygon
      if (polygonPoints.length > 2) {
        const firstPoint = polygonPoints[0];
        const distance = Math.sqrt(
          Math.pow(pos.x - firstPoint.x, 2) + 
          Math.pow(pos.y - firstPoint.y, 2)
        );
        
        if (distance < 10) {
          // Close and fill the polygon
          const canvas = canvasRef.current;
          if (canvas) {
            const ctx = canvas.getContext('2d', { willReadFrequently: true })!;
            ctx.imageSmoothingEnabled = false;
            drawPolygon(ctx, polygonPoints);
            setCanvasVersion(v => v + 1);
            
            // Save state AFTER drawing the polygon
            saveToHistory();
          }
          
          // Reset polygon points
          setPolygonPoints([]);
          return;
        }
      }
      
      // Add new point to polygon
      setPolygonPoints([...polygonPoints, pos]);
    } else {
      const canvas = canvasRef.current;
      if (!canvas) return;
      
      const ctx = canvas.getContext('2d', { willReadFrequently: true })!;
      ctx.imageSmoothingEnabled = false;
      
      setIsDrawing(true);
      setStartPoint(pos);
      
      if (currentTool === 'brush' || currentTool === 'eraser') {
        drawPixel(ctx, pos.x, pos.y);
        // Trigger WebGL update
        setCanvasVersion(v => v + 1);
      }
    }
  }, [currentTool, getMousePos, drawPixel, polygonPoints, drawPolygon, saveToHistory]);
  
  const handleMouseMove = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    const pos = getMousePos(e);
    setMousePos(pos);
    
    if (!isDrawing) return;
    
    const canvas = canvasRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d', { willReadFrequently: true })!;
    ctx.imageSmoothingEnabled = false;
    
    if (currentTool === 'brush' || currentTool === 'eraser') {
      if (startPoint) {
        drawLine(ctx, startPoint.x, startPoint.y, pos.x, pos.y);
        // Trigger WebGL update
        setCanvasVersion(v => v + 1);
      }
      setStartPoint(pos);
    }
  }, [isDrawing, currentTool, startPoint, getMousePos, drawLine]);
  
  const handleMouseLeave = useCallback(() => {
    // If we were drawing, save the state
    if (isDrawing) {
      saveToHistory();
    }
    setIsDrawing(false);
    setMousePos(null);
  }, [isDrawing, saveToHistory]);
  
  const handleMouseEnter = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    const pos = getMousePos(e);
    setMousePos(pos);
  }, [getMousePos]);
  
  const handleMouseUp = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!isDrawing || !startPoint) return;
    
    const canvas = canvasRef.current;
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d', { willReadFrequently: true })!;
    ctx.imageSmoothingEnabled = false;
    const pos = getMousePos(e);
    
    switch (currentTool) {
      case 'line':
        drawLine(ctx, startPoint.x, startPoint.y, pos.x, pos.y);
        break;
      case 'rectangle':
        drawRectangle(ctx, startPoint.x, startPoint.y, pos.x, pos.y);
        break;
      case 'circle':
        drawCircle(ctx, startPoint.x, startPoint.y, pos.x, pos.y);
        break;
    }
    
    // Trigger WebGL update
    setCanvasVersion(v => v + 1);
    
    // Save state AFTER drawing is complete
    saveToHistory();
    
    setIsDrawing(false);
    setStartPoint(null);
  }, [isDrawing, currentTool, startPoint, getMousePos, drawLine, drawRectangle, drawCircle, saveToHistory]);
  
  // Save functionality
  const handleSave = useCallback(async () => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    
    // Convert canvas to PGM
    const pgmBuffer = canvasToPGM(canvas);
    
    // Generate updated YAML
    const updatedMetadata = {
      ...mapMetadata,
      image: 'map.pgm'
    };
    const yamlContent = generateYAML(updatedMetadata);
    
    // Create zip file
    const zipBlob = await createMapZip(pgmBuffer, yamlContent);
    
    // Download zip file
    const url = URL.createObjectURL(zipBlob);
    const link = document.createElement('a');
    link.href = url;
    link.download = 'ros_map.zip';
    link.click();
    
    // Clean up
    URL.revokeObjectURL(url);
  }, [mapMetadata]);
  
  return (
    <div className={cn("flex flex-col h-full bg-white dark:bg-botbot-darker rounded-lg shadow-lg overflow-hidden", className)}>
      {/* Toolbar */}
      <div className="flex flex-wrap items-center gap-2 p-4 bg-gray-50 dark:bg-botbot-dark border-b border-gray-200 dark:border-botbot-border overflow-x-auto">
        {/* File operations */}
        <div className="flex items-center gap-2 pr-4 border-r border-gray-300 dark:border-botbot-border flex-shrink-0">
          <input
            ref={fileInputRef}
            type="file"
            accept=".pgm,.png"
            onChange={handleFileUpload}
            className="hidden"
          />
          <input
            ref={yamlInputRef}
            type="file"
            accept=".yaml,.yml"
            onChange={handleYamlUpload}
            className="hidden"
          />
          <button
            onClick={() => fileInputRef.current?.click()}
            className="flex items-center gap-2 px-3 py-2 text-sm bg-primary text-white rounded-md hover:bg-primary/90 transition-colors"
          >
            <Upload className="w-4 h-4" />
            Upload Map (PGM/PNG)
          </button>
          <button
            onClick={() => yamlInputRef.current?.click()}
            className="flex items-center gap-2 px-3 py-2 text-sm bg-primary text-white rounded-md hover:bg-primary/90 transition-colors"
          >
            <Upload className="w-4 h-4" />
            Upload YAML
          </button>
          <div className="flex items-center gap-2">
            <button
              onClick={handleSave}
              className="flex items-center gap-2 px-3 py-2 text-sm bg-green-600 text-white rounded-md hover:bg-green-700 transition-colors"
            >
              <Save className="w-4 h-4" />
              Save Map
            </button>
            <button
              onClick={performUndo}
              disabled={historyIndex <= 0}
              className={cn(
                "flex items-center gap-2 px-3 py-2 text-sm rounded-md transition-colors",
                historyIndex > 0 
                  ? "bg-blue-600 text-white hover:bg-blue-700" 
                  : "bg-gray-300 dark:bg-gray-700 text-gray-500 dark:text-gray-400 cursor-not-allowed"
              )}
              title="Undo last action (Ctrl+Z)"
            >
              <Undo className="w-4 h-4" />
              Undo
            </button>
            <button
              onClick={performRedo}
              disabled={historyIndex >= history.length - 1}
              className={cn(
                "flex items-center gap-2 px-3 py-2 text-sm rounded-md transition-colors",
                historyIndex < history.length - 1 
                  ? "bg-blue-600 text-white hover:bg-blue-700" 
                  : "bg-gray-300 dark:bg-gray-700 text-gray-500 dark:text-gray-400 cursor-not-allowed"
              )}
              title="Redo last action (Ctrl+Y)"
            >
              <Redo className="w-4 h-4" />
              Redo
            </button>
          </div>
        </div>
        
        {/* Drawing tools */}
        <div className="flex items-center gap-1">
          <button
            onClick={() => setCurrentTool('brush')}
            className={cn(
              "p-2 rounded-md transition-colors",
              currentTool === 'brush' ? "bg-primary text-white" : "bg-gray-200 dark:bg-botbot-border hover:bg-gray-300 dark:hover:bg-botbot-border/80"
            )}
            title="Brush tool"
          >
            <Brush className="w-4 h-4" />
          </button>
          <button
            onClick={() => setCurrentTool('eraser')}
            className={cn(
              "p-2 rounded-md transition-colors",
              currentTool === 'eraser' ? "bg-primary text-white" : "bg-gray-200 dark:bg-botbot-border hover:bg-gray-300 dark:hover:bg-botbot-border/80"
            )}
            title="Eraser tool"
          >
            <Eraser className="w-4 h-4" />
          </button>
          <button
            onClick={() => setCurrentTool('line')}
            className={cn(
              "p-2 rounded-md transition-colors",
              currentTool === 'line' ? "bg-primary text-white" : "bg-gray-200 dark:bg-botbot-border hover:bg-gray-300 dark:hover:bg-botbot-border/80"
            )}
            title="Line tool"
          >
            <Minus className="w-4 h-4" />
          </button>
          <button
            onClick={() => setCurrentTool('rectangle')}
            className={cn(
              "p-2 rounded-md transition-colors",
              currentTool === 'rectangle' ? "bg-primary text-white" : "bg-gray-200 dark:bg-botbot-border hover:bg-gray-300 dark:hover:bg-botbot-border/80"
            )}
            title="Rectangle tool"
          >
            <Square className="w-4 h-4" />
          </button>
          <button
            onClick={() => setCurrentTool('circle')}
            className={cn(
              "p-2 rounded-md transition-colors",
              currentTool === 'circle' ? "bg-primary text-white" : "bg-gray-200 dark:bg-botbot-border hover:bg-gray-300 dark:hover:bg-botbot-border/80"
            )}
            title="Circle tool"
          >
            <Circle className="w-4 h-4" />
          </button>
          <button
            onClick={() => setCurrentTool('polygon')}
            className={cn(
              "p-2 rounded-md transition-colors",
              currentTool === 'polygon' ? "bg-primary text-white" : "bg-gray-200 dark:bg-botbot-border hover:bg-gray-300 dark:hover:bg-botbot-border/80"
            )}
            title="Polygon tool - Draw custom shapes"
          >
            <Hexagon className="w-4 h-4" />
          </button>
        </div>
        
        {/* Brush size */}
        <div className="flex items-center gap-2 px-4 border-x border-gray-300 dark:border-botbot-border">
          <label className="text-sm font-medium">Size:</label>
          <input
            type="range"
            min="1"
            max="50"
            value={brushSize}
            onChange={(e) => setBrushSize(Number(e.target.value))}
            className="w-24"
          />
          <span className="text-sm text-gray-600 dark:text-gray-400 w-8">{brushSize}</span>
        </div>
        
        {/* Region selector */}
        <div className="flex items-center gap-2">
          <label className="text-sm font-medium">Region:</label>
          <div className="flex gap-1">
            <button
              onClick={() => setCurrentRegion('occupied')}
              className={cn(
                "px-3 py-1 text-sm rounded-md transition-colors",
                currentRegion === 'occupied' ? "bg-black text-white" : "bg-gray-200 dark:bg-botbot-border hover:bg-gray-300 dark:hover:bg-botbot-border/80"
              )}
            >
              Occupied
            </button>
            <button
              onClick={() => setCurrentRegion('free')}
              className={cn(
                "px-3 py-1 text-sm rounded-md transition-colors",
                currentRegion === 'free' ? "bg-white text-black border border-gray-300" : "bg-gray-200 dark:bg-botbot-border hover:bg-gray-300 dark:hover:bg-botbot-border/80"
              )}
            >
              Free
            </button>
            <button
              onClick={() => setCurrentRegion('unknown')}
              className={cn(
                "px-3 py-1 text-sm rounded-md transition-colors",
                currentRegion === 'unknown' ? "bg-gray-500 text-white" : "bg-gray-200 dark:bg-botbot-border hover:bg-gray-300 dark:hover:bg-botbot-border/80"
              )}
            >
              Unknown
            </button>
          </div>
        </div>
        
        {/* View options */}
        <div className="flex items-center gap-2 ml-auto">
          <button
            onClick={() => setShowThreshold(!showThreshold)}
            className={cn(
              "p-2 rounded-md transition-colors",
              showThreshold ? "bg-primary text-white" : "bg-gray-200 dark:bg-botbot-border hover:bg-gray-300 dark:hover:bg-botbot-border/80"
            )}
            title="Toggle threshold visualization"
          >
            {showThreshold ? <Eye className="w-4 h-4" /> : <EyeOff className="w-4 h-4" />}
          </button>
          <button
            onClick={() => {
              const newState = !showInflation;
              setShowInflation(newState);
              if (newState) {
                console.log('Inflation enabled:', {
                  robotRadius,
                  resolution: mapMetadata.resolution,
                  inflationPixels: robotRadius / mapMetadata.resolution,
                  thresholds: { occupied: mapMetadata.occupied_thresh, free: mapMetadata.free_thresh },
                  webglContext: !!webglContext,
                  canvas: !!canvasRef.current,
                  previewCanvas: !!previewCanvasRef.current
                });
              } else {
                console.log('Inflation disabled');
              }
              // Force canvas update
              setCanvasVersion(v => v + 1);
            }}
            className={cn(
              "p-2 rounded-md transition-colors",
              showInflation ? "bg-primary text-white" : "bg-gray-200 dark:bg-botbot-border hover:bg-gray-300 dark:hover:bg-botbot-border/80"
            )}
            title="Toggle inflation visualization"
          >
            <Layers className="w-4 h-4" />
          </button>
        </div>
      </div>
      
      {/* Canvas area */}
      <div className="flex-1 relative overflow-hidden bg-gray-100 dark:bg-botbot-darkest">
        <div className="absolute inset-0 overflow-auto flex items-center justify-center">
          <div className="relative" style={{ transform: `scale(${zoom})`, transformOrigin: 'center' }}>
            <canvas
              ref={canvasRef}
              width={800}
              height={600}
              className="border border-gray-300 dark:border-botbot-border bg-white"
              onMouseDown={handleMouseDown}
              onMouseMove={handleMouseMove}
              onMouseUp={handleMouseUp}
              onMouseLeave={handleMouseLeave}
              onMouseEnter={handleMouseEnter}
              onContextMenu={(e) => {
                e.preventDefault();
                if (currentTool === 'polygon') {
                  setPolygonPoints([]);
                }
              }}
              style={{ imageRendering: 'pixelated', cursor: 'none' }}
            />
            {(showThreshold || showInflation) && (
              <canvas
                ref={previewCanvasRef}
                width={800}
                height={600}
                className="absolute top-0 left-0 pointer-events-none z-10"
                style={{ 
                  imageRendering: 'pixelated',
                  opacity: showInflation ? 0.9 : 0.6,
                  mixBlendMode: 'normal'
                }}
              />
            )}
            {/* Tool preview canvas */}
            <canvas
              ref={toolPreviewCanvasRef}
              width={800}
              height={600}
              className="absolute top-0 left-0 pointer-events-none z-20"
              style={{ 
                imageRendering: 'auto'
              }}
            />
          </div>
        </div>
        
        {/* Zoom controls */}
        <div className="absolute bottom-4 right-4 flex flex-col gap-2">
          <button
            onClick={() => setZoom(Math.min(zoom * 1.2, 5))}
            className="p-2 bg-white dark:bg-botbot-dark rounded-md shadow-md hover:bg-gray-100 dark:hover:bg-botbot-border transition-colors"
          >
            <ZoomIn className="w-4 h-4" />
          </button>
          <button
            onClick={() => setZoom(Math.max(zoom / 1.2, 0.1))}
            className="p-2 bg-white dark:bg-botbot-dark rounded-md shadow-md hover:bg-gray-100 dark:hover:bg-botbot-border transition-colors"
          >
            <ZoomOut className="w-4 h-4" />
          </button>
          <button
            onClick={() => setZoom(1)}
            className="p-2 bg-white dark:bg-botbot-dark rounded-md shadow-md hover:bg-gray-100 dark:hover:bg-botbot-border transition-colors"
          >
            <RotateCcw className="w-4 h-4" />
          </button>
        </div>
      </div>
      
      {/* Status bar */}
      <div className="px-4 py-2 bg-gray-50 dark:bg-botbot-dark border-t border-gray-200 dark:border-botbot-border text-sm text-gray-600 dark:text-gray-400">
        <div className="flex items-center gap-4 flex-wrap">
          <span>Resolution: {mapMetadata.resolution}m/pixel</span>
          <span>Origin: [{mapMetadata.origin.join(', ')}]</span>
          <span>Occupied: &lt;{mapMetadata.occupied_thresh}</span>
          <span>Free: &gt;{mapMetadata.free_thresh}</span>
          {currentTool === 'polygon' && (
            <span className="ml-auto text-primary font-medium">
              {polygonPoints.length === 0 
                ? "Click to start drawing a polygon" 
                : polygonPoints.length < 3 
                ? `Add ${3 - polygonPoints.length} more point${3 - polygonPoints.length > 1 ? 's' : ''} to close` 
                : "Click near the first point to close, or press ESC/right-click to cancel"}
            </span>
          )}
          {showInflation && (
            <div className="flex items-center gap-2 ml-auto">
              <span>Robot radius:</span>
              <input
                type="number"
                value={robotRadius}
                onChange={(e) => setRobotRadius(Number(e.target.value))}
                className="w-20 px-2 py-1 bg-white dark:bg-botbot-darker border border-gray-300 dark:border-botbot-border rounded"
                step="0.1"
                min="0.1"
                max="5.0"
              />
              <span>m</span>
              <span className="text-xs text-gray-500">(Inflation: {Math.round(robotRadius / mapMetadata.resolution)}px)</span>
            </div>
          )}
        </div>
      </div>
    </div>
  );
} 