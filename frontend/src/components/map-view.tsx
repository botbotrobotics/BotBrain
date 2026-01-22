'use client';

import React, { useEffect, useRef, useState } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { MapPin, Map as MapIcon } from 'lucide-react';
import { useLanguage } from '@/contexts/LanguageContext';
import * as ROSLIB from 'roslib';

// Import ROS2D types
declare global {
  interface Window {
    ROS2D: any;
    createjs: any;
    ROSLIB: any;
    ROS2D_MAP_DATA?: {
      width: number;
      height: number;
      resolution: number;
      originX: number;
      originY: number;
    };
  }
}

interface MapViewProps {
  className?: string;
}

export default function MapView({ className = '' }: MapViewProps) {
  const { connection } = useRobotConnection();
  const { t } = useLanguage();
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const viewerRef = useRef<any>(null);
  const gridClientRef = useRef<any>(null);
  const robotMarkerRef = useRef<any>(null);
  const [mapError, setMapError] = useState<string | null>(null);
  const [isMapLoaded, setIsMapLoaded] = useState(false);
  const [showCostmap, setShowCostmap] = useState(false);
  const tryFallbackMapRenderingRef = useRef<(() => void) | null>(null);
  const fallbackCleanupRef = useRef<(() => void) | null>(null);

  useEffect(() => {
    if (!connection.ros || !connection.online || !mapContainerRef.current) {
      return;
    }

    setMapError(null);
    setIsMapLoaded(false);

    // List available topics to help debug
    const listTopics = () => {
      if (connection.ros && connection.online) {
        const topicsClient = new ROSLIB.Service({
          ros: connection.ros,
          name: '/rosapi/topics',
          serviceType: 'rosapi/Topics'
        });

        topicsClient.callService({}, (result: any) => {
          console.log('Available ROS topics:', result.topics);
          const mapRelatedTopics = result.topics.filter((topic: string) => 
            topic.includes('map') || topic.includes('costmap') || topic.includes('grid')
          );
          console.log('Map-related topics:', mapRelatedTopics);
          
          // Check for common map topics
          const commonMapTopics = ['/map', '/map_metadata', '/move_base/global_costmap/costmap'];
          const availableMapTopics = commonMapTopics.filter(topic => result.topics.includes(topic));
          if (availableMapTopics.length === 0) {
            console.warn('No standard map topics found. You may need to run a map server or navigation stack.');
            setMapError('No map topics available. Please ensure map_server or navigation is running.');
          } else {
            console.log('Found map topics:', availableMapTopics);
          }
        }, (error: any) => {
          console.warn('Could not list topics:', error);
        });
      }
    };

    listTopics();

    // Dynamically load the required scripts
    const loadScripts = async () => {
      try {
        // Make ROSLIB available globally for ros2d
        if (!window.ROSLIB) {
          window.ROSLIB = ROSLIB;
        }

        // Check if scripts are already loaded
        if (window.ROS2D && window.createjs) {
          initializeMap();
          return;
        }

        // Load EaselJS first (CreateJS)
        const easelScript = document.createElement('script');
        easelScript.src = 'https://cdn.jsdelivr.net/npm/easeljs@1.0.2/lib/easeljs.min.js';
        easelScript.async = true;
        
        await new Promise((resolve, reject) => {
          easelScript.onload = resolve;
          easelScript.onerror = reject;
          document.head.appendChild(easelScript);
        });

        // Load EventEmitter2
        const eventEmitterScript = document.createElement('script');
        eventEmitterScript.src = 'https://cdn.jsdelivr.net/npm/eventemitter2@6.4.9/lib/eventemitter2.min.js';
        eventEmitterScript.async = true;
        
        await new Promise((resolve, reject) => {
          eventEmitterScript.onload = () => {
            console.log('EventEmitter2 loaded successfully');
            resolve(true);
          };
          eventEmitterScript.onerror = reject;
          document.head.appendChild(eventEmitterScript);
        });

        // Load ROS2D
        const ros2dScript = document.createElement('script');
        ros2dScript.src = '/ros2d.min.js'; // We'll need to copy this to public folder
        ros2dScript.async = true;
        
        await new Promise((resolve, reject) => {
          ros2dScript.onload = () => {
            console.log('ROS2D loaded successfully');
            resolve(true);
          };
          ros2dScript.onerror = (error) => {
            console.error('Failed to load ROS2D:', error);
            reject(error);
          };
          document.head.appendChild(ros2dScript);
        });

        // Give a small delay to ensure all scripts are fully initialized
        await new Promise(resolve => setTimeout(resolve, 100));

        // Initialize map after scripts are loaded
        initializeMap();
      } catch (error) {
        console.error('Error loading map scripts:', error);
        setMapError('Failed to load map libraries');
      }
    };

    const initializeMap = async () => {
      try {
        console.log('Initializing map viewer...');
        console.log('ROS2D available:', !!window.ROS2D);
        console.log('createjs available:', !!window.createjs);
        
        // Clear previous viewer if exists
        if (viewerRef.current) {
          viewerRef.current.scene.removeAllChildren();
        }

        if (!window.ROS2D) {
          throw new Error('ROS2D library not loaded');
        }

        // Create the main viewer
        const viewer = new window.ROS2D.Viewer({
          divID: 'ros2d-map',
          width: mapContainerRef.current!.clientWidth,
          height: mapContainerRef.current!.clientHeight,
          background: '#1a1a1a'
        });
        viewerRef.current = viewer;

        // Setup the map client - use either costmap or regular map based on toggle
        const mapTopic = showCostmap ? '/move_base/global_costmap/costmap' : '/map';
        console.log(`Subscribing to map topic: ${mapTopic}`);
        
        // Create the occupancy grid client with error handling
        let gridClient;
        try {
          // First, let's verify the map topic exists and has data
          const mapTestListener = new ROSLIB.Topic({
            ros: connection.ros!,
            name: mapTopic,
            messageType: 'nav_msgs/OccupancyGrid'
          });

          let mapDataReceived = false;
          mapTestListener.subscribe((msg: any) => {
            if (!mapDataReceived) {
              mapDataReceived = true;
              console.log('Map test successful - data structure:', {
                hasHeader: !!msg.header,
                hasInfo: !!msg.info,
                hasData: !!msg.data,
                dataLength: msg.data?.length,
                width: msg.info?.width,
                height: msg.info?.height,
                resolution: msg.info?.resolution,
                origin: msg.info?.origin
              });
              mapTestListener.unsubscribe();
            }
          });

          // Small delay to ensure ROS connection is stable
          await new Promise(resolve => setTimeout(resolve, 500));

          gridClient = new window.ROS2D.OccupancyGridClient({
            ros: connection.ros!,
            topic: mapTopic,
            rootObject: viewer.scene,
            continuous: true
          });
          gridClientRef.current = gridClient;
          console.log('OccupancyGridClient created successfully for topic:', mapTopic);
        } catch (error) {
          console.error('Error creating OccupancyGridClient:', error);
          setMapError('Failed to create map client');
          return;
        }

        // Add a timeout to show error if no map is received
        const mapTimeout = setTimeout(() => {
          if (!isMapLoaded) {
            console.error('ROS2D timeout - trying fallback renderer');
            // Instead of showing error, try fallback
            if (tryFallbackMapRenderingRef.current) {
              tryFallbackMapRenderingRef.current();
            }
          }
        }, 10000); // 10 second timeout before fallback

        // Scale the canvas to fit to the map when it changes
        let changeEventFired = false;
        gridClient.on('change', () => {
          changeEventFired = true;
          console.log('ROS2D change event fired!');
          clearTimeout(mapTimeout);
          
          if (!gridClient.currentGrid) {
            console.warn('No currentGrid available in change event');
            return;
          }
          
          console.log('Map dimensions from ROS2D:', {
            width: gridClient.currentGrid.width,
            height: gridClient.currentGrid.height,
            resolution: gridClient.currentGrid.scaleX,
            origin: gridClient.currentGrid.pose?.position,
            x: gridClient.currentGrid.x,
            y: gridClient.currentGrid.y
          });
          
          // Important: Store the actual map dimensions and origin for coordinate calculations
          const mapData = {
            width: gridClient.currentGrid.width,
            height: gridClient.currentGrid.height,
            resolution: gridClient.currentGrid.scaleX,
            originX: gridClient.currentGrid.pose.position.x,
            originY: gridClient.currentGrid.pose.position.y
          };
          
          // Store in a ref so we can access in the pose callback
          window.ROS2D_MAP_DATA = mapData;
          
          // Calculate scale to fit map in viewer
          const mapWidth = gridClient.currentGrid.width;
          const mapHeight = gridClient.currentGrid.height;
          const viewerWidth = viewer.width;
          const viewerHeight = viewer.height;
          
          const scaleX = viewerWidth / mapWidth;
          const scaleY = viewerHeight / mapHeight;
          const scale = Math.min(scaleX, scaleY) * 0.9; // 90% to leave some margin
          
          viewer.scaleToDimensions(mapWidth * scale, mapHeight * scale);
          viewer.shift(mapWidth * scale / 2, mapHeight * scale / 2);
          
          setIsMapLoaded(true);
          setMapError(null);
        });

        // Also check if currentGrid exists immediately and periodically
        const checkCurrentGrid = setInterval(() => {
          if (gridClient.currentGrid) {
            console.log('CurrentGrid found!', gridClient.currentGrid);
            if (!changeEventFired) {
              console.log('Manually triggering change handler');
              // Manually call the change handler
              const event = new Event('change');
              gridClient.emit('change', event);
              clearInterval(checkCurrentGrid);
            }
          } else {
            console.log('No currentGrid yet...');
          }
        }, 500);
        
        // Stop checking after 10 seconds
        setTimeout(() => clearInterval(checkCurrentGrid), 10000);

        // Test direct subscription to map topic first
        const testMapSubscription = () => {
          const mapListener = new ROSLIB.Topic({
            ros: connection.ros!,
            name: mapTopic,
            messageType: 'nav_msgs/OccupancyGrid'
          });

          console.log(`Testing direct subscription to ${mapTopic}`);
          
          mapListener.subscribe((message: any) => {
            console.log('Raw map message received!', {
              hasInfo: !!message.info,
              hasData: !!message.data,
              dataLength: message.data?.length,
              width: message.info?.width,
              height: message.info?.height,
              resolution: message.info?.resolution
            });
            
            // If we get data via direct subscription but not via ROS2D,
            // it might be a ROS2D compatibility issue
            if (message.info && message.data) {
              console.warn('Map data received via direct subscription but ROS2D might not be updating');
              // Try the fallback renderer if we're getting data but ROS2D isn't working
              setTimeout(() => {
                if (!gridClientRef.current?.currentGrid) {
                  console.log('ROS2D not working, trying fallback');
                  if (tryFallbackMapRenderingRef.current) {
                    tryFallbackMapRenderingRef.current();
                  }
                }
              }, 2000);
            }
          });

          // Clean up after 10 seconds
          setTimeout(() => {
            mapListener.unsubscribe();
          }, 10000);
        };

        testMapSubscription();

        // Add robot position marker - use purple to match native view
        const robotMarker = new window.ROS2D.NavigationArrow({
          size: 15,
          strokeSize: 2,
          fillColor: window.createjs.Graphics.getRGB(139, 92, 246, 0.8), // Purple #8b5cf6
          strokeColor: window.createjs.Graphics.getRGB(255, 255, 255, 1),
          pulse: true
        });
        robotMarkerRef.current = robotMarker;
        
        // Add to viewer scene for now
        viewer.scene.addChild(robotMarker);

        // Store map metadata
        let mapMetadata: any = null;
        
        // Subscribe to map metadata to get origin information
        const mapMetadataListener = new ROSLIB.Topic({
          ros: connection.ros!,
          name: '/map_metadata',
          messageType: 'nav_msgs/MapMetaData'
        });
        
        mapMetadataListener.subscribe((metadata: any) => {
          mapMetadata = metadata;
          console.log('Map metadata received:', {
            resolution: metadata.resolution,
            width: metadata.width,
            height: metadata.height,
            origin: metadata.origin
          });
        });

        // Subscribe to robot pose - use /titan/map_odom for map-relative position
        const poseListener = new ROSLIB.Topic({
          ros: connection.ros!,
          name: '/titan/map_odom',
          messageType: 'geometry_msgs/PoseStamped'
        });
        console.log('Subscribing to /titan/map_odom for robot position');

        if (poseListener) {
          poseListener.subscribe((message: any) => {
            if (!robotMarker || !gridClient.currentGrid) return;
            
            // PoseStamped has pose directly at message.pose
            const pose = message.pose;
            if (!pose) return;
            
            // Get map data from stored reference
            const mapData = window.ROS2D_MAP_DATA || {
              width: gridClient.currentGrid.width,
              height: gridClient.currentGrid.height,
              resolution: gridClient.currentGrid.scaleX,
              originX: gridClient.currentGrid.pose.position.x,
              originY: gridClient.currentGrid.pose.position.y
            };
            
            // Debug logging
            console.log('Robot position update:', {
              frameId: message.header?.frame_id,
              robotPos: { x: pose.position.x, y: pose.position.y },
              mapOrigin: { x: mapData.originX, y: mapData.originY },
              mapResolution: mapData.resolution,
              mapDimensions: { width: mapData.width, height: mapData.height }
            });
            
            // Convert to map pixel coordinates (matching native view logic)
            const mapX = (pose.position.x - mapData.originX) / mapData.resolution;
            const mapY = (pose.position.y - mapData.originY) / mapData.resolution;
            
            // ROS2D positions objects relative to the map's coordinate system
            // The map image is positioned with its bottom-left at the origin
            // So we just use the map coordinates directly
            const ros2dX = mapX;
            const ros2dY = mapData.height - mapY; // Flip Y coordinate
            
            console.log('Robot coordinates:', { 
              mapPixels: { x: mapX, y: mapY },
              ros2d: { x: ros2dX, y: ros2dY }
            });
            
            // Set robot position
            robotMarker.x = ros2dX;
            robotMarker.y = ros2dY;
            
            // Calculate rotation from quaternion
            const q = pose.orientation;
            const rotation = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
            robotMarker.rotation = -rotation * 180 / Math.PI; // Convert to degrees and invert
            
            // Make sure the robot marker is visible
            robotMarker.visible = true;
          });
        }

        // Handle resize
        const handleResize = () => {
          if (!mapContainerRef.current || !viewer) return;
          
          viewer.width = mapContainerRef.current.clientWidth;
          viewer.height = mapContainerRef.current.clientHeight;
          viewer.resize(viewer.width, viewer.height);
        };

        window.addEventListener('resize', handleResize);

        // Cleanup
        return () => {
          window.removeEventListener('resize', handleResize);
          if (poseListener) {
            poseListener.unsubscribe();
          }
          if (mapMetadataListener) {
            mapMetadataListener.unsubscribe();
          }
          if (gridClient) {
            gridClient.rootObject.removeAllChildren();
          }
          if (viewer) {
            viewer.scene.removeAllChildren();
          }
        };
      } catch (error) {
        console.error('Error initializing map:', error);
        setMapError(error instanceof Error ? error.message : 'Failed to initialize map viewer');
        
        // Try a fallback approach with manual canvas rendering
        if (tryFallbackMapRenderingRef.current) {
          tryFallbackMapRenderingRef.current();
        }
      }
    };

    // Fallback map rendering if ROS2D fails
    const tryFallbackMapRendering = () => {
      console.log('Trying fallback map rendering...');
      
      // Clean up any existing fallback
      if (fallbackCleanupRef.current) {
        fallbackCleanupRef.current();
        fallbackCleanupRef.current = null;
      }
      
      const mapListener = new ROSLIB.Topic({
        ros: connection.ros!,
        name: showCostmap ? '/move_base/global_costmap/costmap' : '/map',
        messageType: 'nav_msgs/OccupancyGrid'
      });

      let mapInfo: any = null;
      let mapCanvas: HTMLCanvasElement | null = null;
      let robotMarker: HTMLDivElement | null = null;
      let handleMouseMove: ((e: MouseEvent) => void) | null = null;
      let handleMouseUp: (() => void) | null = null;
      
      // Use a state object to ensure closures work properly
      const state = {
        scale: 1,
        translateX: 0,
        translateY: 0,
        isDragging: false,
        dragStartX: 0,
        dragStartY: 0,
        viewport: null as HTMLDivElement | null
      };

      mapListener.subscribe((message: any) => {
        if (message.info && message.data) {
          console.log('Fallback: Map data received, attempting manual render', {
            width: message.info.width,
            height: message.info.height,
            resolution: message.info.resolution,
            origin: message.info.origin
          });
          
          mapInfo = message.info;
          
          const container = document.getElementById('ros2d-map');
          if (!container) return;
          
          // Clear container
          container.innerHTML = '';
          
          // Create main wrapper with overflow hidden
          const mainWrapper = document.createElement('div');
          mainWrapper.style.position = 'relative';
          mainWrapper.style.width = '100%';
          mainWrapper.style.height = '100%';
          mainWrapper.style.overflow = 'hidden';
          mainWrapper.style.backgroundColor = '#1a1a1a';
          mainWrapper.style.cursor = 'grab';
          mainWrapper.style.userSelect = 'none';
          
          // Create viewport for centering
          const viewport = document.createElement('div');
          viewport.style.position = 'absolute';
          viewport.style.top = '50%';
          viewport.style.left = '50%';
          viewport.style.width = '0';
          viewport.style.height = '0';
          
          // Create transformable wrapper for map and overlays
          const wrapper = document.createElement('div');
          wrapper.style.position = 'absolute';
          wrapper.style.width = `${message.info.width}px`;
          wrapper.style.height = `${message.info.height}px`;
          wrapper.style.left = `-${message.info.width / 2}px`;
          wrapper.style.top = `-${message.info.height / 2}px`;
          
          const canvas = document.createElement('canvas');
          canvas.width = message.info.width;
          canvas.height = message.info.height;
          canvas.style.width = '100%';
          canvas.style.height = '100%';
          
          // Calculate initial scale to fit the container
          const containerRect = container.getBoundingClientRect();
          const scaleX = (containerRect.width * 0.9) / message.info.width;
          const scaleY = (containerRect.height * 0.9) / message.info.height;
          state.scale = Math.min(scaleX, scaleY, 2); // Max initial scale of 2
          
          canvas.style.display = 'block';
          canvas.style.imageRendering = 'pixelated'; // Crisp pixels when zoomed
          
          mapCanvas = canvas;
          
          const ctx = canvas.getContext('2d');
          if (!ctx) return;
          
          // Create image data
          const imageData = ctx.createImageData(message.info.width, message.info.height);
          const data = imageData.data;
          
          // Convert occupancy grid to image - flip Y axis
          for (let y = 0; y < message.info.height; y++) {
            for (let x = 0; x < message.info.width; x++) {
              const srcIdx = y * message.info.width + x;
              const dstIdx = ((message.info.height - 1 - y) * message.info.width + x) * 4;
              
              const value = message.data[srcIdx];
              let color;
              
              if (value === -1) {
                // Unknown - dark gray
                color = 100;
              } else if (value > 50) {
                // Occupied - black
                color = 0;
              } else {
                // Free - white
                color = 255;
              }
              
              data[dstIdx] = color;     // R
              data[dstIdx + 1] = color; // G
              data[dstIdx + 2] = color; // B
              data[dstIdx + 3] = 255;   // A
            }
          }
          
          ctx.putImageData(imageData, 0, 0);
          
          // Create robot marker
          robotMarker = document.createElement('div');
          robotMarker.style.position = 'absolute';
          robotMarker.style.width = '20px';
          robotMarker.style.height = '20px';
          robotMarker.style.backgroundColor = 'orange';
          robotMarker.style.borderRadius = '50%';
          robotMarker.style.border = '2px solid white';
          robotMarker.style.zIndex = '10';
          robotMarker.style.display = 'none'; // Hidden until we get position
          robotMarker.style.pointerEvents = 'none'; // Don't interfere with dragging
          
          // Add arrow indicator
          const arrow = document.createElement('div');
          arrow.style.position = 'absolute';
          arrow.style.width = '0';
          arrow.style.height = '0';
          arrow.style.borderLeft = '5px solid transparent';
          arrow.style.borderRight = '5px solid transparent';
          arrow.style.borderBottom = '10px solid white';
          arrow.style.top = '-8px';
          arrow.style.left = '50%';
          arrow.style.transform = 'translateX(-50%)';
          robotMarker.appendChild(arrow);
          
          wrapper.appendChild(canvas);
          wrapper.appendChild(robotMarker);
          viewport.appendChild(wrapper);
          
          // Store viewport reference
          state.viewport = viewport;
          
          // Function to update transform
          const updateTransform = () => {
            if (!state.viewport) return;
            const transform = `translate(${state.translateX}px, ${state.translateY}px) scale(${state.scale})`;
            state.viewport.style.transform = transform;
          };
          
          // Initial transform
          updateTransform();
          
          // Add zoom controls
          const zoomControls = document.createElement('div');
          zoomControls.style.position = 'absolute';
          zoomControls.style.top = '10px';
          zoomControls.style.right = '10px';
          zoomControls.style.display = 'flex';
          zoomControls.style.flexDirection = 'column';
          zoomControls.style.gap = '5px';
          zoomControls.style.zIndex = '20';
          
          const zoomInBtn = document.createElement('button');
          zoomInBtn.textContent = '+';
          zoomInBtn.style.width = '30px';
          zoomInBtn.style.height = '30px';
          zoomInBtn.style.fontSize = '20px';
          zoomInBtn.style.border = '1px solid white';
          zoomInBtn.style.backgroundColor = 'rgba(0,0,0,0.5)';
          zoomInBtn.style.color = 'white';
          zoomInBtn.style.cursor = 'pointer';
          zoomInBtn.style.borderRadius = '4px';
          
          const zoomOutBtn = document.createElement('button');
          zoomOutBtn.textContent = '−';
          zoomOutBtn.style.width = '30px';
          zoomOutBtn.style.height = '30px';
          zoomOutBtn.style.fontSize = '20px';
          zoomOutBtn.style.border = '1px solid white';
          zoomOutBtn.style.backgroundColor = 'rgba(0,0,0,0.5)';
          zoomOutBtn.style.color = 'white';
          zoomOutBtn.style.cursor = 'pointer';
          zoomOutBtn.style.borderRadius = '4px';
          
          const resetBtn = document.createElement('button');
          resetBtn.textContent = '⟲';
          resetBtn.style.width = '30px';
          resetBtn.style.height = '30px';
          resetBtn.style.fontSize = '16px';
          resetBtn.style.border = '1px solid white';
          resetBtn.style.backgroundColor = 'rgba(0,0,0,0.5)';
          resetBtn.style.color = 'white';
          resetBtn.style.cursor = 'pointer';
          resetBtn.style.borderRadius = '4px';
          resetBtn.title = 'Reset view';
          
          zoomInBtn.onclick = (e) => {
            e.preventDefault();
            e.stopPropagation();
            state.scale = Math.min(state.scale * 1.2, 10);
            updateTransform();
          };
          
          zoomOutBtn.onclick = (e) => {
            e.preventDefault();
            e.stopPropagation();
            state.scale = Math.max(state.scale / 1.2, 0.1);
            updateTransform();
          };
          
          resetBtn.onclick = () => {
            const containerRect = container.getBoundingClientRect();
            const scaleX = (containerRect.width * 0.9) / message.info.width;
            const scaleY = (containerRect.height * 0.9) / message.info.height;
            state.scale = Math.min(scaleX, scaleY, 2);
            state.translateX = 0;
            state.translateY = 0;
            updateTransform();
          };
          
          zoomControls.appendChild(zoomInBtn);
          zoomControls.appendChild(zoomOutBtn);
          zoomControls.appendChild(resetBtn);
          
          // Mouse wheel zoom
          mainWrapper.addEventListener('wheel', (e) => {
            e.preventDefault();
            const delta = e.deltaY > 0 ? 0.9 : 1.1;
            state.scale = Math.max(0.1, Math.min(10, state.scale * delta));
            updateTransform();
          }, { passive: false });
          
          // Pan functionality
          mainWrapper.addEventListener('mousedown', (e) => {
            e.preventDefault();
            state.isDragging = true;
            state.dragStartX = e.clientX - state.translateX;
            state.dragStartY = e.clientY - state.translateY;
            mainWrapper.style.cursor = 'grabbing';
          });
          
          handleMouseMove = (e: MouseEvent) => {
            if (!state.isDragging) return;
            e.preventDefault();
            state.translateX = e.clientX - state.dragStartX;
            state.translateY = e.clientY - state.dragStartY;
            updateTransform();
          };
          
          handleMouseUp = () => {
            state.isDragging = false;
            mainWrapper.style.cursor = 'grab';
          };
          
          window.addEventListener('mousemove', handleMouseMove);
          window.addEventListener('mouseup', handleMouseUp);
          
          // Touch support for mobile
          let touchStartDistance = 0;
          let touchStartScale = state.scale;
          
          mainWrapper.addEventListener('touchstart', (e) => {
            if (e.touches.length === 2) {
              const dx = e.touches[0].clientX - e.touches[1].clientX;
              const dy = e.touches[0].clientY - e.touches[1].clientY;
              touchStartDistance = Math.sqrt(dx * dx + dy * dy);
              touchStartScale = state.scale;
            } else if (e.touches.length === 1) {
              state.isDragging = true;
              state.dragStartX = e.touches[0].clientX - state.translateX;
              state.dragStartY = e.touches[0].clientY - state.translateY;
            }
          });
          
          mainWrapper.addEventListener('touchmove', (e) => {
            e.preventDefault();
            if (e.touches.length === 2) {
              const dx = e.touches[0].clientX - e.touches[1].clientX;
              const dy = e.touches[0].clientY - e.touches[1].clientY;
              const distance = Math.sqrt(dx * dx + dy * dy);
              state.scale = Math.max(0.1, Math.min(10, touchStartScale * (distance / touchStartDistance)));
              updateTransform();
            } else if (e.touches.length === 1 && state.isDragging) {
              state.translateX = e.touches[0].clientX - state.dragStartX;
              state.translateY = e.touches[0].clientY - state.dragStartY;
              updateTransform();
            }
          });
          
          mainWrapper.addEventListener('touchend', () => {
            state.isDragging = false;
          });
          
          mainWrapper.appendChild(viewport);
          mainWrapper.appendChild(zoomControls);
          container.appendChild(mainWrapper);
          setIsMapLoaded(true);
          setMapError(null);
          console.log('Fallback map rendering successful');
          
          mapListener.unsubscribe();
        }
      });

            // Subscribe to robot position
      const poseListener = new ROSLIB.Topic({
        ros: connection.ros!,
        name: '/titan/map_odom',
        messageType: 'geometry_msgs/PoseStamped'
      });

      poseListener.subscribe((message: any) => {
        if (!mapInfo || !mapCanvas || !robotMarker) return;
        
        // PoseStamped has pose directly at message.pose
        const pose = message.pose;
        if (!pose) return;
        
        console.log('Fallback robot position update:', {
          frameId: message.header?.frame_id,
          robotPos: { x: pose.position.x, y: pose.position.y },
          mapOrigin: mapInfo.origin?.position,
          mapResolution: mapInfo.resolution,
          mapSize: { width: mapInfo.width, height: mapInfo.height }
        });
        
        // Convert robot position to map coordinates
        // This matches the transformation used in map-view-native.tsx
        const mapX = (pose.position.x - mapInfo.origin.position.x) / mapInfo.resolution;
        const mapY = (pose.position.y - mapInfo.origin.position.y) / mapInfo.resolution;
        
        console.log('Fallback robot map coordinates:', { mapX, mapY });
        
        // Convert to canvas coordinates (with Y flip)
        const canvasX = mapX;
        const canvasY = mapInfo.height - mapY;
        
        // Position robot marker in pixels relative to canvas
        robotMarker.style.left = `${canvasX}px`;
        robotMarker.style.top = `${canvasY}px`;
        robotMarker.style.display = 'block';
        
        // Calculate rotation from quaternion
        const q = pose.orientation;
        const rotation = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        const degrees = rotation * 180 / Math.PI;
        
        robotMarker.style.transform = `translate(-50%, -50%) rotate(${-degrees}deg)`;
      });
      
      // Store cleanup function
      const cleanup = () => {
        console.log('Cleaning up fallback map renderer');
        mapListener.unsubscribe();
        poseListener.unsubscribe();
        if (window && handleMouseMove && handleMouseUp) {
          window.removeEventListener('mousemove', handleMouseMove);
          window.removeEventListener('mouseup', handleMouseUp);
        }
      };
      
      fallbackCleanupRef.current = cleanup;
    };
      
      // Store the fallback function in ref so it can be accessed from JSX
      tryFallbackMapRenderingRef.current = tryFallbackMapRendering;

      loadScripts();

    return () => {
      // Cleanup
      console.log('Cleaning up map view...');
      
      // Clean up fallback renderer if active
      if (fallbackCleanupRef.current) {
        fallbackCleanupRef.current();
        fallbackCleanupRef.current = null;
      }
      
      if (gridClientRef.current) {
        try {
          gridClientRef.current.rootObject.removeAllChildren();
        } catch (e) {
          console.warn('Error cleaning up grid client:', e);
        }
      }
      if (viewerRef.current) {
        try {
          viewerRef.current.scene.removeAllChildren();
        } catch (e) {
          console.warn('Error cleaning up viewer:', e);
        }
      }
      // Clear the container
      const container = document.getElementById('ros2d-map');
      if (container) {
        container.innerHTML = '';
      }
      setIsMapLoaded(false);
      setMapError(null);
    };
  }, [connection.ros, connection.online, showCostmap]);

  if (!connection.online) {
    return (
      <div className={`w-full h-full flex items-center justify-center bg-gray-100 dark:bg-botbot-dark rounded-lg ${className}`}>
        <div className="text-center">
          <MapIcon className="w-12 h-12 mx-auto mb-2 text-gray-400" />
          <p className="text-sm text-gray-500">{t('robotOffline', 'robotDisconnected')}</p>
        </div>
      </div>
    );
  }

  return (
    <div className={`w-full h-full bg-gray-100 dark:bg-botbot-dark rounded-lg overflow-hidden relative ${className}`}>
      {/* Map toggle button */}
      <div className="absolute top-3 right-3 z-10">
        <button
          onClick={() => setShowCostmap(!showCostmap)}
          className="bg-white dark:bg-gray-800 text-gray-700 dark:text-white text-xs rounded-lg px-3 py-1.5 border border-gray-300 dark:border-gray-700 shadow-sm hover:bg-gray-50 dark:hover:bg-gray-700 transition-colors"
          title={showCostmap ? "Show base map" : "Show costmap"}
        >
          {showCostmap ? 'Costmap' : 'Base Map'}
        </button>
      </div>

      {/* Map viewer container */}
      <div 
        ref={mapContainerRef}
        id="ros2d-map" 
        className="w-full h-full"
      />

      {/* Loading indicator */}
      {!isMapLoaded && !mapError && (
        <div className="absolute inset-0 flex items-center justify-center bg-black bg-opacity-50">
          <div className="text-center">
            <div className="w-8 h-8 border-4 border-gray-300 border-t-white rounded-full animate-spin mb-2"></div>
            <p className="text-sm text-white mb-2">Loading map...</p>
            <button
              onClick={() => {
                console.log('Manual fallback trigger');
                if (tryFallbackMapRenderingRef.current) {
                  tryFallbackMapRenderingRef.current();
                }
              }}
              className="text-xs text-gray-300 underline hover:text-white"
            >
              Use fallback renderer
            </button>
          </div>
        </div>
      )}

      {/* Error message */}
      {mapError && (
        <div className="absolute inset-0 flex items-center justify-center">
          <div className="text-center text-red-500">
            <MapIcon className="w-12 h-12 mx-auto mb-2" />
            <p className="text-sm">{mapError}</p>
          </div>
        </div>
      )}

      {/* Legend */}
      {isMapLoaded && (
        <div className="absolute bottom-3 left-3 bg-white dark:bg-gray-800 bg-opacity-90 dark:bg-opacity-90 rounded-lg p-2 text-xs">
          <div className="flex items-center gap-2 mb-1">
            <MapPin className="w-3 h-3 text-orange-500" />
            <span className="text-gray-700 dark:text-gray-300">Robot Position</span>
          </div>
          <div className="flex items-center gap-2">
            <div className="w-3 h-3 bg-gray-700 dark:bg-gray-300"></div>
            <span className="text-gray-700 dark:text-gray-300">
              {showCostmap ? 'Obstacles' : 'Walls'}
            </span>
          </div>
        </div>
      )}
    </div>
  );
}
