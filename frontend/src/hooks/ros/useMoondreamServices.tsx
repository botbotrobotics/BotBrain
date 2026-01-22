'use client';

import { useCallback, useState } from 'react';
import * as ROSLIB from 'roslib';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';

export interface MoondreamBoundingBox {
  x_min: number;
  y_min: number;
  x_max: number;
  y_max: number;
}

export interface MoondreamPoint {
  x: number;
  y: number;
}

export interface MoondreamGrounding {
  start_idx: number;
  end_idx: number;
  points: [number, number][];
}

export interface MoondreamResponse {
  success: boolean;
  caption?: string;
  answer?: string;
  reasoning?: {
    text: string;
    grounding: MoondreamGrounding[];
  };
  objects?: MoondreamBoundingBox[];
  points?: MoondreamPoint[];
  error?: string;
}

export default function useMoondreamServices() {
  const { connection } = useRobotConnection();
  const [loading, setLoading] = useState(false);
  const [boundingBoxes, setBoundingBoxes] = useState<MoondreamBoundingBox[]>([]);
  const [points, setPoints] = useState<MoondreamPoint[]>([]);

  const callCaption = useCallback(async (): Promise<MoondreamResponse> => {
    if (!connection.ros || !connection.online) {
      return { success: false, error: 'Robot not connected' };
    }

    setLoading(true);
    setBoundingBoxes([]);
    setPoints([]);

    return new Promise((resolve) => {
      const captionService = new ROSLIB.Service({
        ros: connection.ros!,
        name: '/caption',
        serviceType: 'std_srvs/Trigger'
      });

      const request = {};

      captionService.callService(request, (response: any) => {
        setLoading(false);
        if (response.success) {
          try {
            // First try to parse as JSON
            const data = JSON.parse(response.message);
            // Extract caption from parsed data
            if (data.caption) {
              resolve({ success: true, caption: data.caption });
            } else if (typeof data === 'string') {
              resolve({ success: true, caption: data });
            } else {
              resolve({ success: true, caption: response.message });
            }
          } catch (error) {
            // If parsing fails, check if it's a Python dict string format
            const captionMatch = response.message.match(/['""]caption['""]:\s*['""]([^'"]+)['"]/);
            if (captionMatch && captionMatch[1]) {
              resolve({ success: true, caption: captionMatch[1] });
            } else {
              // Fallback to raw message
              resolve({ success: true, caption: response.message });
            }
          }
        } else {
          resolve({ success: false, error: response.message || 'Caption generation failed' });
        }
      }, (error: any) => {
        setLoading(false);
        resolve({ success: false, error: error || 'Service call failed' });
      });
    });
  }, [connection]);

  const callQuery = useCallback(async (question: string, reasoning: boolean): Promise<MoondreamResponse> => {
    if (!connection.ros || !connection.online) {
      return { success: false, error: 'Robot not connected' };
    }

    setLoading(true);
    setBoundingBoxes([]);
    setPoints([]);

    return new Promise((resolve) => {
      const queryService = new ROSLIB.Service({
        ros: connection.ros!,
        name: '/query',
        serviceType: 'bot_yolo_interfaces/Query'
      });

      const request = {
        question: question,
        reasoning: reasoning
      };

      queryService.callService(request, (response: any) => {
        setLoading(false);
        if (response.success) {
          try {
            // First try to parse as JSON
            const data = JSON.parse(response.answer);

            // Handle reasoning response with bounding boxes
            if (data.reasoning) {
              // Convert grounding points to bounding boxes for visualization
              const boxes: MoondreamBoundingBox[] = [];
              if (data.reasoning.grounding && Array.isArray(data.reasoning.grounding)) {
                data.reasoning.grounding.forEach((ground: MoondreamGrounding) => {
                  if (ground.points && ground.points.length > 0) {
                    // Find min/max to create bounding box from points
                    let minX = 1, minY = 1, maxX = 0, maxY = 0;
                    ground.points.forEach((point: [number, number]) => {
                      minX = Math.min(minX, point[0]);
                      minY = Math.min(minY, point[1]);
                      maxX = Math.max(maxX, point[0]);
                      maxY = Math.max(maxY, point[1]);
                    });
                    // Add some padding to make the box visible
                    const padding = 0.02;
                    boxes.push({
                      x_min: Math.max(0, minX - padding),
                      y_min: Math.max(0, minY - padding),
                      x_max: Math.min(1, maxX + padding),
                      y_max: Math.min(1, maxY + padding)
                    });
                  }
                });
              }
              setBoundingBoxes(boxes);

              resolve({
                success: true,
                answer: data.answer,
                reasoning: data.reasoning
              });
            } else if (data.answer) {
              // Simple answer without reasoning
              resolve({ success: true, answer: data.answer });
            } else if (typeof data === 'string') {
              resolve({ success: true, answer: data });
            } else {
              resolve({ success: true, answer: response.answer });
            }
          } catch (error) {
            // If parsing fails, check if it's a Python dict string format
            const answerMatch = response.answer.match(/['""]answer['""]:\s*['""]([^'"]+)['"]/);
            if (answerMatch && answerMatch[1]) {
              resolve({ success: true, answer: answerMatch[1] });
            } else {
              // Fallback to raw answer
              resolve({ success: true, answer: response.answer });
            }
          }
        } else {
          resolve({ success: false, error: response.answer || 'Query failed' });
        }
      }, (error: any) => {
        setLoading(false);
        resolve({ success: false, error: error || 'Service call failed' });
      });
    });
  }, [connection]);

  const callDetect = useCallback(async (description: string): Promise<MoondreamResponse> => {
    if (!connection.ros || !connection.online) {
      return { success: false, error: 'Robot not connected' };
    }

    setLoading(true);
    setPoints([]);

    return new Promise((resolve) => {
      const detectService = new ROSLIB.Service({
        ros: connection.ros!,
        name: '/detect',
        serviceType: 'bot_yolo_interfaces/Detect'
      });

      const request = {
        description: description
      };

      detectService.callService(request, (response: any) => {
        setLoading(false);
        if (response.success) {
          try {
            const data = JSON.parse(response.result);
            if (data.objects && Array.isArray(data.objects)) {
              setBoundingBoxes(data.objects);
              resolve({ success: true, objects: data.objects });
            } else {
              resolve({ success: false, error: 'No objects detected' });
            }
          } catch (error) {
            // If parsing fails, try to parse Python dict string format
            try {
              // Replace single quotes with double quotes and parse
              const jsonStr = response.result
                .replace(/'/g, '"')
                .replace(/True/g, 'true')
                .replace(/False/g, 'false')
                .replace(/None/g, 'null');

              const data = JSON.parse(jsonStr);
              if (data.objects && Array.isArray(data.objects)) {
                setBoundingBoxes(data.objects);
                resolve({ success: true, objects: data.objects });
              } else {
                resolve({ success: false, error: 'No objects detected' });
              }
            } catch (parseError) {
              console.error('Failed to parse detection result:', response.result, parseError);
              resolve({ success: false, error: 'Failed to parse detection result' });
            }
          }
        } else {
          resolve({ success: false, error: response.result || 'Detection failed' });
        }
      }, (error: any) => {
        setLoading(false);
        resolve({ success: false, error: error || 'Service call failed' });
      });
    });
  }, [connection]);

  const callPoint = useCallback(async (description: string): Promise<MoondreamResponse> => {
    if (!connection.ros || !connection.online) {
      return { success: false, error: 'Robot not connected' };
    }

    setLoading(true);
    setBoundingBoxes([]);

    return new Promise((resolve) => {
      const pointService = new ROSLIB.Service({
        ros: connection.ros!,
        name: '/point',
        serviceType: 'bot_yolo_interfaces/Point'
      });

      const request = {
        description: description
      };

      pointService.callService(request, (response: any) => {
        setLoading(false);
        if (response.success) {
          try {
            const data = JSON.parse(response.result);
            if (data.points && Array.isArray(data.points)) {
              setPoints(data.points);
              resolve({ success: true, points: data.points });
            } else {
              resolve({ success: false, error: 'No points found' });
            }
          } catch (error) {
            // If parsing fails, try to parse Python dict string format
            try {
              // Replace single quotes with double quotes and parse
              const jsonStr = response.result
                .replace(/'/g, '"')
                .replace(/True/g, 'true')
                .replace(/False/g, 'false')
                .replace(/None/g, 'null');

              const data = JSON.parse(jsonStr);
              if (data.points && Array.isArray(data.points)) {
                setPoints(data.points);
                resolve({ success: true, points: data.points });
              } else {
                resolve({ success: false, error: 'No points found' });
              }
            } catch (parseError) {
              console.error('Failed to parse point result:', response.result, parseError);
              resolve({ success: false, error: 'Failed to parse point result' });
            }
          }
        } else {
          resolve({ success: false, error: response.result || 'Point identification failed' });
        }
      }, (error: any) => {
        setLoading(false);
        resolve({ success: false, error: error || 'Service call failed' });
      });
    });
  }, [connection]);

  const clearOverlays = useCallback(() => {
    setBoundingBoxes([]);
    setPoints([]);
  }, []);

  return {
    callCaption,
    callQuery,
    callDetect,
    callPoint,
    clearOverlays,
    loading,
    boundingBoxes,
    points
  };
}