'use client';

import { useState, useEffect, useCallback, useRef, useMemo } from 'react';
import { Widget } from './Widget';
import { Brain, Clock, AlertCircle, ChevronRight, Loader2, RefreshCw } from 'lucide-react';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { formatDistanceToNow } from 'date-fns';
import { useRouter } from 'next/navigation';
import { getSignedImageUrl } from '@/utils/supabase-storage';

interface Detection {
  id: number;
  created_at: string;
  object_identified: string;
  confidence_score: number;
  image_link: string | null;
  signed_url?: string | null;
}

interface RecentDetectionsWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
}

const getConfidenceColor = (confidence: number) => {
  if (confidence >= 0.9) return 'text-green-600 dark:text-green-400';
  if (confidence >= 0.7) return 'text-yellow-600 dark:text-yellow-400';
  return 'text-orange-600 dark:text-orange-400';
};

export function RecentDetectionsWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 400, height: 500 },
}: RecentDetectionsWidgetProps) {
  const { supabase, user } = useSupabase();
  const router = useRouter();
  const [detections, setDetections] = useState<Detection[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [realtimeEnabled, setRealtimeEnabled] = useState(false);
  const [widgetSize, setWidgetSize] = useState(initialSize);
  const isMountedRef = useRef(true);
  
  // Calculate how many items to show based on widget height
  const maxItems = useMemo(() => {
    // Each card is roughly 80px height, plus padding and button
    const availableHeight = widgetSize.height - 120; // Account for header, padding, button
    const itemHeight = 80;
    return Math.max(3, Math.floor(availableHeight / itemHeight));
  }, [widgetSize.height]);

  const fetchDetections = useCallback(async () => {
    if (!supabase || !user || !isMountedRef.current) return;

    try {
      setError(null);
      const { data, error } = await supabase
        .from('yolo_data')
        .select('id, created_at, object_identified, confidence_score, image_link')
        .eq('user_id', user.id)
        .order('created_at', { ascending: false })
        .limit(maxItems + 5); // Fetch a few extra to ensure we have enough

      if (error) throw error;

      if (data && isMountedRef.current) {
        // Generate signed URLs for detections with images
        const detectionsWithUrls = await Promise.all(
          data.slice(0, maxItems).map(async (detection) => {
            const signed_url = await getSignedImageUrl(supabase, detection.image_link);
            return { ...detection, signed_url };
          })
        );
        setDetections(detectionsWithUrls);
      }
    } catch (err) {
      console.error('Error fetching detections:', err);
      if (isMountedRef.current) {
        setError('Failed to load detections');
      }
    } finally {
      if (isMountedRef.current) {
        setLoading(false);
      }
    }
  }, [supabase, user, maxItems]);
  
  // Function to fetch only new detections (for auto-refresh)
  const fetchNewDetections = useCallback(async () => {
    if (!supabase || !user || !isMountedRef.current) return;

    try {
      // Get the timestamp of the most recent detection we have
      const mostRecentTimestamp = detections.length > 0 
        ? detections[0].created_at 
        : new Date(0).toISOString();

      // Fetch only detections newer than our most recent one
      const { data, error } = await supabase
        .from('yolo_data')
        .select('id, created_at, object_identified, confidence_score, image_link')
        .eq('user_id', user.id)
        .gt('created_at', mostRecentTimestamp)
        .order('created_at', { ascending: false })
        .limit(10);

      if (error) {
        console.error('Error fetching new detections:', error);
        return;
      }

      if (data && data.length > 0 && isMountedRef.current) {
        // Generate signed URLs for new detections
        const newDetectionsWithUrls = await Promise.all(
          data.map(async (detection) => {
            const signed_url = await getSignedImageUrl(supabase, detection.image_link);
            return { ...detection, signed_url };
          })
        );

        // Add new detections to the beginning and limit to maxItems
        setDetections(prev => {
          const combined = [...newDetectionsWithUrls, ...prev];
          return combined.slice(0, maxItems);
        });
      }
    } catch (err) {
      console.error('Error in fetchNewDetections:', err);
    }
  }, [supabase, user, detections, maxItems]);

  // Component lifecycle
  useEffect(() => {
    isMountedRef.current = true;
    return () => {
      isMountedRef.current = false;
    };
  }, []);
  
  // Initial fetch and refetch when maxItems changes
  useEffect(() => {
    fetchDetections();
  }, [fetchDetections]);
  
  // Auto-refresh when realtime is enabled
  useEffect(() => {
    if (!realtimeEnabled || !supabase || !user) return;

    const interval = setInterval(() => {
      fetchNewDetections();
    }, 2000); // Refresh every 2 seconds

    return () => {
      clearInterval(interval);
    };
  }, [realtimeEnabled, supabase, user, fetchNewDetections]);
  
  // Real-time subscription
  useEffect(() => {
    if (!supabase || !user) return;

    const channel = supabase
      .channel('yolo_detections_widget')
      .on(
        'postgres_changes',
        {
          event: 'INSERT',
          schema: 'public',
          table: 'yolo_data',
          filter: `user_id=eq.${user.id}`,
        },
        async (payload) => {
          const newDetection = payload.new as Detection;
          const signed_url = await getSignedImageUrl(supabase, newDetection.image_link);
          setDetections(prev => [
            { ...newDetection, signed_url },
            ...prev.slice(0, maxItems - 1)
          ]);
        }
      )
      .subscribe();

    return () => {
      supabase.removeChannel(channel);
    };
  }, [supabase, user, maxItems]);

  const handleCardClick = (detectionId: number) => {
    router.push(`/ai/viewer?id=${detectionId}`);
  };

  const handleViewAll = () => {
    router.push('/ai');
  };

  return (
    <Widget
      id={id}
      title="Recent AI Detections"
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      initialPosition={initialPosition}
      initialSize={initialSize}
      minWidth={320}
      minHeight={300}
      onResize={(size) => setWidgetSize(size)}
    >
      <div className="h-full flex flex-col p-2">
        {/* Auto-refresh toggle */}
        <div className="flex items-center justify-between mb-2 px-1">
          <span className="text-xs text-gray-500 dark:text-gray-400">
            Showing {Math.min(detections.length, maxItems)} recent
          </span>
          <button
            onClick={() => setRealtimeEnabled(!realtimeEnabled)}
            className={`flex items-center gap-1 px-2 py-1 rounded-md text-xs transition-colors ${
              realtimeEnabled
                ? 'bg-green-100 dark:bg-green-900/30 text-green-700 dark:text-green-400'
                : 'bg-gray-100 dark:bg-gray-800 text-gray-600 dark:text-gray-400'
            }`}
            title={realtimeEnabled ? 'Auto-refresh enabled' : 'Auto-refresh disabled'}
          >
            <RefreshCw className={`w-3 h-3 ${realtimeEnabled ? 'animate-spin' : ''}`} />
            {realtimeEnabled ? 'Live' : 'Paused'}
          </button>
        </div>
        {loading ? (
          <div className="flex-1 flex items-center justify-center">
            <Loader2 className="w-6 h-6 animate-spin text-gray-400" />
          </div>
        ) : error ? (
          <div className="flex-1 flex items-center justify-center">
            <div className="text-center">
              <AlertCircle className="w-8 h-8 text-red-500 mx-auto mb-2" />
              <p className="text-sm text-gray-600 dark:text-gray-400">{error}</p>
            </div>
          </div>
        ) : detections.length === 0 ? (
          <div className="flex-1 flex items-center justify-center">
            <div className="text-center">
              <Brain className="w-12 h-12 text-gray-300 dark:text-gray-600 mx-auto mb-3" />
              <p className="text-sm text-gray-500 dark:text-gray-400">No detections yet</p>
            </div>
          </div>
        ) : (
          <>
            <div className="flex-1 overflow-y-auto custom-scrollbar">
              <div className="space-y-1.5">
              {detections.map((detection) => (
                <div
                  key={detection.id}
                  onClick={() => handleCardClick(detection.id)}
                  className="bg-white dark:bg-botbot-darkest/50 rounded-lg p-2.5 cursor-pointer
                           hover:bg-gray-50 dark:hover:bg-botbot-darkest/70 transition-all duration-200
                           border border-gray-200 dark:border-gray-700 hover:shadow-md"
                >
                  <div className="flex items-start gap-2.5">
                    {detection.signed_url ? (
                      <div className="w-14 h-14 rounded-md overflow-hidden flex-shrink-0 bg-gray-100 dark:bg-gray-800">
                        <img
                          src={detection.signed_url}
                          alt={detection.object_identified}
                          className="w-full h-full object-cover"
                          onError={(e) => {
                            const target = e.target as HTMLImageElement;
                            target.style.display = 'none';
                            const parent = target.parentElement;
                            if (parent) {
                              parent.innerHTML = '<div class="w-full h-full flex items-center justify-center"><svg class="w-6 h-6 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M4 16l4.586-4.586a2 2 0 012.828 0L16 16m-2-2l1.586-1.586a2 2 0 012.828 0L20 14m-6-6h.01M6 20h12a2 2 0 002-2V6a2 2 0 00-2-2H6a2 2 0 00-2 2v12a2 2 0 002 2z"></path></svg></div>';
                            }
                          }}
                        />
                      </div>
                    ) : (
                      <div className="w-14 h-14 rounded-md bg-gray-100 dark:bg-gray-800 flex items-center justify-center flex-shrink-0">
                        <Brain className="w-5 h-5 text-gray-400" />
                      </div>
                    )}
                    
                    <div className="flex-1 min-w-0">
                      <div className="flex items-center justify-between mb-1">
                        <h4 className="font-medium text-sm text-gray-900 dark:text-white truncate">
                          {detection.object_identified}
                        </h4>
                        <ChevronRight className="w-4 h-4 text-gray-400 flex-shrink-0 ml-2" />
                      </div>
                      
                      <div className="flex items-center gap-3 text-xs">
                        <span className={`font-medium ${getConfidenceColor(detection.confidence_score)}`}>
                          {(detection.confidence_score * 100).toFixed(0)}%
                        </span>
                        <span className="text-gray-500 dark:text-gray-400 flex items-center gap-1">
                          <Clock className="w-3 h-3" />
                          {formatDistanceToNow(new Date(detection.created_at), { addSuffix: true })}
                        </span>
                      </div>
                    </div>
                  </div>
                </div>
              ))}
              </div>
            </div>
            
            <button
              onClick={handleViewAll}
              className="mt-2 w-full py-1.5 px-3 bg-botbot-primary/10 hover:bg-botbot-primary/20 
                       text-botbot-primary dark:text-botbot-primary rounded-lg transition-colors
                       text-sm font-medium flex items-center justify-center gap-2 flex-shrink-0"
            >
              View All Detections
              <ChevronRight className="w-4 h-4" />
            </button>
          </>
        )}
      </div>
    </Widget>
  );
}