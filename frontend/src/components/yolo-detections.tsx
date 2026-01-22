'use client';

import { useEffect, useState, useRef, useCallback, useMemo, forwardRef, useImperativeHandle } from 'react';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { formatDistanceToNow, subHours, subDays } from 'date-fns';
import { ImageOff, Clock, AlertCircle, Sparkles, Loader2, Maximize2, Trash2 } from 'lucide-react';
import { useRouter } from 'next/navigation';
import type { TimeFilter, ImageFilter, ConfidenceFilter } from './yolo-filter-bar';
import { getSignedImageUrl, deleteStorageImage } from '@/utils/supabase-storage';

interface YoloDetection {
  id: number;
  created_at: string;
  user_id: string;
  object_identified: string;
  image_link: string | null;
  object_id: number;
  confidence_score: number;
  track_id: number | null;
  session_id: string;
}

interface YoloDetectionWithSignedUrl extends YoloDetection {
  signed_url?: string | null;
}

const getConfidenceColor = (confidence: number) => {
  if (confidence >= 0.9) return 'bg-green-500/10 text-green-600 dark:text-green-400 border-green-500/20';
  if (confidence >= 0.7) return 'bg-yellow-500/10 text-yellow-600 dark:text-yellow-400 border-yellow-500/20';
  return 'bg-orange-500/10 text-orange-600 dark:text-orange-400 border-orange-500/20';
};

const ITEMS_PER_PAGE = 20; // Reduced for better performance
const INITIAL_LOAD = 20; // Initial items to load

export interface YoloDetectionsRef {
  triggerDeleteAll: () => void;
}

interface YoloDetectionsProps {
  timeFilter: TimeFilter;
  imageFilter: ImageFilter;
  confidenceFilter: ConfidenceFilter;
  onTotalCountChange?: (count: number) => void;
  onFilteredCountChange?: (count: number) => void;
  scrollContainerRef?: React.RefObject<HTMLDivElement | null>;
  expandedView?: boolean;
  realtimeEnabled?: boolean;
}

const YoloDetections = forwardRef<YoloDetectionsRef, YoloDetectionsProps>((
  {
    timeFilter,
    imageFilter,
    confidenceFilter,
    onTotalCountChange,
    onFilteredCountChange,
    scrollContainerRef,
    expandedView = false,
    realtimeEnabled = false,
  },
  ref
) => {
  const { supabase, user } = useSupabase();
  const router = useRouter();
  const [detections, setDetections] = useState<YoloDetectionWithSignedUrl[]>([]);
  const [loading, setLoading] = useState(true);
  const [loadingMore, setLoadingMore] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [imageLoadingStates, setImageLoadingStates] = useState<Record<number, boolean>>({});
  const [hasMore, setHasMore] = useState(true);
  const [allDetections, setAllDetections] = useState<YoloDetectionWithSignedUrl[]>([]);
  const [totalCount, setTotalCount] = useState<number | null>(null);
  const [deletingIds, setDeletingIds] = useState<Set<number>>(new Set());
  const [showDeleteConfirm, setShowDeleteConfirm] = useState<number | null>(null);
  const [showDeleteAllConfirm, setShowDeleteAllConfirm] = useState(false);
  const [isDeletingAll, setIsDeletingAll] = useState(false);
  const [currentPage, setCurrentPage] = useState(0);
  const [totalFilteredCount, setTotalFilteredCount] = useState<number | null>(null);
  
  // Use refs for values that shouldn't trigger re-renders
  const observerRef = useRef<IntersectionObserver | null>(null);
  const loadMoreRef = useRef<HTMLDivElement>(null);
  const isLoadingRef = useRef(false);
  const hasMoreRef = useRef(true);
  const isMountedRef = useRef(true);

  // Build filter query
  const buildFilterQuery = useCallback((query: any) => {
    // Apply time filter
    if (timeFilter !== 'all') {
      const now = new Date();
      let cutoffDate: Date;

      switch (timeFilter) {
        case '1h':
          cutoffDate = subHours(now, 1);
          break;
        case '6h':
          cutoffDate = subHours(now, 6);
          break;
        case '24h':
          cutoffDate = subHours(now, 24);
          break;
        case '7d':
          cutoffDate = subDays(now, 7);
          break;
        default:
          cutoffDate = new Date(0);
      }

      query = query.gte('created_at', cutoffDate.toISOString());
    }

    // Apply confidence filter
    if (confidenceFilter !== 'all') {
      switch (confidenceFilter) {
        case 'high':
          query = query.gte('confidence_score', 0.9);
          break;
        case 'medium':
          query = query.gte('confidence_score', 0.7).lt('confidence_score', 0.9);
          break;
        case 'low':
          query = query.lt('confidence_score', 0.7);
          break;
      }
    }

    // Note: Image filter will be applied client-side after fetching
    return query;
  }, [timeFilter, confidenceFilter]);

  // Function to fetch initial detections with pagination
  const fetchInitialDetections = useCallback(async () => {
    if (!supabase || !user || !isMountedRef.current) {
      console.log('Cannot fetch - missing supabase or user or unmounted');
      return;
    }

    if (isLoadingRef.current) {
      console.log('Already loading, skipping fetch');
      return;
    }

    isLoadingRef.current = true;
    setLoading(true);
    setError(null);
    setCurrentPage(0);

    try {
      console.log('Starting to fetch initial detections with pagination');

      // Build the base query
      let query = supabase
        .from('yolo_data')
        .select('*', { count: 'exact' })
        .eq('user_id', user.id);

      // Apply filters
      query = buildFilterQuery(query);

      // Apply ordering and pagination
      query = query
        .order('created_at', { ascending: false })
        .range(0, INITIAL_LOAD - 1);

      const { data, error, count } = await query;

      if (error) {
        console.error('Supabase error:', error);
        throw error;
      }

      if (count !== null) {
        setTotalFilteredCount(count);
        console.log(`Total filtered items: ${count}`);
      }

      if (data && data.length > 0 && isMountedRef.current) {
        // Generate signed URLs for detections with images
        const detectionsWithSignedUrls = await Promise.all(
          data.map(async (detection) => {
            const signed_url = await getSignedImageUrl(supabase, detection.image_link);
            return { ...detection, signed_url };
          })
        );

        // Apply image filter client-side
        let filteredData = detectionsWithSignedUrls;
        if (imageFilter === 'with-images') {
          filteredData = filteredData.filter(d => d.image_link && d.image_link !== 'n/a');
        } else if (imageFilter === 'without-images') {
          filteredData = filteredData.filter(d => !d.image_link || d.image_link === 'n/a');
        }

        setAllDetections(filteredData);
        setHasMore(count ? count > INITIAL_LOAD : false);
        hasMoreRef.current = count ? count > INITIAL_LOAD : false;
        setCurrentPage(1);
        console.log(`Loaded ${filteredData.length} initial items`);
      } else {
        setAllDetections([]);
        setHasMore(false);
        hasMoreRef.current = false;
      }
    } catch (err) {
      console.error('Error fetching YOLO detections:', err);
      if (isMountedRef.current) {
        setError('Failed to load detections');
      }
    } finally {
      if (isMountedRef.current) {
        setLoading(false);
        isLoadingRef.current = false;
      }
    }
  }, [supabase, user, buildFilterQuery, imageFilter]);

  // Function to fetch only new detections (for realtime updates)
  const fetchNewDetections = useCallback(async () => {
    if (!supabase || !user || !isMountedRef.current) {
      return;
    }

    try {
      // Get the timestamp of the most recent detection we have
      const mostRecentTimestamp = allDetections.length > 0 
        ? allDetections[0].created_at 
        : new Date(0).toISOString();

      console.log('Checking for new detections after:', mostRecentTimestamp);

      // Fetch only detections newer than our most recent one
      const { data, error } = await supabase
        .from('yolo_data')
        .select('*')
        .eq('user_id', user.id)
        .gt('created_at', mostRecentTimestamp)
        .order('created_at', { ascending: false })
        .limit(100);

      if (error) {
        console.error('Error fetching new detections:', error);
        return;
      }

      if (data && data.length > 0) {
        console.log(`Found ${data.length} new detections`);
        
        // Generate signed URLs for new detections
        const newDetectionsWithUrls = await Promise.all(
          data.map(async (detection) => {
            const signed_url = await getSignedImageUrl(supabase, detection.image_link);
            return { ...detection, signed_url };
          })
        );

        // Add new detections to the beginning of the list
        setAllDetections(prev => {
          // Filter out any duplicates (just in case)
          const existingIds = new Set(prev.map(d => d.id));
          const uniqueNewDetections = newDetectionsWithUrls.filter(d => !existingIds.has(d.id));
          
          if (uniqueNewDetections.length > 0) {
            return [...uniqueNewDetections, ...prev];
          }
          return prev;
        });
      }
    } catch (err) {
      console.error('Error in fetchNewDetections:', err);
    }
  }, [supabase, user, allDetections]);

  // Function to load more on scroll
  const loadMoreOnScroll = useCallback(async () => {
    if (!supabase || !user || !hasMoreRef.current || isLoadingRef.current || !isMountedRef.current) {
      return;
    }

    const offset = currentPage * ITEMS_PER_PAGE;

    // If we've reached the total count, no more to load
    if (totalFilteredCount !== null && offset >= totalFilteredCount) {
      setHasMore(false);
      hasMoreRef.current = false;
      return;
    }

    isLoadingRef.current = true;
    setLoadingMore(true);

    try {
      console.log(`Loading page ${currentPage + 1}, offset: ${offset}`);

      // Build the base query
      let query = supabase
        .from('yolo_data')
        .select('*')
        .eq('user_id', user.id);

      // Apply filters
      query = buildFilterQuery(query);

      // Apply ordering and pagination
      query = query
        .order('created_at', { ascending: false })
        .range(offset, offset + ITEMS_PER_PAGE - 1);

      const { data, error } = await query;

      if (error) {
        console.error('Supabase error:', error);
        throw error;
      }

      if (data && data.length > 0 && isMountedRef.current) {
        // Generate signed URLs for the new detections
        const detectionsWithSignedUrls = await Promise.all(
          data.map(async (detection) => {
            const signed_url = await getSignedImageUrl(supabase, detection.image_link);
            return { ...detection, signed_url };
          })
        );

        // Apply image filter client-side
        let filteredData = detectionsWithSignedUrls;
        if (imageFilter === 'with-images') {
          filteredData = filteredData.filter(d => d.image_link && d.image_link !== 'n/a');
        } else if (imageFilter === 'without-images') {
          filteredData = filteredData.filter(d => !d.image_link || d.image_link === 'n/a');
        }

        setAllDetections(prev => [...prev, ...filteredData]);
        setCurrentPage(prev => prev + 1);
        console.log(`Loaded ${filteredData.length} more items`);

        // Check if we have more
        const newTotal = allDetections.length + filteredData.length;
        if (totalFilteredCount !== null && newTotal >= totalFilteredCount) {
          setHasMore(false);
          hasMoreRef.current = false;
        } else if (data.length < ITEMS_PER_PAGE) {
          setHasMore(false);
          hasMoreRef.current = false;
        }
      } else {
        setHasMore(false);
        hasMoreRef.current = false;
      }
    } catch (err) {
      console.error('Error loading more detections:', err);
    } finally {
      if (isMountedRef.current) {
        setLoadingMore(false);
        isLoadingRef.current = false;
      }
    }
  }, [supabase, user, currentPage, totalFilteredCount, buildFilterQuery, allDetections.length, imageFilter]);

  // Since filters are now applied at query level, filteredDetections is just allDetections
  const filteredDetections = useMemo(() => {
    return allDetections;
  }, [allDetections]);

  // Delete a single detection
  const deleteDetection = useCallback(async (detection: YoloDetectionWithSignedUrl) => {
    if (!supabase || !user) return;
    
    setDeletingIds(prev => new Set(prev).add(detection.id));
    
    try {
      // Delete from storage first
      if (detection.image_link && detection.image_link !== 'n/a') {
        await deleteStorageImage(supabase, detection.image_link);
      }
      
      // Delete from database
      const { error } = await supabase
        .from('yolo_data')
        .delete()
        .eq('id', detection.id)
        .eq('user_id', user.id);
      
      if (error) {
        console.error('Error deleting detection:', error);
        throw error;
      }
      
      // Remove from local state
      setAllDetections(prev => prev.filter(d => d.id !== detection.id));
      setShowDeleteConfirm(null);
    } catch (err) {
      console.error('Failed to delete detection:', err);
    } finally {
      setDeletingIds(prev => {
        const newSet = new Set(prev);
        newSet.delete(detection.id);
        return newSet;
      });
    }
  }, [supabase, user]);

  // Delete all filtered detections
  const deleteAllDetections = useCallback(async () => {
    if (!supabase || !user) return;

    setIsDeletingAll(true);

    try {
      // First, we need to fetch ALL IDs that match the current filters (not just loaded ones)
      console.log('Fetching all IDs to delete with current filters...');

      let allIdsToDelete: number[] = [];
      let allImageUrls: (string | null)[] = [];
      let offset = 0;
      let hasMoreToFetch = true;

      // Fetch all matching records in batches
      while (hasMoreToFetch) {
        // Build the query with filters
        let query = supabase
          .from('yolo_data')
          .select('id, image_link')
          .eq('user_id', user.id);

        // Apply filters
        query = buildFilterQuery(query);

        // Apply image filter
        if (imageFilter === 'with-images') {
          query = query.neq('image_link', 'n/a').not('image_link', 'is', null);
        } else if (imageFilter === 'without-images') {
          query = query.or('image_link.eq.n/a,image_link.is.null');
        }

        query = query
          .order('created_at', { ascending: false })
          .range(offset, offset + 999);

        const { data, error } = await query;

        if (error) {
          console.error('Error fetching IDs to delete:', error);
          throw error;
        }

        if (data && data.length > 0) {
          allIdsToDelete = [...allIdsToDelete, ...data.map(d => d.id)];
          allImageUrls = [...allImageUrls, ...data.map(d => d.image_link).filter(url => url && url !== 'n/a')];
          offset += data.length;
          hasMoreToFetch = data.length === 1000;
        } else {
          hasMoreToFetch = false;
        }
      }

      console.log(`Total items to delete: ${allIdsToDelete.length}`);

      if (allIdsToDelete.length === 0) {
        setShowDeleteAllConfirm(false);
        return;
      }

      // Delete images from storage in batches
      const validImageUrls = allImageUrls.filter((url): url is string => url !== null);
      for (let i = 0; i < validImageUrls.length; i += 50) {
        const batch = validImageUrls.slice(i, i + 50);
        await Promise.all(batch.map(url => deleteStorageImage(supabase, url)));
        console.log(`Deleted ${Math.min(50, batch.length)} images from storage...`);
      }

      // Delete from database in batches
      for (let i = 0; i < allIdsToDelete.length; i += 100) {
        const batch = allIdsToDelete.slice(i, i + 100);
        const { error } = await supabase
          .from('yolo_data')
          .delete()
          .in('id', batch)
          .eq('user_id', user.id);

        if (error) {
          console.error('Error deleting batch:', error);
          throw error;
        }
        console.log(`Deleted ${batch.length} records from database...`);
      }

      // Clear local state and reset
      setAllDetections([]);
      setTotalFilteredCount(0);
      setShowDeleteAllConfirm(false);
      setHasMore(false);
      hasMoreRef.current = false;
      setCurrentPage(0);

      // Refresh the data
      fetchInitialDetections();
    } catch (err) {
      console.error('Failed to delete all detections:', err);
    } finally {
      setIsDeletingAll(false);
    }
  }, [supabase, user, buildFilterQuery, imageFilter, fetchInitialDetections]);

  // Update detections when filters change
  useEffect(() => {
    setDetections(filteredDetections);
  }, [filteredDetections]);


  // Component lifecycle
  useEffect(() => {
    isMountedRef.current = true;
    return () => {
      isMountedRef.current = false;
    };
  }, []);

  // Initial load and filter changes - fetch data with pagination
  useEffect(() => {
    if (!supabase || !user) return;

    // Reset state for new user/session or filter change
    setHasMore(true);
    hasMoreRef.current = true;
    setAllDetections([]);
    setCurrentPage(0);
    isLoadingRef.current = false;

    // Fetch initial detections with filters
    console.log('Initial fetch starting with pagination and filters');
    fetchInitialDetections();
  }, [supabase, user, timeFilter, imageFilter, confidenceFilter, fetchInitialDetections]);

  // Auto-refresh when realtime is enabled
  useEffect(() => {
    if (!realtimeEnabled || !supabase || !user) return;

    console.log('Setting up auto-refresh interval (2 seconds)');
    const interval = setInterval(() => {
      console.log('Auto-refreshing - checking for new detections...');
      fetchNewDetections();
    }, 2000);

    return () => {
      console.log('Clearing auto-refresh interval');
      clearInterval(interval);
    };
  }, [realtimeEnabled, supabase, user, fetchNewDetections]);

  // Real-time subscription
  useEffect(() => {
    if (!supabase || !user) return;

    const subscription = supabase
      .channel('yolo_detections')
      .on(
        'postgres_changes',
        {
          event: 'INSERT',
          schema: 'public',
          table: 'yolo_data',
          filter: `user_id=eq.${user.id}`
        },
        async (payload) => {
          console.log('New detection received via subscription');
          const newDetection = payload.new as YoloDetection;
          const signed_url = await getSignedImageUrl(supabase, newDetection.image_link);
          setAllDetections(prev => [{ ...newDetection, signed_url }, ...prev]);
        }
      )
      .subscribe();

    return () => {
      subscription.unsubscribe();
    };
  }, [supabase, user]);

  // Notify parent of count changes
  useEffect(() => {
    // Use total filtered count if available, otherwise use loaded count
    onTotalCountChange?.(totalFilteredCount || allDetections.length);
  }, [totalFilteredCount, allDetections.length, onTotalCountChange]);

  useEffect(() => {
    // Since filters are applied at query level, filtered count is same as total filtered count
    onFilteredCountChange?.(totalFilteredCount || filteredDetections.length);
  }, [totalFilteredCount, filteredDetections.length, onFilteredCountChange]);

  // Expose delete all trigger to parent
  useImperativeHandle(ref, () => ({
    triggerDeleteAll: () => setShowDeleteAllConfirm(true)
  }), []);

  // Set up intersection observer for infinite scroll (if needed for future pagination)
  useEffect(() => {
    // Clean up previous observer
    if (observerRef.current) {
      observerRef.current.disconnect();
      observerRef.current = null;
    }

    // Only set up observer if we have more data to load
    if (!hasMore || allDetections.length === 0) {
      return;
    }

    // Get the scroll container
    const scrollRoot = scrollContainerRef?.current || null;
    console.log('Setting up observer, hasMore:', hasMore, 'scrollRoot:', scrollRoot ? 'custom' : 'viewport');

    // Create new observer
    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach(entry => {
          if (entry.isIntersecting && hasMoreRef.current && !isLoadingRef.current) {
            console.log('Load more trigger is visible!');
            loadMoreOnScroll();
          }
        });
      },
      { 
        root: scrollRoot,
        threshold: 0.1,
        rootMargin: '200px'
      }
    );

    // Start observing immediately if element exists
    if (loadMoreRef.current) {
      console.log('Starting to observe load more trigger');
      observer.observe(loadMoreRef.current);
      observerRef.current = observer;
    }

    return () => {
      if (observerRef.current) {
        observerRef.current.disconnect();
        observerRef.current = null;
      }
    };
  }, [hasMore, scrollContainerRef, loadMoreOnScroll, allDetections.length]);

  if (loading) {
    return (
      <div className="h-full flex items-center justify-center">
        <div className="text-center">
          <div className="w-8 h-8 border-2 border-purple-500 border-t-transparent rounded-full animate-spin mx-auto mb-2"></div>
          <p className="text-sm text-gray-600 dark:text-gray-400">Loading detections...</p>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="h-full flex items-center justify-center">
        <div className="text-center">
          <AlertCircle className="w-8 h-8 text-red-500 mx-auto mb-2" />
          <p className="text-sm text-red-600 dark:text-red-400">{error}</p>
        </div>
      </div>
    );
  }

  if (detections.length === 0 && !hasMore) {
    return (
      <div className="h-full flex items-center justify-center">
        <div className="text-center">
          <div className="w-16 h-16 rounded-2xl bg-gradient-to-br from-purple-100 to-pink-100 dark:from-purple-900/20 dark:to-pink-900/20 flex items-center justify-center mb-4 mx-auto">
            <Sparkles className="w-8 h-8 text-purple-600 dark:text-purple-400" />
          </div>
          <p className="text-gray-700 dark:text-gray-300 font-medium">No detections yet</p>
          <p className="text-sm text-gray-500 dark:text-gray-400 mt-2">
            Objects detected by YOLO will appear here
          </p>
        </div>
      </div>
    );
  }

  return (
    <>
      {/* Delete All Confirmation Dialog */}
      {showDeleteAllConfirm && (
        <div className="fixed inset-0 bg-black/50 z-50 flex items-center justify-center p-4">
          <div className="bg-white dark:bg-botbot-darker rounded-lg p-6 max-w-md w-full">
            <h3 className="text-lg font-semibold text-gray-900 dark:text-white mb-2">
              Delete {totalFilteredCount || filteredDetections.length} Detection{(totalFilteredCount || filteredDetections.length) !== 1 ? 's' : ''}?
            </h3>
            <p className="text-gray-600 dark:text-gray-400 mb-6">
              This will permanently delete {totalFilteredCount || filteredDetections.length} detection{(totalFilteredCount || filteredDetections.length) !== 1 ? 's' : ''} and their associated images from storage. This action cannot be undone.
              {totalFilteredCount && totalFilteredCount > allDetections.length && (
                <span className="block mt-2 text-xs text-yellow-600 dark:text-yellow-400">
                  Note: This includes detections that haven't been loaded yet.
                </span>
              )}
            </p>
            <div className="flex gap-3 justify-end">
              <button
                onClick={() => setShowDeleteAllConfirm(false)}
                className="px-4 py-2 text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-800 rounded-lg transition-colors"
                disabled={isDeletingAll}
              >
                Cancel
              </button>
              <button
                onClick={deleteAllDetections}
                className="px-4 py-2 bg-red-500 text-white hover:bg-red-600 rounded-lg transition-colors flex items-center gap-2"
                disabled={isDeletingAll}
              >
                {isDeletingAll ? (
                  <>
                    <Loader2 className="w-4 h-4 animate-spin" />
                    Deleting...
                  </>
                ) : (
                  <>
                    <Trash2 className="w-4 h-4" />
                    Delete All
                  </>
                )}
              </button>
            </div>
          </div>
        </div>
      )}

      <div className={`grid gap-4 ${
        expandedView 
          ? 'grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 2xl:grid-cols-5 3xl:grid-cols-6'
          : 'grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 2xl:grid-cols-4'
      }`}>
        {detections.map((detection) => (
          <div
            key={detection.id}
            className="bg-white dark:bg-botbot-darker/50 rounded-xl overflow-hidden border border-gray-200 dark:border-gray-700 hover:border-purple-400 dark:hover:border-purple-500 transition-all duration-300 hover:shadow-xl group relative"
          >
            {/* Delete Confirmation for Individual Item */}
            {showDeleteConfirm === detection.id && (
              <div className="absolute inset-0 bg-black/80 z-40 flex items-center justify-center p-4 rounded-xl">
                <div className="bg-white dark:bg-botbot-darker rounded-lg p-4 max-w-xs w-full" onClick={(e) => e.stopPropagation()}>
                  <h4 className="text-sm font-semibold text-gray-900 dark:text-white mb-2">
                    Delete this detection?
                  </h4>
                  <p className="text-xs text-gray-600 dark:text-gray-400 mb-4">
                    This will permanently delete the detection and its image.
                  </p>
                  <div className="flex gap-2">
                    <button
                      onClick={(e) => {
                        e.stopPropagation();
                        setShowDeleteConfirm(null);
                      }}
                      className="flex-1 px-3 py-1.5 text-xs text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-800 rounded-lg transition-colors"
                    >
                      Cancel
                    </button>
                    <button
                      onClick={(e) => {
                        e.stopPropagation();
                        deleteDetection(detection);
                      }}
                      className="flex-1 px-3 py-1.5 text-xs bg-red-500 text-white hover:bg-red-600 rounded-lg transition-colors flex items-center justify-center gap-1"
                      disabled={deletingIds.has(detection.id)}
                    >
                      {deletingIds.has(detection.id) ? (
                        <>
                          <Loader2 className="w-3 h-3 animate-spin" />
                          Deleting
                        </>
                      ) : (
                        <>
                          <Trash2 className="w-3 h-3" />
                          Delete
                        </>
                      )}
                    </button>
                  </div>
                </div>
              </div>
            )}
            {/* Image Section */}
            <div 
              className="relative aspect-[16/10] bg-gradient-to-br from-gray-100 to-gray-200 dark:from-gray-800 dark:to-gray-900 overflow-hidden cursor-pointer"
              onClick={() => router.push(`/ai/viewer?id=${detection.id}`)}
            >
              {detection.image_link && detection.image_link !== 'n/a' ? (
                <>
                  {/* Loading shimmer effect */}
                  {imageLoadingStates[detection.id] !== false && (
                    <div className="absolute inset-0 bg-gradient-to-r from-transparent via-white/20 to-transparent animate-shimmer" />
                  )}
                  <img
                    src={detection.signed_url || detection.image_link}
                    alt={`${detection.object_identified} detection`}
                    className="w-full h-full object-cover transition-opacity duration-300"
                    style={{ opacity: imageLoadingStates[detection.id] === false ? 1 : 0 }}
                    loading="lazy"
                    onLoad={() => {
                      setImageLoadingStates(prev => ({ ...prev, [detection.id]: false }));
                    }}
                    onError={(e) => {
                      const target = e.target as HTMLImageElement;
                      target.style.display = 'none';
                      setImageLoadingStates(prev => ({ ...prev, [detection.id]: false }));
                      const fallback = target.nextElementSibling as HTMLElement;
                      if (fallback) fallback.style.display = 'flex';
                    }}
                  />
                  <div className="hidden absolute inset-0 items-center justify-center bg-gradient-to-br from-gray-100 to-gray-200 dark:from-gray-800 dark:to-gray-900">
                    <ImageOff className="w-12 h-12 text-gray-400 dark:text-gray-600" />
                  </div>
                </>
              ) : (
                <div className="absolute inset-0 flex items-center justify-center">
                  <div className="text-center">
                    <ImageOff className="w-12 h-12 text-gray-400 dark:text-gray-600 mx-auto mb-2" />
                    <p className="text-xs text-gray-500 dark:text-gray-400">No image</p>
                  </div>
                </div>
              )}
              
              {/* View Icon Overlay on Hover */}
              <div className="absolute inset-0 bg-black/0 group-hover:bg-black/40 transition-all duration-300 flex items-center justify-center pointer-events-none">
                <div className="opacity-0 group-hover:opacity-100 transition-opacity duration-300 bg-white/10 backdrop-blur-sm rounded-full p-3">
                  <Maximize2 className="w-6 h-6 text-white" />
                </div>
              </div>
              
              {/* Confidence Badge and Delete Button */}
              <div className="absolute top-3 right-3 flex items-center gap-2">
                <button
                  onClick={(e) => {
                    e.stopPropagation();
                    setShowDeleteConfirm(detection.id);
                  }}
                  className="p-1.5 bg-red-500/10 hover:bg-red-500/20 text-red-600 dark:text-red-400 rounded-full backdrop-blur-sm transition-all opacity-0 group-hover:opacity-100"
                  title="Delete detection"
                >
                  <Trash2 className="w-3.5 h-3.5" />
                </button>
                <div className={`px-2.5 py-1 rounded-full text-xs font-semibold border ${getConfidenceColor(detection.confidence_score)} backdrop-blur-sm`}>
                  {(detection.confidence_score * 100).toFixed(0)}%
                </div>
              </div>
            </div>

            {/* Content Section */}
            <div className="p-4 cursor-pointer" onClick={() => router.push(`/ai/viewer?id=${detection.id}`)}>
              <div className="flex items-start justify-between mb-3">
                <div>
                  <h3 className="font-semibold text-gray-900 dark:text-white capitalize text-lg">
                    {detection.object_identified}
                  </h3>
                  <div className="flex items-center gap-1.5 mt-1">
                    <Clock className="w-3 h-3 text-gray-400" />
                    <span className="text-xs text-gray-500 dark:text-gray-400">
                      {formatDistanceToNow(new Date(detection.created_at), { addSuffix: true })}
                    </span>
                  </div>
                </div>
              </div>

              {/* Metadata */}
              <div className="flex items-center gap-3 text-xs text-gray-600 dark:text-gray-400">
                <div className="flex items-center gap-1">
                  <span className="font-medium">ID:</span>
                  <span className="font-mono">{detection.object_id}</span>
                </div>
                {detection.track_id && (
                  <>
                    <span className="text-gray-300 dark:text-gray-600">•</span>
                    <div className="flex items-center gap-1">
                      <span className="font-medium">Track:</span>
                      <span className="font-mono">{detection.track_id}</span>
                    </div>
                  </>
                )}
              </div>
            </div>
          </div>
        ))}
      </div>
      
      {/* Load More Trigger */}
      {hasMore && (
        <div
          ref={loadMoreRef}
          className="flex justify-center items-center py-8 relative"
          style={{ minHeight: '100px' }}
        >
          {loadingMore ? (
            <div className="flex flex-col items-center gap-3">
              <div className="relative">
                <div className="w-12 h-12 rounded-full border-4 border-gray-200 dark:border-gray-700"></div>
                <div className="absolute top-0 left-0 w-12 h-12 rounded-full border-4 border-purple-500 border-t-transparent animate-spin"></div>
              </div>
              <div className="text-center">
                <p className="text-sm font-medium text-gray-700 dark:text-gray-300">Loading more detections...</p>
                <p className="text-xs text-gray-500 dark:text-gray-400 mt-1">
                  {allDetections.length} of {totalFilteredCount || '...'} loaded
                </p>
              </div>
            </div>
          ) : (
            <div className="flex flex-col items-center gap-2">
              <div className="flex items-center gap-2 px-4 py-2 rounded-full bg-gray-100 dark:bg-gray-800">
                <div className="w-2 h-2 rounded-full bg-gray-400 dark:bg-gray-500 animate-pulse"></div>
                <span className="text-sm text-gray-600 dark:text-gray-400">Scroll for more</span>
              </div>
              <p className="text-xs text-gray-500 dark:text-gray-400">
                {allDetections.length} of {totalFilteredCount || '...'} loaded
              </p>
            </div>
          )}
        </div>
      )}

      {!hasMore && allDetections.length > 0 && (
        <div className="text-center py-8">
          <div className="inline-flex items-center gap-2 px-4 py-2 rounded-full bg-green-50 dark:bg-green-900/20 border border-green-200 dark:border-green-800">
            <svg className="w-4 h-4 text-green-600 dark:text-green-400" fill="none" viewBox="0 0 24 24" stroke="currentColor">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 13l4 4L19 7" />
            </svg>
            <p className="text-sm font-medium text-green-700 dark:text-green-300">
              All {totalFilteredCount || allDetections.length} detections loaded
            </p>
          </div>
        </div>
      )}
    </>
  );
});

YoloDetections.displayName = 'YoloDetections';

export default YoloDetections;