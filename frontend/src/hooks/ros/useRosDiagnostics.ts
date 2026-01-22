'use client';

import { useState, useEffect, useCallback, useMemo, useRef } from 'react';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import * as ROSLIB from 'roslib';
import type { DiagnosticArray, DiagnosticStatus } from '@/types/RosDiagnostics';

// Cache entry for diagnostics with timestamp
interface CachedDiagnostic {
  diagnostic: DiagnosticStatus;
  timestamp: number;
  lastSeen: number;
}

// How long to keep an issue in cache after it's no longer reported (in milliseconds)
const ISSUE_CACHE_DURATION = 5000; // 5 seconds
const CACHE_CLEANUP_INTERVAL = 1000; // Clean up cache every 1 second

export function useRosDiagnostics() {
  const { connection } = useRobotConnection();
  const [diagnostics, setDiagnostics] = useState<DiagnosticStatus[]>([]);
  const [lastUpdate, setLastUpdate] = useState<Date | null>(null);
  const [isSubscribed, setIsSubscribed] = useState(false);

  // Cache for issues to prevent flashing
  const issueCache = useRef<Map<string, CachedDiagnostic>>(new Map());
  const [cachedIssues, setCachedIssues] = useState<DiagnosticStatus[]>([]);

  // Create diagnostics topic
  const diagnosticsTopic = useMemo(() => {
    if (!connection.ros || !connection.online) return null;

    return new ROSLIB.Topic({
      ros: connection.ros,
      name: '/diagnostics',
      messageType: 'diagnostic_msgs/DiagnosticArray',
    });
  }, [connection.ros, connection.online]);

  // Clean up expired cache entries
  const cleanupCache = useCallback(() => {
    const now = Date.now();
    const updatedCache = new Map<string, CachedDiagnostic>();
    let hasChanges = false;

    issueCache.current.forEach((cached, key) => {
      // Keep issues that are still within the cache duration
      if (now - cached.lastSeen < ISSUE_CACHE_DURATION) {
        updatedCache.set(key, cached);
      } else {
        hasChanges = true;
      }
    });

    if (hasChanges) {
      issueCache.current = updatedCache;
      // Update cached issues state
      const issues = Array.from(updatedCache.values())
        .filter(c => c.diagnostic.level > 0) // Only cache issues, not OK statuses
        .map(c => c.diagnostic);
      setCachedIssues(issues);
    }
  }, []);

  // Message handler with caching
  const handleDiagnosticsMessage = useCallback((message: DiagnosticArray) => {
    if (message.status && Array.isArray(message.status)) {
      const now = Date.now();
      const currentIssueKeys = new Set<string>();

      // Process incoming diagnostics
      const processedDiagnostics = message.status.map(diagnostic => {
        const key = `${diagnostic.name}-${diagnostic.hardware_id}`;

        // If it's an issue (not OK), update the cache
        if (diagnostic.level > 0) {
          currentIssueKeys.add(key);

          const existingCache = issueCache.current.get(key);
          if (!existingCache || existingCache.diagnostic.level !== diagnostic.level ||
              existingCache.diagnostic.message !== diagnostic.message) {
            // New issue or changed issue
            issueCache.current.set(key, {
              diagnostic,
              timestamp: existingCache?.timestamp || now,
              lastSeen: now
            });
          } else {
            // Update last seen time for existing issue
            issueCache.current.set(key, {
              ...existingCache,
              lastSeen: now
            });
          }
        }

        return diagnostic;
      });

      // Mark issues that are no longer being reported but keep them in cache
      issueCache.current.forEach((cached, key) => {
        if (!currentIssueKeys.has(key) && cached.lastSeen === now) {
          // This issue was not in the current message, just update lastSeen to previous value
          // This allows it to expire after ISSUE_CACHE_DURATION
        }
      });

      // Combine current diagnostics with cached issues
      const allDiagnosticsMap = new Map<string, DiagnosticStatus>();

      // Add all current diagnostics
      processedDiagnostics.forEach(d => {
        const key = `${d.name}-${d.hardware_id}`;
        allDiagnosticsMap.set(key, d);
      });

      // Add cached issues that aren't in current diagnostics
      issueCache.current.forEach((cached, key) => {
        if (!allDiagnosticsMap.has(key) && cached.diagnostic.level > 0) {
          allDiagnosticsMap.set(key, cached.diagnostic);
        }
      });

      const mergedDiagnostics = Array.from(allDiagnosticsMap.values());
      setDiagnostics(mergedDiagnostics);
      setLastUpdate(new Date());

      // Update cached issues for display
      const issues = Array.from(issueCache.current.values())
        .filter(c => c.diagnostic.level > 0)
        .map(c => c.diagnostic);
      setCachedIssues(issues);
    }
  }, []);

  // Set up cache cleanup interval
  useEffect(() => {
    const interval = setInterval(cleanupCache, CACHE_CLEANUP_INTERVAL);
    return () => clearInterval(interval);
  }, [cleanupCache]);

  // Subscribe to diagnostics topic
  useEffect(() => {
    if (!diagnosticsTopic) {
      setIsSubscribed(false);
      return;
    }

    try {
      diagnosticsTopic.subscribe((message: any) => {
        handleDiagnosticsMessage(message as DiagnosticArray);
      });
      setIsSubscribed(true);
      console.log('Subscribed to /diagnostics topic');
    } catch (error) {
      console.error('Failed to subscribe to diagnostics topic:', error);
      setIsSubscribed(false);
    }

    return () => {
      try {
        diagnosticsTopic.unsubscribe();
        setIsSubscribed(false);
        console.log('Unsubscribed from /diagnostics topic');
      } catch (error) {
        console.error('Failed to unsubscribe from diagnostics topic:', error);
      }
    };
  }, [diagnosticsTopic, handleDiagnosticsMessage]);

  // Clear data on disconnect
  useEffect(() => {
    if (!connection.online) {
      setDiagnostics([]);
      setLastUpdate(null);
      setIsSubscribed(false);
      issueCache.current.clear();
      setCachedIssues([]);
    }
  }, [connection.online]);

  // Filter diagnostics by level
  const getIssues = useCallback(() => {
    return diagnostics.filter(d => d.level > 0); // Only WARN, ERROR, STALE
  }, [diagnostics]);

  const getErrors = useCallback(() => {
    return diagnostics.filter(d => d.level === 2); // Only ERROR
  }, [diagnostics]);

  const getWarnings = useCallback(() => {
    return diagnostics.filter(d => d.level === 1); // Only WARN
  }, [diagnostics]);

  const getStale = useCallback(() => {
    return diagnostics.filter(d => d.level === 3); // Only STALE
  }, [diagnostics]);

  return {
    diagnostics,
    issues: getIssues(),
    errors: getErrors(),
    warnings: getWarnings(),
    stale: getStale(),
    lastUpdate,
    isConnected: connection.online,
    isSubscribed,
  };
}