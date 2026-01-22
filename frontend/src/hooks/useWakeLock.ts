'use client';

import { useEffect, useRef } from 'react';

export function useWakeLock() {
  const wakeLockRef = useRef<WakeLockSentinel | null>(null);

  useEffect(() => {
    const requestWakeLock = async () => {
      try {
        if ('wakeLock' in navigator) {
          wakeLockRef.current = await navigator.wakeLock.request('screen');
          console.log('Wake Lock activated');

          wakeLockRef.current.addEventListener('release', () => {
            console.log('Wake Lock released');
          });
        }
      } catch (err) {
        console.error('Wake Lock request failed:', err);
      }
    };

    const handleVisibilityChange = () => {
      if (wakeLockRef.current && document.visibilityState === 'visible') {
        requestWakeLock();
      }
    };

    requestWakeLock();
    document.addEventListener('visibilitychange', handleVisibilityChange);

    return () => {
      document.removeEventListener('visibilitychange', handleVisibilityChange);

      if (wakeLockRef.current) {
        wakeLockRef.current.release().catch((err) => {
          console.error('Wake Lock release failed:', err);
        });
        wakeLockRef.current = null;
      }
    };
  }, []);
}