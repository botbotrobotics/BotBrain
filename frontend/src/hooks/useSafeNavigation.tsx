'use client';

import { useRouter } from 'next/navigation';
import { useCallback, useRef } from 'react';

export function useSafeNavigation() {
  const router = useRouter();
  const navigationTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  const navigate = useCallback((path: string) => {
    // Clear any pending navigation
    if (navigationTimeoutRef.current) {
      clearTimeout(navigationTimeoutRef.current);
    }

    // Use setTimeout to ensure navigation happens after current render cycle
    navigationTimeoutRef.current = setTimeout(() => {
      router.push(path);
    }, 0);
  }, [router]);

  return { navigate };
}