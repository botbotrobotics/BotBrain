'use client';

import { useEffect } from 'react';
import { useRouter } from 'next/navigation';
import { useSupabase } from '@/contexts/SupabaseProvider';

interface ProtectedRouteProps {
  children: React.ReactNode;
}

export default function ProtectedRoute({ children }: ProtectedRouteProps) {
  const router = useRouter();
  const { user, loading } = useSupabase();

  useEffect(() => {
    // Wait for auth to finish loading
    if (loading) {
      return;
    }

    // Redirect to login if not authenticated
    if (!user) {
      router.push('/');
    }
  }, [user, loading, router]);

  // Show loading state while checking auth
  if (loading) {
    return (
      <div className="w-full h-full flex items-center justify-center">
        <div className="text-gray-600 dark:text-gray-400">
          <div className="flex items-center gap-2">
            <div className="w-4 h-4 border-2 border-primary dark:border-botbot-accent border-t-transparent rounded-full animate-spin"></div>
            Loading...
          </div>
        </div>
      </div>
    );
  }

  // Don't render children until we confirm user is authenticated
  if (!user) {
    return null;
  }

  return <>{children}</>;
} 