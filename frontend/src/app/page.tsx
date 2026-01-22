'use client';

import { useEffect } from 'react';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { useRouter } from 'next/navigation';
import Login from '@/components/login';

// Force dynamic rendering to prevent static generation issues
export const dynamic = 'force-dynamic';

export default function Home() {
  const { user, loading } = useSupabase();
  const router = useRouter();

  useEffect(() => {
    // Set page title
    document.title = 'Home - BotBot';
  }, []);

  useEffect(() => {
    // If user is already logged in, redirect to dashboard
    if (!loading && user) {
      router.push('/dashboard');
    }
  }, [user, loading, router]);

  // Show loading while checking auth
  if (loading) {
    return (
      <div className="w-full h-screen flex items-center justify-center">
        <div className="text-gray-600 dark:text-gray-400">
          <div className="flex items-center gap-2">
            <div className="w-4 h-4 border-2 border-primary dark:border-botbot-accent border-t-transparent rounded-full animate-spin"></div>
            Loading...
          </div>
        </div>
      </div>
    );
  }

  // Show login only if no user
  if (!user) {
    return (
      <div>
        <Login />
      </div>
    );
  }

  // Return null while redirecting
  return null;
}