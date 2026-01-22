'use client';

import { useSupabase } from '@/contexts/SupabaseProvider';
import { Database } from '@/types/database.types';
import { useState, useEffect } from 'react';

type UserProfile = Database['public']['Tables']['user_profiles']['Row'];

export function useUserSettings() {
  const { user, supabase } = useSupabase();
  const [userProfile, setUserProfile] = useState<UserProfile | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Default connection timeout (20 seconds)
  const DEFAULT_CONNECTION_TIMEOUT = 20000;

  // Load user profile
  useEffect(() => {
    if (user && supabase) {
      loadUserProfile();
    }
  }, [user, supabase]);

  const loadUserProfile = async () => {
    if (!supabase || !user) return;

    setLoading(true);
    setError(null);

    try {
      const { data, error: fetchError } = await supabase
        .from('user_profiles')
        .select('*')
        .eq('user_id', user.id)
        .single();

      if (fetchError) {
        if (fetchError.code === 'PGRST116') {
          // Profile doesn't exist, create one with defaults
          const { data: newProfile, error: createError } = await supabase
            .from('user_profiles')
            .insert({
              user_id: user.id,
              name: user.user_metadata?.name || '',
              connection_timeout: DEFAULT_CONNECTION_TIMEOUT,
            })
            .select()
            .single();

          if (createError) throw createError;
          setUserProfile(newProfile);
        } else {
          throw fetchError;
        }
      } else {
        setUserProfile(data);
      }
    } catch (err) {
      console.error('Error loading user profile:', err);
      setError(err instanceof Error ? err.message : 'Unknown error');
    } finally {
      setLoading(false);
    }
  };

  const updateConnectionTimeout = async (timeout: number) => {
    if (!supabase || !user) {
      throw new Error('Not authenticated');
    }

    try {
      const { error: updateError } = await supabase
        .from('user_profiles')
        .update({
          connection_timeout: timeout,
          updated_at: new Date().toISOString(),
        })
        .eq('user_id', user.id);

      if (updateError) throw updateError;

      // Update local state
      setUserProfile(prev => prev ? { ...prev, connection_timeout: timeout } : null);

      return true;
    } catch (err) {
      console.error('Error updating connection timeout:', err);
      throw err;
    }
  };

  return {
    userProfile,
    connectionTimeout: userProfile?.connection_timeout || DEFAULT_CONNECTION_TIMEOUT,
    loading,
    error,
    updateConnectionTimeout,
    reloadProfile: loadUserProfile,
  };
}