'use client';

import { useEffect, useState } from 'react';
import { User, Calendar, Clock, Activity, TrendingUp, Shield, Sparkles } from 'lucide-react';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { Database } from '@/types/database.types';

type UserProfile = Database['public']['Tables']['user_profiles']['Row'];

export default function WelcomeSection() {
  const { user, supabase } = useSupabase();
  const [userProfile, setUserProfile] = useState<UserProfile | null>(null);
  const [currentTime, setCurrentTime] = useState(new Date());
  const [lastSeen, setLastSeen] = useState<string>('');
  const [weeklyActivity, setWeeklyActivity] = useState<number>(0);
  const [streak, setStreak] = useState<number>(0);
  const [memberSince, setMemberSince] = useState<string>('');

  useEffect(() => {
    const fetchUserProfile = async () => {
      if (!user || !supabase) return;

      try {
        const { data: profile } = await supabase
          .from('user_profiles')
          .select('*')
          .eq('user_id', user.id)
          .single();

        if (profile) {
          setUserProfile(profile);
        }

        // Fetch last activity
        const { data: logs } = await supabase
          .from('audit_logs')
          .select('created_at')
          .eq('user_id', user.id)
          .order('created_at', { ascending: false })
          .limit(1);

        if (logs && logs.length > 0) {
          const lastActivity = new Date(logs[0].created_at);
          const now = new Date();
          const diffMs = now.getTime() - lastActivity.getTime();
          const diffMins = Math.floor(diffMs / 60000);

          if (diffMins < 1) setLastSeen('Just now');
          else if (diffMins < 60) setLastSeen(`${diffMins}m ago`);
          else if (diffMins < 1440) setLastSeen(`${Math.floor(diffMins / 60)}h ago`);
          else setLastSeen(`${Math.floor(diffMins / 1440)}d ago`);
        }

        // Fetch weekly activity count
        const oneWeekAgo = new Date();
        oneWeekAgo.setDate(oneWeekAgo.getDate() - 7);

        const { count: weeklyCount } = await supabase
          .from('audit_logs')
          .select('*', { count: 'exact', head: true })
          .eq('user_id', user.id)
          .gte('created_at', oneWeekAgo.toISOString());

        if (weeklyCount) {
          setWeeklyActivity(weeklyCount);
        }

        // Calculate actual activity streak
        const { data: streakLogs } = await supabase
          .from('audit_logs')
          .select('created_at')
          .eq('user_id', user.id)
          .order('created_at', { ascending: false })
          .limit(365); // Check up to a year of activity

        if (streakLogs && streakLogs.length > 0) {
          // Get unique days with activity
          const activityDays = new Set<string>();
          streakLogs.forEach(log => {
            activityDays.add(new Date(log.created_at).toDateString());
          });

          // Convert to sorted array of dates
          const sortedDays = Array.from(activityDays)
            .map(dateStr => new Date(dateStr))
            .sort((a, b) => b.getTime() - a.getTime());

          // Calculate consecutive day streak starting from today or most recent activity
          let currentStreak = 0;
          const today = new Date();
          today.setHours(0, 0, 0, 0);

          if (sortedDays.length > 0) {
            const mostRecent = new Date(sortedDays[0]);
            mostRecent.setHours(0, 0, 0, 0);

            // Check if there's activity today or yesterday to have an active streak
            const daysSinceLastActivity = Math.floor((today.getTime() - mostRecent.getTime()) / (1000 * 60 * 60 * 24));

            if (daysSinceLastActivity <= 1) {
              currentStreak = 1;
              let checkDate = new Date(mostRecent);

              for (let i = 1; i < sortedDays.length; i++) {
                checkDate.setDate(checkDate.getDate() - 1);
                const dayStr = checkDate.toDateString();

                if (activityDays.has(dayStr)) {
                  currentStreak++;
                } else {
                  break;
                }
              }
            }
          }

          setStreak(currentStreak);
        } else {
          setStreak(0);
        }

        // Calculate member since from profile created_at
        if (profile && profile.created_at) {
          const createdDate = new Date(profile.created_at);
          const now = new Date();
          const diffMs = now.getTime() - createdDate.getTime();
          const diffDays = Math.floor(diffMs / (1000 * 60 * 60 * 24));
          const diffMonths = Math.floor(diffDays / 30);
          const diffYears = Math.floor(diffDays / 365);

          if (diffYears > 0) {
            setMemberSince(`${diffYears} ${diffYears === 1 ? 'year' : 'years'}`);
          } else if (diffMonths > 0) {
            setMemberSince(`${diffMonths} ${diffMonths === 1 ? 'month' : 'months'}`);
          } else if (diffDays > 7) {
            const weeks = Math.floor(diffDays / 7);
            setMemberSince(`${weeks} ${weeks === 1 ? 'week' : 'weeks'}`);
          } else if (diffDays > 0) {
            setMemberSince(`${diffDays} ${diffDays === 1 ? 'day' : 'days'}`);
          } else {
            setMemberSince('Today');
          }
        }
      } catch (error) {
        console.error('Error fetching user data:', error);
      }
    };

    fetchUserProfile();
  }, [user, supabase]);

  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentTime(new Date());
    }, 1000);

    return () => clearInterval(timer);
  }, []);

  const getGreeting = () => {
    const hour = currentTime.getHours();
    if (hour < 12) return 'Good Morning';
    if (hour < 17) return 'Good Afternoon';
    return 'Good Evening';
  };

  const formatDate = () => {
    return currentTime.toLocaleDateString('en-US', {
      weekday: 'short',
      month: 'short',
      day: 'numeric',
    });
  };

  const formatTime = () => {
    return currentTime.toLocaleTimeString('en-US', {
      hour: '2-digit',
      minute: '2-digit',
      hour12: false,
    });
  };

  return (
    <div className="relative overflow-hidden rounded-3xl bg-gradient-to-br from-violet-600 via-purple-600 to-indigo-700 dark:from-violet-900 dark:via-purple-900 dark:to-indigo-950 p-8 text-white shadow-2xl">
      {/* Animated gradient mesh background */}
      <div className="absolute inset-0">
        <div className="absolute inset-0 bg-gradient-to-br from-violet-600/20 via-transparent to-purple-600/20 animate-gradient-flow" />
        <div className="absolute top-0 left-1/4 h-64 w-64 animate-float-slow rounded-full bg-purple-500/30 blur-3xl" />
        <div className="absolute bottom-0 right-1/4 h-80 w-80 animate-float-slower rounded-full bg-indigo-500/30 blur-3xl" />
        <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 h-96 w-96 animate-pulse rounded-full bg-violet-500/20 blur-3xl" />
      </div>

      {/* Content */}
      <div className="relative z-10">
        <div className="flex flex-col lg:flex-row items-start lg:items-center justify-between gap-6">
          {/* User Info */}
          <div className="flex items-start gap-5">
            <div className="relative">
              <div className="relative group">
                <div className="absolute -inset-1 bg-gradient-to-r from-purple-400 to-pink-400 rounded-2xl blur opacity-50 group-hover:opacity-75 transition duration-500"></div>
                <div className="relative flex h-16 w-16 items-center justify-center rounded-2xl bg-gradient-to-br from-white/30 to-white/10 backdrop-blur-md border border-white/30 shadow-lg">
                  {userProfile?.avatar_url ? (
                    <img
                      src={userProfile.avatar_url}
                      alt="User Avatar"
                      className="h-14 w-14 rounded-xl object-cover"
                    />
                  ) : (
                    <User className="h-8 w-8 text-white" />
                  )}
                </div>
              </div>
              {/* Online indicator with pulse */}
              <div className="absolute -bottom-1 -right-1">
                <div className="relative">
                  <div className="absolute inset-0 h-5 w-5 rounded-full bg-green-400 animate-ping opacity-75" />
                  <div className="relative h-5 w-5 rounded-full bg-green-400 border-2 border-white shadow-lg" />
                </div>
              </div>
            </div>
            <div className="flex-1">
              <h1 className="text-3xl font-bold bg-clip-text text-transparent bg-gradient-to-r from-white to-purple-200">
                {getGreeting()}, {userProfile?.name || user?.email?.split('@')[0] || 'User'}!
              </h1>
              <p className="mt-2 text-white/90">Welcome back to BotBrain</p>

              {/* User stats in a row */}
              <div className="flex flex-wrap items-center gap-4 mt-4">
                <div className="flex items-center gap-2 px-3 py-1.5 rounded-full bg-white/10 backdrop-blur-sm border border-white/20">
                  <Activity className="h-3.5 w-3.5 text-green-300" />
                  <span className="text-xs font-medium">Active {lastSeen || 'Now'}</span>
                </div>
                <div className="flex items-center gap-2 px-3 py-1.5 rounded-full bg-white/10 backdrop-blur-sm border border-white/20">
                  <TrendingUp className="h-3.5 w-3.5 text-blue-300" />
                  <span className="text-xs font-medium">{weeklyActivity} actions this week</span>
                </div>
                {streak > 0 && (
                  <div className="flex items-center gap-2 px-3 py-1.5 rounded-full bg-white/10 backdrop-blur-sm border border-white/20">
                    <Sparkles className="h-3.5 w-3.5 text-yellow-300" />
                    <span className="text-xs font-medium">{streak} day{streak !== 1 ? 's' : ''} streak</span>
                  </div>
                )}
                <div className="flex items-center gap-2 px-3 py-1.5 rounded-full bg-white/10 backdrop-blur-sm border border-white/20">
                  <Shield className="h-3.5 w-3.5 text-purple-300" />
                  <span className="text-xs font-medium">User for {memberSince || '0 days'}</span>
                </div>
              </div>
            </div>
          </div>

          {/* Time & Date Cards */}
          <div className="flex flex-col gap-3">
            <div className="relative group">
              <div className="absolute -inset-0.5 bg-gradient-to-r from-purple-300/50 to-pink-300/50 rounded-xl blur opacity-0 group-hover:opacity-100 transition duration-500"></div>
              <div className="relative flex items-center gap-3 px-4 py-2.5 rounded-xl bg-white/10 backdrop-blur-md border border-white/20 hover:bg-white/15 transition-all">
                <Clock className="h-5 w-5 text-white/90" />
                <div>
                  <p className="text-xs text-white/70">Current Time</p>
                  <p className="font-mono font-bold text-lg">{formatTime()}</p>
                </div>
              </div>
            </div>
            <div className="relative group">
              <div className="absolute -inset-0.5 bg-gradient-to-r from-purple-300/50 to-pink-300/50 rounded-xl blur opacity-0 group-hover:opacity-100 transition duration-500"></div>
              <div className="relative flex items-center gap-3 px-4 py-2.5 rounded-xl bg-white/10 backdrop-blur-md border border-white/20 hover:bg-white/15 transition-all">
                <Calendar className="h-5 w-5 text-white/90" />
                <div>
                  <p className="text-xs text-white/70">Today's Date</p>
                  <p className="font-medium">{formatDate()}</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}