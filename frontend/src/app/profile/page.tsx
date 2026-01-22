'use client';

import { useState, useEffect } from 'react';
import { useLanguage } from '@/contexts/LanguageContext';
import { Check, Save, LogOut, Key, AlertTriangle, Zap } from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { useTheme } from '@/contexts/ThemeContext';
import { useSpeedMode } from '@/contexts/SpeedModeContext';
import { useRouter } from 'next/navigation';
import dynamic from 'next/dynamic';
import ChangePasswordPopup from '@/components/change-password-popup';
import ColorPicker from '@/components/color-picker';
import { auditLogger } from '@/utils/audit-logger';
import { SpeedMode, SPEED_MODE_MULTIPLIERS, SPEED_MODE_LABELS, SPEED_MODE_DESCRIPTIONS, DEFAULT_SPEED_MODE } from '@/types/speed-mode';

// Dynamically import AvatarUpload with SSR disabled to avoid window.confirm issues
const AvatarUpload = dynamic(
  () => import('@/components/avatar-upload'),
  { 
    ssr: false,
    loading: () => (
      <div className="w-32 h-32 bg-gray-200 dark:bg-botbot-dark animate-pulse rounded-lg"></div>
    )
  }
);

export default function ProfilePage() {
  const { t } = useLanguage();
  const { user, signOut, supabase } = useSupabase();
  const { setThemeColor } = useTheme();
  const { setSpeedMode: setGlobalSpeedMode, reloadSpeedMode } = useSpeedMode();
  const router = useRouter();
  const [name, setName] = useState('');
  const [avatarUrl, setAvatarUrl] = useState<string | null>(null);
  const [localThemeColor, setLocalThemeColor] = useState<string | null>(null);
  const [showPasswordPopup, setShowPasswordPopup] = useState(false);
  const [profileId, setProfileId] = useState<string | null>(null);
  const [hideBranding, setHideBranding] = useState(false);
  const [auditLoggingEnabled, setAuditLoggingEnabled] = useState(true);
  const [speedMode, setSpeedMode] = useState<SpeedMode>(DEFAULT_SPEED_MODE);
  const [showInsaneConfirmation, setShowInsaneConfirmation] = useState(false);

  useEffect(() => {
    // Set page title
    document.title = 'Profile - BotBot';
  }, []);
  
  // Track original values to detect changes
  const [originalName, setOriginalName] = useState('');
  const [originalAvatarUrl, setOriginalAvatarUrl] = useState<string | null>(null);
  const [originalThemeColor, setOriginalThemeColor] = useState<string | null>(null);
  const [originalHideBranding, setOriginalHideBranding] = useState(false);
  const [originalAuditLoggingEnabled, setOriginalAuditLoggingEnabled] = useState(true);
  const [connectionTimeout, setConnectionTimeout] = useState(20000);
  const [originalConnectionTimeout, setOriginalConnectionTimeout] = useState(20000);
  const [originalSpeedMode, setOriginalSpeedMode] = useState<SpeedMode>(DEFAULT_SPEED_MODE);

  // Track if any setting has changed
  const [hasChanges, setHasChanges] = useState(false);
  
  // Track save status for feedback
  const [saveStatus, setSaveStatus] = useState<'idle' | 'saving' | 'saved'>('idle');
  
  // Load user profile when component mounts
  useEffect(() => {
    if (user && supabase) {
      loadProfile();
    }
  }, [user, supabase]);

  // Load profile from database
  const loadProfile = async () => {
    if (!supabase) return;
    
    try {
      // First, try to get the existing profile
      const { data: profile, error } = await supabase
        .from('user_profiles')
        .select('*')
        .eq('user_id', user!.id)
        .single();

      if (error && error.code === 'PGRST116') {
        // Profile doesn't exist, create one
        const { data: newProfile, error: createError } = await supabase
          .from('user_profiles')
          .insert({
            user_id: user!.id,
            name: user!.user_metadata?.name || '',
            avatar_url: null,
            theme_color: null,
            hide_branding: false,
            connection_timeout: 20000
          })
          .select()
          .single();

        if (createError) throw createError;

        if (newProfile) {
          setProfileId(newProfile.id);
          setName(newProfile.name || '');
          setOriginalName(newProfile.name || '');
          setAvatarUrl(newProfile.avatar_url);
          setOriginalAvatarUrl(newProfile.avatar_url);
          setLocalThemeColor(newProfile.theme_color);
          setOriginalThemeColor(newProfile.theme_color);
          setHideBranding(newProfile.hide_branding || false);
          setOriginalHideBranding(newProfile.hide_branding || false);
          setAuditLoggingEnabled(newProfile.audit_logging_enabled ?? true);
          setOriginalAuditLoggingEnabled(newProfile.audit_logging_enabled ?? true);
          setConnectionTimeout(newProfile.connection_timeout || 20000);
          setOriginalConnectionTimeout(newProfile.connection_timeout || 20000);
          const loadedSpeedMode = (newProfile.speed_mode as SpeedMode) || DEFAULT_SPEED_MODE;
          setSpeedMode(loadedSpeedMode);
          setOriginalSpeedMode(loadedSpeedMode);
        }
      } else if (error) {
        throw error;
      } else if (profile) {
        setProfileId(profile.id);
        setName(profile.name || '');
        setOriginalName(profile.name || '');
        setAvatarUrl(profile.avatar_url);
        setOriginalAvatarUrl(profile.avatar_url);
        setLocalThemeColor(profile.theme_color);
        setOriginalThemeColor(profile.theme_color);
        setHideBranding(profile.hide_branding || false);
        setOriginalHideBranding(profile.hide_branding || false);
        setAuditLoggingEnabled(profile.audit_logging_enabled ?? true);
        setOriginalAuditLoggingEnabled(profile.audit_logging_enabled ?? true);
        setConnectionTimeout(profile.connection_timeout || 20000);
        setOriginalConnectionTimeout(profile.connection_timeout || 20000);
        const loadedSpeedMode = (profile.speed_mode as SpeedMode) || DEFAULT_SPEED_MODE;
        setSpeedMode(loadedSpeedMode);
        setOriginalSpeedMode(loadedSpeedMode);
      }
    } catch (error) {
      console.error('Error loading profile:', error);
    }
  };

  // Check for changes whenever any setting is updated
  useEffect(() => {
    const hasChanged = name !== originalName ||
                      avatarUrl !== originalAvatarUrl ||
                      localThemeColor !== originalThemeColor ||
                      hideBranding !== originalHideBranding ||
                      auditLoggingEnabled !== originalAuditLoggingEnabled ||
                      connectionTimeout !== originalConnectionTimeout ||
                      speedMode !== originalSpeedMode;
    setHasChanges(hasChanged);

    // Reset save status when changes are made after saving
    if (hasChanged && saveStatus === 'saved') {
      setSaveStatus('idle');
    }
  }, [name, avatarUrl, localThemeColor, hideBranding, auditLoggingEnabled, connectionTimeout, speedMode, originalName, originalAvatarUrl, originalThemeColor, originalHideBranding, originalAuditLoggingEnabled, originalConnectionTimeout, originalSpeedMode, saveStatus]);

  // Handle name change
  const handleNameChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setName(e.target.value);
  };

  // Handle avatar update
  const handleAvatarUpdate = (url: string | null) => {
    setAvatarUrl(url);
  };
  
  // Handle theme color change
  const handleThemeColorChange = (color: string | null) => {
    setLocalThemeColor(color);
  };

  // Handle speed mode change with confirmation for insane mode
  const handleSpeedModeChange = (mode: SpeedMode) => {
    if (mode === 'insane') {
      setShowInsaneConfirmation(true);
    } else {
      setSpeedMode(mode);
    }
  };

  const confirmInsaneMode = () => {
    setSpeedMode('insane');
    setShowInsaneConfirmation(false);
  };

  const cancelInsaneMode = () => {
    setShowInsaneConfirmation(false);
  };
  
  // Save profile changes
  const handleSave = async () => {
    if (!supabase) return;
    
    setSaveStatus('saving');
    
    try {
      // Update profile in database
      const { error } = await supabase
        .from('user_profiles')
        .update({
          name: name,
          avatar_url: avatarUrl,
          theme_color: localThemeColor,
          hide_branding: hideBranding,
          audit_logging_enabled: auditLoggingEnabled,
          connection_timeout: connectionTimeout,
          speed_mode: speedMode,
          updated_at: new Date().toISOString()
        })
        .eq('user_id', user!.id);
      
      if (error) throw error;

      // Also update user metadata for consistency
      await supabase.auth.updateUser({
        data: { name: name }
      });
      
      // Update original values
      setOriginalName(name);
      setOriginalAvatarUrl(avatarUrl);
      setOriginalThemeColor(localThemeColor);
      setOriginalHideBranding(hideBranding);
      setOriginalAuditLoggingEnabled(auditLoggingEnabled);
      setOriginalConnectionTimeout(connectionTimeout);
      setOriginalSpeedMode(speedMode);

      // Update the global theme
      setThemeColor(localThemeColor);

      // Update the global speed mode and reload from DB
      setGlobalSpeedMode(speedMode);
      await reloadSpeedMode();
      
      // Show saved status briefly
      setTimeout(() => {
        setSaveStatus('saved');
        setTimeout(() => {
          if (!hasChanges) {
            setSaveStatus('idle');
          }
        }, 2000);
      }, 500);

      // Log the update with changes
      const changes: Record<string, any> = {};
      
      if (name !== originalName) {
        changes.name = { from: originalName, to: name };
      }
      if (avatarUrl !== originalAvatarUrl) {
        changes.avatar_url = { from: originalAvatarUrl, to: avatarUrl };
      }
      if (localThemeColor !== originalThemeColor) {
        changes.theme_color = { from: originalThemeColor, to: localThemeColor };
      }
      if (hideBranding !== originalHideBranding) {
        changes.hide_branding = { from: originalHideBranding, to: hideBranding };
      }
      if (auditLoggingEnabled !== originalAuditLoggingEnabled) {
        changes.audit_logging_enabled = { from: originalAuditLoggingEnabled, to: auditLoggingEnabled };
      }
      if (connectionTimeout !== originalConnectionTimeout) {
        changes.connection_timeout = { from: originalConnectionTimeout, to: connectionTimeout };
      }
      if (speedMode !== originalSpeedMode) {
        changes.speed_mode = { from: originalSpeedMode, to: speedMode };
      }

      // Only log if audit logging is still enabled
      if (auditLoggingEnabled) {
        await auditLogger.log({
          event_type: 'system',
          event_action: 'profile_updated',
          event_details: {
            changes,
            profile: {
              name,
              avatar_url: avatarUrl,
              theme_color: localThemeColor,
              hide_branding: hideBranding,
              audit_logging_enabled: auditLoggingEnabled,
              connection_timeout: connectionTimeout,
              speed_mode: speedMode
            }
          }
        });
      }
    } catch (error) {
      console.error('Error updating profile:', error);
      setSaveStatus('idle');
    }
  };
  
  // Handle sign out
  const handleSignOut = async () => {
    await signOut();
    router.push('/');
  };
  
  return (
    <div className="w-full h-[calc(100vh-56px-24px)] overflow-y-auto custom-scrollbar bg-gradient-to-br from-purple-50/30 via-indigo-50/20 to-blue-50/30 dark:from-botbot-darker dark:via-botbot-dark dark:to-botbot-darker">
      <div className="container mx-auto p-4 md:p-6 lg:p-8 max-w-7xl">
        {/* Header Section */}
        <div className="mb-8">
          <div className="flex flex-col sm:flex-row justify-between items-start sm:items-center gap-4">
            <div>
              <h1 className="text-3xl font-bold text-gray-800 dark:text-white mb-2">
                {t('profile', 'title') || 'Profile Settings'}
              </h1>
              <p className="text-gray-600 dark:text-gray-400">
                Manage your account settings and preferences
              </p>
            </div>

            {/* Save Button */}
            <button
              onClick={handleSave}
              disabled={!hasChanges || saveStatus === 'saving'}
              className={`flex items-center justify-center px-6 py-3 rounded-lg text-sm font-medium
                transition-all duration-200 shadow-sm ${
                  hasChanges && saveStatus !== 'saving'
                    ? 'bg-primary dark:bg-botbot-accent text-white hover:bg-primary/90 dark:hover:bg-botbot-accent/90 hover:shadow-md'
                    : 'bg-gray-200 dark:bg-botbot-dark text-gray-500 dark:text-gray-400 cursor-not-allowed'
                }`}
            >
              {saveStatus === 'saving' ? (
                <>
                  <div className="w-4 h-4 border-2 border-white border-t-transparent rounded-full animate-spin mr-2"></div>
                  {t('common', 'saving') || 'Saving...'}
                </>
              ) : saveStatus === 'saved' ? (
                <>
                  <Check className="w-4 h-4 mr-2" />
                  {t('common', 'saved') || 'Saved'}
                </>
              ) : (
                <>
                  <Save className="w-4 h-4 mr-2" />
                  {t('common', 'save') || 'Save Changes'}
                </>
              )}
            </button>
          </div>
        </div>

        {/* Main Grid Layout */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
          {/* Left Column - Personal Information */}
          <div className="space-y-6">
            {/* Avatar and Name Card */}
            <div className="bg-white dark:bg-botbot-darker rounded-xl shadow-sm border border-gray-200 dark:border-botbot-dark p-6">
              <h2 className="text-lg font-semibold text-gray-800 dark:text-white mb-6">
                Personal Information
              </h2>

              {/* Avatar Upload Section */}
              <div className="flex flex-col items-center mb-6">
                {user && (
                  <AvatarUpload
                    avatarUrl={avatarUrl}
                    userId={user.id}
                    onAvatarUpdate={handleAvatarUpdate}
                  />
                )}
              </div>

              {/* Name Field */}
              <div className="space-y-2">
                <label htmlFor="user-name" className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                  {t('profile', 'name') || 'Display Name'}
                </label>
                <input
                  type="text"
                  id="user-name"
                  value={name}
                  onChange={handleNameChange}
                  className="w-full px-4 py-2.5 border border-gray-300 dark:border-botbot-darker rounded-lg
                             focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent
                             bg-white dark:bg-botbot-dark text-gray-800 dark:text-gray-100
                             transition-colors duration-200"
                  placeholder={t('profile', 'namePlaceholder') || 'Enter your name'}
                />
                <p className="text-xs text-gray-500 dark:text-gray-400">
                  {t('profile', 'nameDescription') || 'Your display name in the application'}
                </p>
              </div>
            </div>

            {/* Account Information Card */}
            <div className="bg-white dark:bg-botbot-darker rounded-xl shadow-sm border border-gray-200 dark:border-botbot-dark p-6">
              <h2 className="text-lg font-semibold text-gray-800 dark:text-white mb-6">
                Account Information
              </h2>

              <div className="space-y-4">
                {/* Email Field */}
                <div className="space-y-2">
                  <label htmlFor="user-email" className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                    {t('profile', 'email') || 'Email Address'}
                  </label>
                  <input
                    type="email"
                    id="user-email"
                    value={user?.email || ''}
                    disabled
                    className="w-full px-4 py-2.5 border border-gray-200 dark:border-botbot-darker rounded-lg
                               bg-gray-50 dark:bg-botbot-dark/50 text-gray-600 dark:text-gray-400
                               cursor-not-allowed"
                  />
                  <p className="text-xs text-gray-500 dark:text-gray-400">
                    {t('profile', 'emailDescription') || 'Your email address cannot be changed'}
                  </p>
                </div>

                {/* User ID Field */}
                <div className="space-y-2">
                  <label htmlFor="user-id" className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                    {t('profile', 'userId') || 'User ID'}
                  </label>
                  <input
                    type="text"
                    id="user-id"
                    value={user?.id || ''}
                    disabled
                    className="w-full px-4 py-2.5 border border-gray-200 dark:border-botbot-darker rounded-lg
                               bg-gray-50 dark:bg-botbot-dark/50 text-gray-600 dark:text-gray-400
                               cursor-not-allowed font-mono text-sm"
                  />
                  <p className="text-xs text-gray-500 dark:text-gray-400">
                    {t('profile', 'userIdDescription') || 'Your unique user identifier'}
                  </p>
                </div>
              </div>
            </div>

            {/* Security Actions Card */}
            <div className="bg-white dark:bg-botbot-darker rounded-xl shadow-sm border border-gray-200 dark:border-botbot-dark p-6">
              <h2 className="text-lg font-semibold text-gray-800 dark:text-white mb-6">
                Security
              </h2>

              <div className="space-y-3">
                {/* Change Password Button */}
                <button
                  onClick={() => setShowPasswordPopup(true)}
                  className="w-full flex items-center justify-center px-4 py-3 rounded-lg text-sm font-medium
                             bg-gray-100 dark:bg-botbot-dark text-gray-700 dark:text-gray-300
                             hover:bg-gray-200 dark:hover:bg-botbot-dark/80
                             transition-all duration-200 border border-gray-200 dark:border-botbot-darker"
                >
                  <Key className="w-4 h-4 mr-2" />
                  {t('profile', 'changePassword') || 'Change Password'}
                </button>

                {/* Sign Out Button */}
                <button
                  onClick={handleSignOut}
                  className="w-full flex items-center justify-center px-4 py-3 rounded-lg text-sm font-medium
                             bg-red-50 dark:bg-red-900/20 text-red-600 dark:text-red-400
                             hover:bg-red-100 dark:hover:bg-red-900/30
                             border border-red-200 dark:border-red-900/50
                             transition-all duration-200"
                >
                  <LogOut className="w-4 h-4 mr-2" />
                  {t('userProfile', 'logout') || 'Sign Out'}
                </button>
              </div>
            </div>
          </div>

          {/* Right Column - Preferences */}
          <div className="space-y-6">
            {/* Appearance Settings Card */}
            <div className="bg-white dark:bg-botbot-darker rounded-xl shadow-sm border border-gray-200 dark:border-botbot-dark p-6">
              <h2 className="text-lg font-semibold text-gray-800 dark:text-white mb-6">
                Appearance
              </h2>

              <div className="space-y-6">
                {/* Theme Color Picker */}
                <div>
                  <ColorPicker
                    value={localThemeColor}
                    onChange={handleThemeColorChange}
                  />
                </div>

                {/* Hide Branding Toggle */}
                <div className="space-y-2">
                  <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-3">
                    Branding Options
                  </label>
                  <div className="flex items-center justify-between p-4 bg-gray-50 dark:bg-botbot-dark/50 rounded-lg">
                    <div className="flex-1 mr-4">
                      <p className="text-sm font-medium text-gray-700 dark:text-gray-200">
                        Hide BotBot Logo
                      </p>
                      <p className="text-xs text-gray-500 dark:text-gray-400 mt-1">
                        Remove the BotBot logo from the navigation bar
                      </p>
                    </div>
                    <button
                      type="button"
                      onClick={() => setHideBranding(!hideBranding)}
                      className={`relative inline-flex h-6 w-11 items-center rounded-full transition-colors focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent focus:ring-offset-2 ${
                        hideBranding ? 'bg-primary dark:bg-botbot-accent' : 'bg-gray-200 dark:bg-botbot-darker'
                      }`}
                    >
                      <span
                        className={`inline-block h-4 w-4 transform rounded-full bg-white transition-transform ${
                          hideBranding ? 'translate-x-6' : 'translate-x-1'
                        }`}
                      />
                    </button>
                  </div>
                </div>
              </div>
            </div>

            {/* System Settings Card */}
            <div className="bg-white dark:bg-botbot-darker rounded-xl shadow-sm border border-gray-200 dark:border-botbot-dark p-6">
              <h2 className="text-lg font-semibold text-gray-800 dark:text-white mb-6">
                System Settings
              </h2>

              <div className="space-y-6">
                {/* Audit Logging Toggle */}
                <div className="space-y-2">
                  <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-3">
                    Privacy & Logging
                  </label>
                  <div className="flex items-center justify-between p-4 bg-gray-50 dark:bg-botbot-dark/50 rounded-lg">
                    <div className="flex-1 mr-4">
                      <p className="text-sm font-medium text-gray-700 dark:text-gray-200">
                        Audit Logging
                      </p>
                      <p className="text-xs text-gray-500 dark:text-gray-400 mt-1">
                        Log all user actions for security and analytics purposes
                      </p>
                    </div>
                    <button
                      type="button"
                      onClick={() => setAuditLoggingEnabled(!auditLoggingEnabled)}
                      className={`relative inline-flex h-6 w-11 items-center rounded-full transition-colors focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent focus:ring-offset-2 ${
                        auditLoggingEnabled ? 'bg-primary dark:bg-botbot-accent' : 'bg-gray-200 dark:bg-botbot-darker'
                      }`}
                    >
                      <span
                        className={`inline-block h-4 w-4 transform rounded-full bg-white transition-transform ${
                          auditLoggingEnabled ? 'translate-x-6' : 'translate-x-1'
                        }`}
                      />
                    </button>
                  </div>
                </div>

                {/* Connection Timeout Setting */}
                <div className="space-y-2">
                  <label htmlFor="connection-timeout" className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-3">
                    Robot Connection
                  </label>
                  <div className="p-4 bg-gray-50 dark:bg-botbot-dark/50 rounded-lg">
                    <div className="flex items-center justify-between mb-3">
                      <p className="text-sm font-medium text-gray-700 dark:text-gray-200">
                        Connection Timeout
                      </p>
                      <div className="flex items-center space-x-2">
                        <input
                          type="number"
                          id="connection-timeout"
                          min="1"
                          max="120"
                          step="1"
                          value={Math.round(connectionTimeout / 1000)}
                          onChange={(e) => {
                            const value = parseInt(e.target.value);
                            if (!isNaN(value) && value >= 1 && value <= 120) {
                              setConnectionTimeout(value * 1000);
                            }
                          }}
                          className="w-20 px-3 py-1.5 border border-gray-300 dark:border-botbot-darker rounded-md
                                     focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent
                                     bg-white dark:bg-botbot-dark text-gray-800 dark:text-gray-100
                                     text-sm text-center"
                        />
                        <span className="text-sm text-gray-600 dark:text-gray-400">seconds</span>
                      </div>
                    </div>
                    <p className="text-xs text-gray-500 dark:text-gray-400">
                      Time to wait for robot connection before timing out (1-120 seconds, default: 20)
                    </p>
                  </div>
                </div>
              </div>
            </div>

            {/* Speed & Control Card */}
            <div className="bg-white dark:bg-botbot-darker rounded-xl shadow-sm border border-gray-200 dark:border-botbot-dark p-6">
              <h2 className="text-lg font-semibold text-gray-800 dark:text-white mb-6">
                Speed & Control
              </h2>

              <div className="space-y-4">
                <label className="block text-sm font-medium text-gray-700 dark:text-gray-300">
                  Speed Mode
                </label>

                {/* Segmented Control */}
                <div className="flex rounded-lg bg-gray-100 dark:bg-botbot-dark/50 p-1">
                  {(['beginner', 'normal', 'insane'] as const).map((mode) => (
                    <button
                      key={mode}
                      type="button"
                      onClick={() => handleSpeedModeChange(mode)}
                      className={`flex-1 px-4 py-2.5 text-sm font-medium rounded-md transition-all duration-200
                        ${speedMode === mode
                          ? 'bg-white dark:bg-botbot-accent text-gray-900 dark:text-white shadow-sm'
                          : 'text-gray-600 dark:text-gray-400 hover:text-gray-900 dark:hover:text-gray-200'
                        }`}
                    >
                      {SPEED_MODE_LABELS[mode]}
                    </button>
                  ))}
                </div>

                {/* Description for selected mode */}
                <p className="text-xs text-gray-500 dark:text-gray-400">
                  {SPEED_MODE_DESCRIPTIONS[speedMode]}
                </p>

                {/* Visual indicator */}
                <div className="p-4 bg-gray-50 dark:bg-botbot-dark/50 rounded-lg">
                  <div className="flex items-center justify-between mb-2">
                    <span className="text-sm text-gray-600 dark:text-gray-400">Speed Level</span>
                    <span className="text-sm font-medium text-gray-800 dark:text-gray-200">
                      {Math.round(SPEED_MODE_MULTIPLIERS[speedMode] * 100)}%
                    </span>
                  </div>
                  <div className="w-full bg-gray-200 dark:bg-botbot-darker rounded-full h-2">
                    <div
                      className="bg-primary dark:bg-botbot-accent h-2 rounded-full transition-all duration-300"
                      style={{ width: `${SPEED_MODE_MULTIPLIERS[speedMode] * 100}%` }}
                    />
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Change Password Popup */}
      <ChangePasswordPopup
        isOpen={showPasswordPopup}
        onClose={() => setShowPasswordPopup(false)}
      />

      {/* Insane Mode Confirmation Dialog */}
      <AnimatePresence>
        {showInsaneConfirmation && (
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            transition={{ duration: 0.2 }}
            className="fixed inset-0 bg-black/20 dark:bg-black/40 backdrop-blur-sm z-[90]
                       flex items-center justify-center"
            onClick={cancelInsaneMode}
          >
            <motion.div
              initial={{ opacity: 0, scale: 0.95 }}
              animate={{ opacity: 1, scale: 1 }}
              exit={{ opacity: 0, scale: 0.95 }}
              transition={{
                type: 'spring',
                damping: 25,
                stiffness: 300,
              }}
              className="bg-white dark:bg-botbot-dark shadow-lg rounded-lg p-6
                         min-w-[300px] max-w-[400px] mx-4"
              onClick={(e) => e.stopPropagation()}
            >
              {/* Warning icon */}
              <div className="flex justify-center mb-4">
                <div className="p-3 rounded-full bg-red-100 dark:bg-red-900/30">
                  <Zap className="w-6 h-6 text-red-600 dark:text-red-400" />
                </div>
              </div>

              {/* Title */}
              <h2 className="text-lg font-semibold text-center text-gray-900 dark:text-white mb-2">
                Enable Insane Mode?
              </h2>

              {/* Message */}
              <p className="text-sm text-center text-gray-600 dark:text-gray-300 mb-2">
                This will set robot speed to <span className="font-semibold text-red-600 dark:text-red-400">100%</span> of maximum velocity.
              </p>
              <p className="text-xs text-center text-gray-500 dark:text-gray-400 mb-6">
                Only recommended for experienced operators in controlled environments.
              </p>

              {/* Buttons */}
              <div className="flex gap-3">
                <button
                  onClick={cancelInsaneMode}
                  className="flex-1 py-2.5 px-4 rounded-lg border border-gray-300 dark:border-gray-600
                             text-gray-700 dark:text-gray-200 font-medium
                             hover:bg-gray-100 dark:hover:bg-botbot-darker
                             transition-colors focus:outline-none focus:ring-2 focus:ring-gray-300 dark:focus:ring-gray-600"
                >
                  Cancel
                </button>
                <button
                  onClick={confirmInsaneMode}
                  className="flex-1 py-2.5 px-4 rounded-lg
                             bg-red-500 hover:bg-red-600
                             text-white font-medium
                             transition-colors focus:outline-none focus:ring-2 focus:ring-red-400"
                >
                  Enable Insane
                </button>
              </div>
            </motion.div>
          </motion.div>
        )}
      </AnimatePresence>
    </div>
  );
} 