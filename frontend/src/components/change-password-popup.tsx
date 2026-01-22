'use client';

import React, { useState, useEffect } from 'react';
import { Eye, EyeOff, Check, X, AlertCircle, Loader2 } from 'lucide-react';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { useLanguage } from '@/contexts/LanguageContext';
import { auditLogger } from '@/utils/audit-logger';

interface ChangePasswordPopupProps {
  isOpen: boolean;
  onClose: () => void;
}

interface PasswordStrength {
  score: number;
  label: string;
  color: string;
}

export default function ChangePasswordPopup({
  isOpen,
  onClose,
}: ChangePasswordPopupProps) {
  const { supabase } = useSupabase();
  const { t } = useLanguage();
  const [newPassword, setNewPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [showNewPassword, setShowNewPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState(false);
  const [loading, setLoading] = useState(false);
  const [passwordStrength, setPasswordStrength] = useState<PasswordStrength>({
    score: 0,
    label: '',
    color: '',
  });

  // Reset states when popup closes
  useEffect(() => {
    if (!isOpen) {
      setTimeout(() => {
        setNewPassword('');
        setConfirmPassword('');
        setError('');
        setSuccess(false);
        setShowNewPassword(false);
        setShowConfirmPassword(false);
        setPasswordStrength({ score: 0, label: '', color: '' });
      }, 300);
    }
  }, [isOpen]);

  // Calculate password strength
  useEffect(() => {
    if (!newPassword) {
      setPasswordStrength({ score: 0, label: '', color: '' });
      return;
    }

    let score = 0;
    const checks = {
      length: newPassword.length >= 8,
      lowercase: /[a-z]/.test(newPassword),
      uppercase: /[A-Z]/.test(newPassword),
      number: /[0-9]/.test(newPassword),
      special: /[^A-Za-z0-9]/.test(newPassword),
    };

    score = Object.values(checks).filter(Boolean).length;

    const strengthLevels: PasswordStrength[] = [
      { score: 1, label: t('profile', 'passwordVeryWeak') || 'Very Weak', color: 'bg-red-500' },
      { score: 2, label: t('profile', 'passwordWeak') || 'Weak', color: 'bg-orange-500' },
      { score: 3, label: t('profile', 'passwordFair') || 'Fair', color: 'bg-yellow-500' },
      { score: 4, label: t('profile', 'passwordGood') || 'Good', color: 'bg-green-500' },
      { score: 5, label: t('profile', 'passwordStrong') || 'Strong', color: 'bg-green-600' },
    ];

    const strength = strengthLevels[Math.min(score - 1, 4)] || strengthLevels[0];
    setPasswordStrength(strength);
  }, [newPassword, t]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setSuccess(false);

    if (!supabase) {
      setError('Database connection not available');
      return;
    }

    // Validation
    if (!newPassword || !confirmPassword) {
      setError(t('profile', 'passwordFieldsRequired') || 'All fields are required');
      return;
    }

    if (newPassword !== confirmPassword) {
      setError(t('profile', 'passwordsDoNotMatch') || 'Passwords do not match');
      return;
    }

    if (newPassword.length < 8) {
      setError(t('profile', 'passwordTooShort') || 'Password must be at least 8 characters');
      return;
    }

    setLoading(true);

    try {
      // Update password using Supabase
      const { error: updateError } = await supabase.auth.updateUser({
        password: newPassword,
      });

      if (updateError) {
        setError(updateError.message);
        return;
      }

      // Log password change
      await auditLogger.log({
        event_type: 'system',
        event_action: 'settings_updated',
        event_details: {
          setting: 'password',
          password_strength: passwordStrength.label,
          password_score: passwordStrength.score
        }
      });

      setSuccess(true);
      setTimeout(() => {
        onClose();
      }, 2000);
    } catch (err) {
      setError(t('profile', 'passwordUpdateError') || 'Failed to update password');
    } finally {
      setLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <>
      {/* Overlay */}
      <div 
        className="fixed inset-0 bg-black/20 dark:bg-black/40 backdrop-blur-sm z-[90]"
        onClick={onClose}
      />
      
      {/* Popup */}
      <div className="fixed top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 bg-white dark:bg-botbot-dark shadow-lg rounded-lg p-6 z-[100] w-full max-w-md">
        {/* Header */}
        <div className="flex justify-between items-center mb-4">
          <h2 className="text-xl font-semibold text-gray-800 dark:text-white">
            {t('profile', 'changePassword') || 'Change Password'}
          </h2>
          <button
            onClick={onClose}
            className="p-1 hover:bg-gray-100 dark:hover:bg-botbot-darker rounded-full transition-colors"
          >
            <X className="w-5 h-5 text-gray-500 dark:text-gray-400" />
          </button>
        </div>

        {/* Success Message */}
        {success && (
          <div className="mb-4 p-3 bg-green-100 dark:bg-green-900/20 text-green-700 dark:text-green-400 rounded-md flex items-center">
            <Check className="w-4 h-4 mr-2" />
            {t('profile', 'passwordUpdated') || 'Password updated successfully!'}
          </div>
        )}

        {/* Error Message */}
        {error && (
          <div className="mb-4 p-3 bg-red-100 dark:bg-red-900/20 text-red-700 dark:text-red-400 rounded-md flex items-center">
            <AlertCircle className="w-4 h-4 mr-2" />
            {error}
          </div>
        )}

        {/* Form */}
        <form onSubmit={handleSubmit} className="space-y-4">
          {/* New Password */}
          <div>
            <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
              {t('profile', 'newPassword') || 'New Password'}
            </label>
            <div className="relative">
              <input
                type={showNewPassword ? 'text' : 'password'}
                value={newPassword}
                onChange={(e) => setNewPassword(e.target.value)}
                className="w-full p-2 pr-10 border border-gray-300 dark:border-botbot-darker rounded-md 
                           focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent
                           bg-white dark:bg-botbot-darker text-gray-800 dark:text-gray-100"
                placeholder="••••••••"
              />
              <button
                type="button"
                onClick={() => setShowNewPassword(!showNewPassword)}
                className="absolute right-2 top-1/2 -translate-y-1/2 p-1 hover:bg-gray-100 dark:hover:bg-botbot-dark rounded"
              >
                {showNewPassword ? (
                  <EyeOff className="w-4 h-4 text-gray-500" />
                ) : (
                  <Eye className="w-4 h-4 text-gray-500" />
                )}
              </button>
            </div>
            
            {/* Password Strength Indicator */}
            {newPassword && (
              <div className="mt-2">
                <div className="flex justify-between items-center mb-1">
                  <span className="text-xs text-gray-500 dark:text-gray-400">
                    {t('profile', 'passwordStrength') || 'Password Strength'}
                  </span>
                  <span className={`text-xs font-medium ${
                    passwordStrength.score <= 2 ? 'text-red-500' : 
                    passwordStrength.score === 3 ? 'text-yellow-500' : 
                    'text-green-500'
                  }`}>
                    {passwordStrength.label}
                  </span>
                </div>
                <div className="w-full bg-gray-200 dark:bg-botbot-darker rounded-full h-2">
                  <div
                    className={`h-2 rounded-full transition-all duration-300 ${passwordStrength.color}`}
                    style={{ width: `${(passwordStrength.score / 5) * 100}%` }}
                  />
                </div>
              </div>
            )}
          </div>

          {/* Confirm Password */}
          <div>
            <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
              {t('profile', 'confirmPassword') || 'Confirm Password'}
            </label>
            <div className="relative">
              <input
                type={showConfirmPassword ? 'text' : 'password'}
                value={confirmPassword}
                onChange={(e) => setConfirmPassword(e.target.value)}
                className="w-full p-2 pr-10 border border-gray-300 dark:border-botbot-darker rounded-md 
                           focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent
                           bg-white dark:bg-botbot-darker text-gray-800 dark:text-gray-100"
                placeholder="••••••••"
              />
              <button
                type="button"
                onClick={() => setShowConfirmPassword(!showConfirmPassword)}
                className="absolute right-2 top-1/2 -translate-y-1/2 p-1 hover:bg-gray-100 dark:hover:bg-botbot-dark rounded"
              >
                {showConfirmPassword ? (
                  <EyeOff className="w-4 h-4 text-gray-500" />
                ) : (
                  <Eye className="w-4 h-4 text-gray-500" />
                )}
              </button>
            </div>
          </div>

          {/* Buttons */}
          <div className="flex justify-end space-x-3 pt-4">
            <button
              type="button"
              onClick={onClose}
              className="px-4 py-2 text-sm font-medium text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-botbot-darker rounded-md transition-colors"
            >
              {t('common', 'cancel') || 'Cancel'}
            </button>
            <button
              type="submit"
              disabled={loading}
              className={`px-4 py-2 text-sm font-medium rounded-md transition-all ${
                loading
                  ? 'bg-gray-400 dark:bg-gray-600 cursor-not-allowed'
                  : 'bg-primary dark:bg-botbot-accent text-white hover:bg-primary/90 dark:hover:bg-botbot-accent/90'
              }`}
            >
              {loading ? (
                <>
                  <Loader2 className="w-4 h-4 animate-spin inline mr-2" />
                  {t('profile', 'updating') || 'Updating...'}
                </>
              ) : (
                t('profile', 'updatePassword') || 'Update Password'
              )}
            </button>
          </div>
        </form>
      </div>
    </>
  );
} 