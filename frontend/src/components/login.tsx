'use client';

import Image from 'next/image';
import Button from '@/components/ui/button';
import { useState, useEffect } from 'react';
import LoginInput from '@/components/ui/login-input';
import Checkbox from '@/components/ui/checkbox';
import { useRouter } from 'next/navigation';
import { useLanguage } from '@/contexts/LanguageContext';
import { languageNames, LanguageCode } from '@/utils/translations';
import LegalModal from '@/components/legal/LegalModal';
import { privacyPolicyContent, privacyPolicyLastUpdated } from '@/content/privacy-policy';
import { termsOfServiceContent, termsOfServiceLastUpdated } from '@/content/terms-of-service';

export default function Login() {
  const [mode, setMode] = useState<'login' | 'signup'>('login');
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    name: '',
  });
  const [loading, setLoading] = useState(false);
  const [rememberMe, setRememberMe] = useState(true);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState(false);
  const [signupSuccess, setSignupSuccess] = useState(false);
  const [passwordStrength, setPasswordStrength] = useState<{
    score: number;
    label: string;
    color: string;
  }>({ score: 0, label: '', color: '' });
  const [acceptedTerms, setAcceptedTerms] = useState(false);
  const [acceptedPrivacy, setAcceptedPrivacy] = useState(false);
  const [showPrivacyModal, setShowPrivacyModal] = useState(false);
  const [showTermsModal, setShowTermsModal] = useState(false);
  const [showLanguageDropdown, setShowLanguageDropdown] = useState(false);
  const router = useRouter();
  const { t, language, setLanguage } = useLanguage();

  // Close language dropdown on escape or click outside
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape') setShowLanguageDropdown(false);
    };
    const handleClickOutside = (e: MouseEvent) => {
      const target = e.target as HTMLElement;
      if (!target.closest('[data-language-selector]')) {
        setShowLanguageDropdown(false);
      }
    };
    if (showLanguageDropdown) {
      document.addEventListener('keydown', handleEscape);
      document.addEventListener('mousedown', handleClickOutside);
    }
    return () => {
      document.removeEventListener('keydown', handleEscape);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [showLanguageDropdown]);

  // Password strength calculation
  useEffect(() => {
    if (!formData.password || mode !== 'signup') {
      setPasswordStrength({ score: 0, label: '', color: '' });
      return;
    }

    let score = 0;
    const checks = {
      length: formData.password.length >= 8,
      lowercase: /[a-z]/.test(formData.password),
      uppercase: /[A-Z]/.test(formData.password),
      number: /[0-9]/.test(formData.password),
      special: /[^A-Za-z0-9]/.test(formData.password),
    };

    score = Object.values(checks).filter(Boolean).length;

    const strengthLevels = [
      { score: 1, label: t('login', 'passwordVeryWeak') || 'Very Weak', color: 'bg-red-500' },
      { score: 2, label: t('login', 'passwordWeak') || 'Weak', color: 'bg-orange-500' },
      { score: 3, label: t('login', 'passwordFair') || 'Fair', color: 'bg-yellow-500' },
      { score: 4, label: t('login', 'passwordGood') || 'Good', color: 'bg-green-500' },
      { score: 5, label: t('login', 'passwordStrong') || 'Strong', color: 'bg-green-600' },
    ];

    const strength = strengthLevels[Math.min(score - 1, 4)] || strengthLevels[0];
    setPasswordStrength(strength);
  }, [formData.password, mode, t]);

  function handleChange(e: React.ChangeEvent<HTMLInputElement>) {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value,
    });
    // Clear error when user starts typing
    if (error) setError('');
  }

  function handleModeSwitch(newMode: 'login' | 'signup') {
    setMode(newMode);
    setError('');
    setSuccess(false);
    setSignupSuccess(false);
    setAcceptedTerms(false);
    setAcceptedPrivacy(false);
  }

  async function handleSubmit(e: React.FormEvent) {
    e.preventDefault();
    if (mode === 'signup') {
      await handleSignup();
    } else {
      await handleLogin();
    }
  }

  async function handleLogin() {
    setLoading(true);
    setError('');
    setSuccess(false);

    try {
      const response = await fetch('/api/auth/login', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email: formData.email,
          password: formData.password,
          remember: rememberMe,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        // More user-friendly error messages
        if (data.error?.includes('Invalid login credentials')) {
          setError('The email or password you entered is incorrect. Please try again.');
        } else if (data.error?.includes('Email not confirmed')) {
          setError('Please check your email to confirm your account before logging in.');
        } else if (data.error?.includes('Too many requests')) {
          setError('Too many login attempts. Please wait a few minutes and try again.');
        } else {
          setError(data.error || 'Unable to log in. Please check your credentials and try again.');
        }
        setLoading(false);
      } else if (data.success) {
        setSuccess(true);
        // Use window.location for a full page navigation
        // This ensures the auth state is properly initialized on the next page
        setTimeout(() => {
          window.location.href = '/dashboard';
        }, 1000);
      }
    } catch (err) {
      setError(t('login', 'networkError') || 'Something went wrong. Please check your connection and try again.');
      setLoading(false);
    }
  }

  async function handleSignup() {
    setLoading(true);
    setError('');
    setSignupSuccess(false);

    // Client-side validation
    if (!acceptedTerms || !acceptedPrivacy) {
      setError(t('login', 'termsRequired') || 'You must accept the Terms of Service and Privacy Policy');
      setLoading(false);
      return;
    }

    if (formData.password.length < 8) {
      setError(t('login', 'passwordMinLength') || 'Password must be at least 8 characters');
      setLoading(false);
      return;
    }

    if (formData.password !== formData.confirmPassword) {
      setError(t('login', 'passwordsDoNotMatch') || 'Passwords do not match');
      setLoading(false);
      return;
    }

    try {
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          email: formData.email,
          password: formData.password,
          name: formData.name || null,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        // Map error messages to user-friendly text
        if (data.error?.includes('already registered') || data.error?.includes('already been registered')) {
          setError(t('login', 'emailAlreadyExists') || 'An account with this email already exists. Try logging in instead.');
        } else if (data.error?.includes('weak password') || data.error?.includes('should be at least')) {
          setError(t('login', 'passwordTooWeak') || 'Please choose a stronger password');
        } else if (data.error?.includes('rate limit')) {
          setError(t('login', 'rateLimitExceeded') || 'Too many attempts. Please wait a few minutes and try again.');
        } else {
          setError(data.error || t('login', 'signupError') || 'Unable to create account. Please try again.');
        }
        setLoading(false);
      } else if (data.success) {
        setSignupSuccess(true);
        // Clear form
        setFormData({ email: '', password: '', confirmPassword: '', name: '' });
        setLoading(false);
      }
    } catch (err) {
      setError(t('login', 'networkError') || 'Connection error. Please check your internet and try again.');
      setLoading(false);
    }
  }

  return (
    <div className="relative min-h-screen w-full overflow-hidden">
      {/* Animated Background */}
      <div className="fixed inset-0 -z-10">
        <div className="absolute inset-0 bg-gradient-to-br from-purple-100 via-pink-50 to-indigo-100 dark:from-botbot-darker dark:via-botbot-dark dark:to-botbot-darkest animate-gradient-flow" />
        <div className="absolute inset-0 overflow-hidden">
          <div className="absolute -top-40 -right-40 h-80 w-80 rounded-full bg-purple-300/30 dark:bg-purple-600/20 blur-3xl animate-float-slow" />
          <div className="absolute -bottom-40 -left-40 h-80 w-80 rounded-full bg-pink-300/30 dark:bg-pink-600/20 blur-3xl animate-float-slower" />
          <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 h-96 w-96 rounded-full bg-indigo-300/20 dark:bg-indigo-600/10 blur-3xl animate-float" />
        </div>
      </div>

      {/* Language Selector */}
      <div className="fixed top-4 right-4 sm:top-6 sm:right-6 z-20" data-language-selector>
        <div className="relative">
          <button
            onClick={() => setShowLanguageDropdown(!showLanguageDropdown)}
            className="flex items-center gap-2 px-3 py-2 sm:px-4 sm:py-2.5 rounded-full bg-white/80 dark:bg-botbot-dark/80 backdrop-blur-lg border border-white/20 dark:border-botbot-purple/20 shadow-lg hover:shadow-xl transition-all duration-200 hover:scale-105"
            aria-label="Select language"
          >
            <span className="text-lg sm:text-xl">{language === 'en' ? '🇬🇧' : '🇧🇷'}</span>
            <svg
              className={`w-4 h-4 text-gray-600 dark:text-gray-300 transition-transform duration-200 ${showLanguageDropdown ? 'rotate-180' : ''}`}
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
            </svg>
          </button>

          {/* Dropdown */}
          {showLanguageDropdown && (
            <div className="absolute right-0 mt-2 py-2 w-44 rounded-xl bg-white/90 dark:bg-botbot-dark/90 backdrop-blur-lg border border-white/20 dark:border-botbot-purple/20 shadow-xl animate-fadeIn">
              {(Object.keys(languageNames) as LanguageCode[]).map((code) => (
                <button
                  key={code}
                  onClick={() => {
                    setLanguage(code);
                    setShowLanguageDropdown(false);
                  }}
                  className={`w-full px-4 py-2.5 flex items-center gap-3 transition-colors ${
                    language === code
                      ? 'bg-botbot-purple/10 dark:bg-botbot-purple/20 text-botbot-purple dark:text-botbot-accent font-medium'
                      : 'text-gray-700 dark:text-gray-200 hover:bg-gray-100 dark:hover:bg-botbot-darker/50'
                  }`}
                >
                  <span className="text-xl">{code === 'en' ? '🇬🇧' : '🇧🇷'}</span>
                  <span className="text-sm">{code === 'en' ? 'English' : 'Português'}</span>
                  {language === code && (
                    <svg className="w-4 h-4 ml-auto text-botbot-purple dark:text-botbot-accent" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                    </svg>
                  )}
                </button>
              ))}
            </div>
          )}
        </div>
      </div>

      {/* Main Content Container */}
      <div className="relative z-10 flex min-h-screen flex-col items-center justify-center p-4 pb-40 lg:pb-48">
        {/* Logo - responsive sizing based on mode */}
        <div className={`shrink-0 transition-all duration-300 ${mode === 'signup' ? 'w-24 h-24 sm:w-32 sm:h-32 mb-4' : 'w-36 h-36 sm:w-48 sm:h-48 mb-6'}`}>
          <Image
            src="/botbot-logo.png"
            alt="BotBot Logo"
            width={270}
            height={270}
            priority
            className="drop-shadow-lg w-full h-full object-contain"
          />
        </div>

        {/* Login/Signup Card */}
        <div className="w-full max-w-md animate-fadeIn">
          <div className="rounded-3xl bg-white/80 dark:bg-botbot-dark/80 backdrop-blur-lg shadow-2xl p-8 sm:p-10 border border-white/20 dark:border-botbot-purple/20">
            <h2 className="text-2xl font-bold text-center mb-2 text-gray-800 dark:text-white">
              {mode === 'signup'
                ? (t('login', 'createAccountTitle') || 'Create Your Account')
                : (t('login', 'welcomeBack') || 'Welcome Back')}
            </h2>

            {/* Mode Toggle */}
            <div className="flex justify-center mb-6 mt-4">
              <div className="inline-flex rounded-full bg-gray-100 dark:bg-botbot-darker/50 p-1">
                <button
                  type="button"
                  onClick={() => handleModeSwitch('login')}
                  className={`px-4 py-2 rounded-full text-sm font-medium transition-all duration-200 ${
                    mode === 'login'
                      ? 'bg-white dark:bg-botbot-purple text-gray-800 dark:text-white shadow-sm'
                      : 'text-gray-500 dark:text-gray-400 hover:text-gray-700 dark:hover:text-gray-300'
                  }`}
                >
                  {t('login', 'signIn') || 'Sign In'}
                </button>
                <button
                  type="button"
                  onClick={() => handleModeSwitch('signup')}
                  className={`px-4 py-2 rounded-full text-sm font-medium transition-all duration-200 ${
                    mode === 'signup'
                      ? 'bg-white dark:bg-botbot-purple text-gray-800 dark:text-white shadow-sm'
                      : 'text-gray-500 dark:text-gray-400 hover:text-gray-700 dark:hover:text-gray-300'
                  }`}
                >
                  {t('login', 'createAccount') || 'Create Account'}
                </button>
              </div>
            </div>

            <form
              onSubmit={handleSubmit}
              className="space-y-6"
              method="POST"
              autoComplete="off"
            >
              {/* Status Messages */}
              {error && (
                <div className="rounded-lg bg-red-50 dark:bg-red-900/20 border border-red-200 dark:border-red-800 p-4 text-sm text-red-600 dark:text-red-400 animate-shake">
                  <div className="flex items-start">
                    <svg className="w-5 h-5 mr-2 flex-shrink-0 mt-0.5" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7 4a1 1 0 11-2 0 1 1 0 012 0zm-1-9a1 1 0 00-1 1v4a1 1 0 102 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
                    </svg>
                    <span>{error}</span>
                  </div>
                </div>
              )}

              {/* Login Success */}
              {success && mode === 'login' && (
                <div className="rounded-lg bg-green-50 dark:bg-green-900/20 border border-green-200 dark:border-green-800 p-4 text-sm text-green-600 dark:text-green-400 animate-fadeIn">
                  <div className="flex items-center">
                    <svg className="w-5 h-5 mr-2 flex-shrink-0 animate-bounce" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd" />
                    </svg>
                    <span>Success! Redirecting to your dashboard...</span>
                  </div>
                </div>
              )}

              {/* Signup Success */}
              {signupSuccess && mode === 'signup' && (
                <div className="rounded-lg bg-green-50 dark:bg-green-900/20 border border-green-200 dark:border-green-800 p-4 text-sm text-green-600 dark:text-green-400 animate-fadeIn">
                  <div className="flex items-start">
                    <svg className="w-5 h-5 mr-2 flex-shrink-0 mt-0.5" fill="currentColor" viewBox="0 0 20 20">
                      <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd" />
                    </svg>
                    <div>
                      <p className="font-medium">{t('login', 'signupSuccessTitle') || 'Account Created!'}</p>
                      <p className="mt-1">{t('login', 'signupSuccessMessage') || 'Please check your email to confirm your account before logging in.'}</p>
                    </div>
                  </div>
                </div>
              )}

              {/* Loading State */}
              {loading && !success && !signupSuccess && (
                <div className="rounded-lg bg-blue-50 dark:bg-blue-900/20 border border-blue-200 dark:border-blue-800 p-4 text-sm text-blue-600 dark:text-blue-400 animate-fadeIn">
                  <div className="flex items-center">
                    <svg className="w-5 h-5 mr-2 flex-shrink-0 animate-spin" fill="none" viewBox="0 0 24 24">
                      <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                      <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                    </svg>
                    <span>
                      {mode === 'signup'
                        ? (t('login', 'creatingAccount') || 'Creating account...')
                        : 'Verifying your credentials...'}
                    </span>
                  </div>
                </div>
              )}

              {/* Name Field (Signup only) */}
              {mode === 'signup' && (
                <LoginInput
                  label={t('login', 'name') || 'Name'}
                  name="name"
                  value={formData.name}
                  placeholder={t('login', 'namePlaceholder') || 'Enter your name (optional)'}
                  type="text"
                  onChange={handleChange}
                  autoComplete="name"
                />
              )}

              <LoginInput
                label={t('login', 'email') || 'Email'}
                name="email"
                value={formData.email}
                placeholder="Enter your email"
                type="email"
                onChange={handleChange}
                autoComplete="username"
              />

              <LoginInput
                label={t('login', 'password') || 'Password'}
                name="password"
                value={formData.password}
                placeholder="Enter your password"
                type="password"
                onChange={handleChange}
                autoComplete={mode === 'signup' ? 'new-password' : 'current-password'}
              />

              {/* Confirm Password Field (Signup only) */}
              {mode === 'signup' && (
                <>
                  <LoginInput
                    label={t('login', 'confirmPassword') || 'Confirm Password'}
                    name="confirmPassword"
                    value={formData.confirmPassword}
                    placeholder={t('login', 'confirmPasswordPlaceholder') || 'Re-enter your password'}
                    type="password"
                    onChange={handleChange}
                    autoComplete="new-password"
                  />

                  {/* Password Strength Indicator */}
                  {formData.password && (
                    <div className="mt-2">
                      <div className="flex justify-between items-center mb-1">
                        <span className="text-xs text-gray-500 dark:text-gray-400">
                          {t('login', 'passwordStrength') || 'Password Strength'}
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
                </>
              )}

              {/* Remember Me (Login only) */}
              {mode === 'login' && (
                <div className="flex items-center">
                  <Checkbox
                    label={t('login', 'rememberMe')}
                    id="rememberMe"
                    labelClassName="text-gray-600 dark:text-gray-300 text-sm"
                    checked={rememberMe}
                    onChange={(e) => setRememberMe(e.target.checked)}
                  />
                </div>
              )}

              {/* Terms and Privacy Acceptance (Signup only) */}
              {mode === 'signup' && (
                <div className="space-y-3">
                  {/* Terms of Service Checkbox */}
                  <div className="flex items-start gap-3">
                    <div className="pt-0.5">
                      <input
                        type="checkbox"
                        id="acceptTerms"
                        checked={acceptedTerms}
                        onChange={(e) => setAcceptedTerms(e.target.checked)}
                        className="w-4 h-4 rounded border-gray-300 dark:border-gray-600 text-botbot-purple focus:ring-botbot-purple focus:ring-2 bg-white dark:bg-botbot-darker cursor-pointer"
                      />
                    </div>
                    <label htmlFor="acceptTerms" className="text-sm text-gray-600 dark:text-gray-300 leading-relaxed cursor-pointer">
                      {t('login', 'acceptTermsLabel') || 'I agree to the'}{' '}
                      <button
                        type="button"
                        onClick={(e) => {
                          e.preventDefault();
                          setShowTermsModal(true);
                        }}
                        className="text-botbot-purple hover:underline font-medium"
                      >
                        {t('login', 'termsOfService') || 'Terms of Service'}
                      </button>
                    </label>
                  </div>

                  {/* Privacy Policy Checkbox */}
                  <div className="flex items-start gap-3">
                    <div className="pt-0.5">
                      <input
                        type="checkbox"
                        id="acceptPrivacy"
                        checked={acceptedPrivacy}
                        onChange={(e) => setAcceptedPrivacy(e.target.checked)}
                        className="w-4 h-4 rounded border-gray-300 dark:border-gray-600 text-botbot-purple focus:ring-botbot-purple focus:ring-2 bg-white dark:bg-botbot-darker cursor-pointer"
                      />
                    </div>
                    <label htmlFor="acceptPrivacy" className="text-sm text-gray-600 dark:text-gray-300 leading-relaxed cursor-pointer">
                      {t('login', 'acceptPrivacyLabel') || 'I agree to the'}{' '}
                      <button
                        type="button"
                        onClick={(e) => {
                          e.preventDefault();
                          setShowPrivacyModal(true);
                        }}
                        className="text-botbot-purple hover:underline font-medium"
                      >
                        {t('login', 'privacyPolicy') || 'Privacy Policy'}
                      </button>
                    </label>
                  </div>
                </div>
              )}

              <Button
                type="submit"
                label={
                  loading
                    ? (mode === 'signup'
                        ? (t('login', 'creatingAccount') || 'Creating account...')
                        : (t('login', 'loggingIn') || 'Logging in...'))
                    : (mode === 'signup'
                        ? (t('login', 'createAccountButton') || 'Create Account')
                        : (t('login', 'loginButton') || 'Login'))
                }
                customClasses={`!mt-6 ${loading ? 'opacity-80 cursor-wait' : ''} ${(success || signupSuccess) ? 'bg-green-600 hover:bg-green-700' : ''} transition-all duration-300`}
              />
            </form>
          </div>
        </div>
      </div>

      {/* Bot Image at Bottom - moves side to side only, behind content */}
      <div className="fixed bottom-0 left-0 right-0 z-0 h-32 pointer-events-none hidden lg:block">
        <div className="animate-slide-horizontal">
          <Image
            src="/bot.png"
            alt="BotBot Mascot"
            width={120}
            height={120}
            priority
            className="drop-shadow-xl opacity-80"
          />
        </div>
      </div>

      {/* Legal Modals */}
      <LegalModal
        isOpen={showPrivacyModal}
        onClose={() => setShowPrivacyModal(false)}
        title={t('login', 'privacyPolicy') || 'Privacy Policy'}
        content={privacyPolicyContent[language as keyof typeof privacyPolicyContent] || privacyPolicyContent.en}
        lastUpdated={privacyPolicyLastUpdated}
      />
      <LegalModal
        isOpen={showTermsModal}
        onClose={() => setShowTermsModal(false)}
        title={t('login', 'termsOfService') || 'Terms of Service'}
        content={termsOfServiceContent[language as keyof typeof termsOfServiceContent] || termsOfServiceContent.en}
        lastUpdated={termsOfServiceLastUpdated}
      />
    </div>
  );
}
