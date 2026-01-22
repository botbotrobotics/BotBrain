'use client';

import { useEffect } from 'react';
import { useRouter } from 'next/navigation';
import { MessageCircle, Map, BarChart3, HelpCircle, Radio, ScrollText, Blocks, Navigation, Cloud } from 'lucide-react';
import { useHeader } from '@/contexts/HeaderContext';
import { useLanguage } from '@/contexts/LanguageContext';

export default function LabsPage() {
  const router = useRouter();
  const { setChatPopupOpen } = useHeader();
  const { t } = useLanguage();

  useEffect(() => {
    // Set page title
    document.title = 'Labs - BotBot';
  }, []);

  // OSS features - always available
  const ossFeatures = [
    {
      name: t('labs', 'chat'),
      icon: MessageCircle,
      description: t('labs', 'chatDescription'),
      action: () => setChatPopupOpen(true),
      badge: 'beta' as const,
    },
    {
      name: t('labs', 'charts'),
      icon: BarChart3,
      description: t('labs', 'chartsDescription'),
      action: () => router.push('/charts'),
      badge: 'beta' as const,
    },
    {
      name: t('labs', 'auditLog'),
      icon: ScrollText,
      description: t('labs', 'auditLogDescription'),
      action: () => router.push('/log'),
      badge: 'beta' as const,
    },
    {
      name: t('labs', 'missions'),
      icon: Navigation,
      description: t('labs', 'missionsDescription'),
      action: () => router.push('/missions'),
      badge: 'beta' as const,
    },
  ];

  // Pro features - only available in Pro builds
  const proFeatures = __PRO__ ? [
    {
      name: t('labs', 'mapEdit'),
      icon: Map,
      description: t('labs', 'mapEditDescription'),
      action: () => router.push('/map-edit'),
      badge: 'beta' as const,
    },
    {
      name: t('labs', 'blockProgramming'),
      icon: Blocks,
      description: t('labs', 'blockProgrammingDescription'),
      action: () => router.push('/block-programming'),
      badge: 'alpha' as const,
    },
    {
      name: t('labs', 'help'),
      icon: HelpCircle,
      description: t('labs', 'helpDescription'),
      action: () => router.push('/help'),
      badge: 'alpha' as const,
    },
    {
      name: 'Weather',
      icon: Cloud,
      description: 'Simple weather viewer for current conditions',
      action: () => router.push('/weather'),
      badge: 'alpha' as const,
    },
  ] : [];

  const labsFeatures = [...ossFeatures, ...proFeatures];

  return (
    <div className="w-full h-[calc(100vh-56px-24px)] flex flex-col md:flex-row items-stretch justify-between relative px-1">
      {/* Main content area */}
      <div className="w-full h-full flex flex-col justify-center pt-2 px-1">
        <div className="w-full h-full bg-white/5 backdrop-blur-sm rounded-lg p-4 overflow-y-auto">
          <div className="max-w-6xl mx-auto">
            {/* Header section */}
            <div className="mb-8">
              <h1 className="text-4xl font-bold text-gray-800 dark:text-white mb-4">
                {t('labs', 'pageTitle')}
              </h1>
              <p className="text-lg text-gray-600 dark:text-gray-400">
                {t('labs', 'pageDescription')}
              </p>
            </div>

            {/* Feature cards grid */}
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
              {labsFeatures.map((feature) => {
                const Icon = feature.icon;
                return (
                  <button
                    key={feature.name}
                    onClick={feature.action}
                    className="bg-white dark:bg-botbot-darker rounded-lg shadow-md p-6 hover:shadow-lg transition-all duration-200 transform hover:scale-105 text-left group"
                  >
                    {/* Icon container */}
                    <div className="w-12 h-12 rounded-full bg-purple-100 dark:bg-purple-900/20 flex items-center justify-center mb-4 group-hover:bg-purple-200 dark:group-hover:bg-purple-900/40 transition-colors">
                      <Icon className="w-6 h-6 text-purple-600 dark:text-purple-400" />
                    </div>
                    
                    {/* Feature name */}
                    <h3 className="text-xl font-semibold text-gray-800 dark:text-white mb-2">
                      {feature.name}
                    </h3>
                    
                    {/* Feature description */}
                    <p className="text-sm text-gray-600 dark:text-gray-400">
                      {feature.description}
                    </p>
                    
                    {/* Alpha/Beta badge */}
                    <div className="mt-4">
                      <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
                        feature.badge === 'alpha' 
                          ? 'bg-red-100 text-red-800 dark:bg-red-900/20 dark:text-red-300'
                          : 'bg-yellow-100 text-yellow-800 dark:bg-yellow-900/20 dark:text-yellow-300'
                      }`}>
                        {feature.badge === 'alpha' ? t('labs', 'alpha') : t('labs', 'beta')}
                      </span>
                    </div>
                  </button>
                );
              })}
            </div>

            {/* Additional info */}
            <div className="mt-12 space-y-4">
              {/* Alpha notice */}
              <div className="bg-red-50 dark:bg-red-900/10 border border-red-200 dark:border-red-900/30 rounded-lg p-6">
                <div className="flex items-start">
                  <div className="flex-shrink-0">
                    <svg className="h-5 w-5 text-red-400" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 20" fill="currentColor">
                      <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7 4a1 1 0 11-2 0 1 1 0 012 0zm-1-9a1 1 0 00-1 1v4a1 1 0 102 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
                    </svg>
                  </div>
                  <div className="ml-3">
                    <h3 className="text-sm font-medium text-red-800 dark:text-red-300">
                      {t('labs', 'alphaFeatures')}
                    </h3>
                    <div className="mt-2 text-sm text-red-700 dark:text-red-400">
                      <p>
                        {t('labs', 'alphaNotice')}
                      </p>
                    </div>
                  </div>
                </div>
              </div>

              {/* Beta notice */}
              <div className="bg-yellow-50 dark:bg-yellow-900/10 border border-yellow-200 dark:border-yellow-900/30 rounded-lg p-6">
                <div className="flex items-start">
                  <div className="flex-shrink-0">
                    <svg className="h-5 w-5 text-yellow-400" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 20" fill="currentColor">
                      <path fillRule="evenodd" d="M8.257 3.099c.765-1.36 2.722-1.36 3.486 0l5.58 9.92c.75 1.334-.213 2.98-1.742 2.98H4.42c-1.53 0-2.493-1.646-1.743-2.98l5.58-9.92zM11 13a1 1 0 11-2 0 1 1 0 012 0zm-1-8a1 1 0 00-1 1v3a1 1 0 002 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
                    </svg>
                  </div>
                  <div className="ml-3">
                    <h3 className="text-sm font-medium text-yellow-800 dark:text-yellow-300">
                      {t('labs', 'betaFeatures')}
                    </h3>
                    <div className="mt-2 text-sm text-yellow-700 dark:text-yellow-400">
                      <p>
                        {t('labs', 'betaNotice')}
                      </p>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
} 