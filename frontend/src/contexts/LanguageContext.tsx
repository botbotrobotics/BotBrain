'use client';

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { translations, LanguageCode, TranslationKey, NestedTranslationKey } from '@/utils/translations';

type LanguageContextType = {
  language: LanguageCode;
  setLanguage: (lang: LanguageCode) => void;
  t: <T extends TranslationKey>(section: T, key: NestedTranslationKey<T>) => string;
};

const LanguageContext = createContext<LanguageContextType | undefined>(undefined);
const supportedLanguages = ['en', 'pt', 'zh-CN'] as const;

type LanguageProviderProps = {
  children: ReactNode;
};

function isSupportedLanguage(language: string): language is LanguageCode {
  return supportedLanguages.includes(language as LanguageCode);
}

function detectBrowserLanguage(): LanguageCode {
  const browserLanguages = [
    ...(navigator.languages || []),
    navigator.language,
  ].filter(Boolean).map((lang) => lang.toLowerCase());

  if (browserLanguages.some((lang) => lang === 'zh' || lang.startsWith('zh-'))) {
    return 'zh-CN';
  }

  if (browserLanguages.some((lang) => lang.startsWith('pt'))) {
    return 'pt';
  }

  return 'en';
}

export function LanguageProvider({ children }: LanguageProviderProps) {
  const [language, setLanguageState] = useState<LanguageCode>('pt'); // Default to Portuguese

  useEffect(() => {
    // Load saved language preference from localStorage if available
    const savedLanguage = localStorage.getItem('language');
    if (savedLanguage && isSupportedLanguage(savedLanguage)) {
      setLanguageState(savedLanguage);
    } else {
      setLanguageState(detectBrowserLanguage());
    }
  }, []);

  const setLanguage = (lang: LanguageCode) => {
    setLanguageState(lang);
    localStorage.setItem('language', lang);
  };

  // Translation function
  const t = <T extends TranslationKey>(section: T, key: NestedTranslationKey<T>): string => {
    // Type assertion to avoid index signature error
    const sectionTranslations = translations[language][section] as Record<string, string>;
    return sectionTranslations[key as string] || `${String(section)}.${String(key)}`;
  };

  return (
    <LanguageContext.Provider value={{ language, setLanguage, t }}>
      {children}
    </LanguageContext.Provider>
  );
}

export function useLanguage() {
  const context = useContext(LanguageContext);
  if (context === undefined) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }
  return context;
}
