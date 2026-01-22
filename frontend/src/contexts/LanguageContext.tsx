'use client';

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { translations, LanguageCode, TranslationKey, NestedTranslationKey } from '@/utils/translations';

type LanguageContextType = {
  language: LanguageCode;
  setLanguage: (lang: LanguageCode) => void;
  t: <T extends TranslationKey>(section: T, key: NestedTranslationKey<T>) => string;
};

const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

type LanguageProviderProps = {
  children: ReactNode;
};

export function LanguageProvider({ children }: LanguageProviderProps) {
  const [language, setLanguageState] = useState<LanguageCode>('pt'); // Default to Portuguese

  useEffect(() => {
    // Load saved language preference from localStorage if available
    const savedLanguage = localStorage.getItem('language') as LanguageCode | null;
    if (savedLanguage && ['en', 'pt'].includes(savedLanguage)) {
      setLanguageState(savedLanguage);
    } else {
      // Auto-detect from browser language
      const browserLang = navigator.language?.toLowerCase() || '';
      if (browserLang.startsWith('pt')) {
        setLanguageState('pt');
      } else {
        setLanguageState('en');
      }
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