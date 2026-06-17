import { enTranslations } from './en';
import { ptTranslations } from './pt';
import { zhCNTranslations } from './zh-CN';

export type TranslationKey = keyof typeof enTranslations;
export type NestedTranslationKey<T extends TranslationKey> =
  keyof (typeof enTranslations)[T];

export type LanguageCode = 'en' | 'pt' | 'zh-CN';

export const translations = {
  en: enTranslations,
  pt: ptTranslations,
  'zh-CN': zhCNTranslations,
};

export const languageNames = {
  en: '🇬🇧 English',
  pt: '🇧🇷 Português',
  'zh-CN': '🇨🇳 简体中文',
};
