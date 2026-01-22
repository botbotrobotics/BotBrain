import { enTranslations } from './en';
import { ptTranslations } from './pt';

export type TranslationKey = keyof typeof enTranslations;
export type NestedTranslationKey<T extends TranslationKey> =
  keyof (typeof enTranslations)[T];

export type LanguageCode = 'en' | 'pt';

export const translations = {
  en: enTranslations,
  pt: ptTranslations,
};

export const languageNames = {
  en: '🇬🇧 English',
  pt: '🇧🇷 Português',
};
