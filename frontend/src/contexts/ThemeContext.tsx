'use client';

import { createContext, useContext, useEffect, useState } from 'react';
import { useSupabase } from './SupabaseProvider';
import { generateTheme, applyTheme, resetTheme } from '@/utils/theme-generator';

interface ThemeContextType {
  themeColor: string | null;
  setThemeColor: (color: string | null) => void;
  isLoading: boolean;
}

const ThemeContext = createContext<ThemeContextType | undefined>(undefined);

export function ThemeProvider({ children }: { children: React.ReactNode }) {
  const { user, supabase } = useSupabase();
  const [themeColor, setThemeColor] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // Load theme color from user profile
  useEffect(() => {
    if (user && supabase) {
      loadThemeColor();
    } else {
      // Reset theme when user logs out
      setThemeColor(null);
      resetTheme();
      setIsLoading(false);
    }
  }, [user, supabase]);

  // Apply theme whenever color changes
  useEffect(() => {
    if (themeColor) {
      const colors = generateTheme(themeColor);
      if (colors) {
        applyTheme(colors);
      }
    } else {
      resetTheme();
    }
  }, [themeColor]);

  const loadThemeColor = async () => {
    if (!supabase || !user) return;

    try {
      const { data: profile, error } = await supabase
        .from('user_profiles')
        .select('theme_color')
        .eq('user_id', user.id)
        .single();

      if (!error && profile && profile.theme_color) {
        setThemeColor(profile.theme_color);
      }
    } catch (error) {
      console.error('Error loading theme color:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const updateThemeColor = (color: string | null) => {
    setThemeColor(color);
  };

  return (
    <ThemeContext.Provider value={{ themeColor, setThemeColor: updateThemeColor, isLoading }}>
      {children}
    </ThemeContext.Provider>
  );
}

export const useTheme = () => {
  const context = useContext(ThemeContext);
  if (context === undefined) {
    throw new Error('useTheme must be used within a ThemeProvider');
  }
  return context;
};
