/**
 * Converts hex color to RGB
 */
function hexToRgb(hex: string): { r: number; g: number; b: number } | null {
  const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return result
    ? {
        r: parseInt(result[1], 16),
        g: parseInt(result[2], 16),
        b: parseInt(result[3], 16),
      }
    : null;
}

/**
 * Converts RGB to HSL
 */
function rgbToHsl(r: number, g: number, b: number): { h: number; s: number; l: number } {
  r /= 255;
  g /= 255;
  b /= 255;

  const max = Math.max(r, g, b);
  const min = Math.min(r, g, b);
  let h = 0;
  let s = 0;
  const l = (max + min) / 2;

  if (max !== min) {
    const d = max - min;
    s = l > 0.5 ? d / (2 - max - min) : d / (max + min);
    
    switch (max) {
      case r:
        h = ((g - b) / d + (g < b ? 6 : 0)) / 6;
        break;
      case g:
        h = ((b - r) / d + 2) / 6;
        break;
      case b:
        h = ((r - g) / d + 4) / 6;
        break;
    }
  }

  return {
    h: Math.round(h * 360),
    s: Math.round(s * 100),
    l: Math.round(l * 100),
  };
}

/**
 * Converts HSL to hex color
 */
function hslToHex(h: number, s: number, l: number): string {
  h = h / 360;
  s = s / 100;
  l = l / 100;

  let r, g, b;

  if (s === 0) {
    r = g = b = l;
  } else {
    const hue2rgb = (p: number, q: number, t: number) => {
      if (t < 0) t += 1;
      if (t > 1) t -= 1;
      if (t < 1 / 6) return p + (q - p) * 6 * t;
      if (t < 1 / 2) return q;
      if (t < 2 / 3) return p + (q - p) * (2 / 3 - t) * 6;
      return p;
    };

    const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
    const p = 2 * l - q;
    r = hue2rgb(p, q, h + 1 / 3);
    g = hue2rgb(p, q, h);
    b = hue2rgb(p, q, h - 1 / 3);
  }

  const toHex = (x: number) => {
    const hex = Math.round(x * 255).toString(16);
    return hex.length === 1 ? '0' + hex : hex;
  };

  return `#${toHex(r)}${toHex(g)}${toHex(b)}`;
}

/**
 * Generates a complete theme based on a base color
 */
export function generateTheme(baseColor: string) {
  const rgb = hexToRgb(baseColor);
  if (!rgb) return null;

  const hsl = rgbToHsl(rgb.r, rgb.g, rgb.b);

  // Generate color variations
  const colors = {
    // Primary color (the base color)
    primary: baseColor,
    
    // Darker variations for dark mode backgrounds
    primaryDark: hslToHex(hsl.h, Math.min(hsl.s * 0.9, 60), 20),
    primaryDarker: hslToHex(hsl.h, Math.min(hsl.s * 0.8, 50), 12),
    primaryDarkest: hslToHex(hsl.h, Math.min(hsl.s * 0.7, 40), 8),
    
    // Lighter variations for light mode (more vibrant)
    primaryLight: hslToHex(hsl.h, Math.max(hsl.s * 0.85, 60), 55),
    primaryLighter: hslToHex(hsl.h, Math.max(hsl.s * 0.7, 50), 70),
    primaryLightest: hslToHex(hsl.h, Math.max(hsl.s * 0.5, 30), 85),
    
    // Accent color (slightly shifted hue, more vibrant)
    accent: hslToHex((hsl.h + 20) % 360, Math.max(hsl.s * 0.9, 70), 50),
    
    // Focus and border colors (more saturated)
    focus: hslToHex(hsl.h, Math.max(hsl.s * 0.95, 70), 35),
    border: hslToHex(hsl.h, Math.min(hsl.s * 0.4, 30), 25),
    
    // Action button focus (more vibrant)
    actionBtnFocus: hslToHex(hsl.h, Math.max(hsl.s * 0.6, 40), 65),
    
    // Secondary color (more saturated)
    secondary: hslToHex(hsl.h, Math.max(hsl.s * 0.5, 35), 80),
    
    // Clear variations (slightly more tinted)
    clearPink: hslToHex(hsl.h, Math.max(hsl.s * 0.3, 20), 94),
    clearGray: hslToHex(hsl.h, 10, 82),
    clearGray2: hslToHex(hsl.h, 10, 72),
  };

  return colors;
}

/**
 * Applies theme colors to CSS variables
 */
export function applyTheme(colors: ReturnType<typeof generateTheme>) {
  if (!colors) return;

  const root = document.documentElement;

  // Apply colors to CSS variables
  root.style.setProperty('--theme-primary', colors.primary);
  root.style.setProperty('--theme-primary-dark', colors.primaryDark);
  root.style.setProperty('--theme-primary-darker', colors.primaryDarker);
  root.style.setProperty('--theme-primary-darkest', colors.primaryDarkest);
  root.style.setProperty('--theme-primary-light', colors.primaryLight);
  root.style.setProperty('--theme-primary-lighter', colors.primaryLighter);
  root.style.setProperty('--theme-primary-lightest', colors.primaryLightest);
  root.style.setProperty('--theme-accent', colors.accent);
  root.style.setProperty('--theme-focus', colors.focus);
  root.style.setProperty('--theme-border', colors.border);
  root.style.setProperty('--theme-action-btn-focus', colors.actionBtnFocus);
  root.style.setProperty('--theme-secondary', colors.secondary);
  root.style.setProperty('--theme-clear-pink', colors.clearPink);
  root.style.setProperty('--theme-clear-gray', colors.clearGray);
  root.style.setProperty('--theme-clear-gray-2', colors.clearGray2);
}

/**
 * Resets theme to default
 */
export function resetTheme() {
  const root = document.documentElement;
  
  // Remove all custom theme properties
  const themeProps = [
    '--theme-primary',
    '--theme-primary-dark',
    '--theme-primary-darker',
    '--theme-primary-darkest',
    '--theme-primary-light',
    '--theme-primary-lighter',
    '--theme-primary-lightest',
    '--theme-accent',
    '--theme-focus',
    '--theme-border',
    '--theme-action-btn-focus',
    '--theme-secondary',
    '--theme-clear-pink',
    '--theme-clear-gray',
    '--theme-clear-gray-2',
  ];
  
  themeProps.forEach(prop => root.style.removeProperty(prop));
} 