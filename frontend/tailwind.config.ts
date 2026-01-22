import type { Config } from 'tailwindcss';

export default {
  darkMode: ['class'],
  content: [
    './src/pages/**/*.{js,ts,jsx,tsx,mdx}',
    './src/components/**/*.{js,ts,jsx,tsx,mdx}',
    './src/app/**/*.{js,ts,jsx,tsx,mdx}',
  ],
  theme: {
    extend: {
      colors: {
        background: 'var(--background)',
        foreground: 'var(--foreground)',
        primary: 'var(--theme-primary, #821db7)',
        border: 'var(--theme-border, #230633)',
        secondary: 'var(--theme-secondary, #e9d0f5)',
        focus: 'var(--theme-focus, #6e159e)',
        'action-btn-focus': 'var(--theme-action-btn-focus, #d8bae6)',
        'pink-lighter': 'var(--theme-clear-pink, #f8eaff)',
        'focus-shadow': '#0c0112',
        'clear-pink': 'var(--theme-clear-pink, #F6EFFD)',
        'clear-gray': 'var(--theme-clear-gray, #dfd8e6)',
        'clear-gray-2': 'var(--theme-clear-gray-2, #c5becc)',
        botbot: {
          purple: 'var(--theme-primary, #8A2BE2)',
          dark: 'var(--theme-primary-dark, #2D1A45)',
          darker: 'var(--theme-primary-darker, #1A0F29)',
          darkest: 'var(--theme-primary-darkest, #0F0919)',
          accent: 'var(--theme-accent, #B388FF)',
        },
      },
      backgroundImage: {
        'home-img': "url('/bg.png')",
        'gradient-radial': 'radial-gradient(var(--tw-gradient-stops))',
        'gradient-conic':
          'conic-gradient(from 0deg at 50% 50%, var(--tw-gradient-stops))',
      },
      height: {
        inherit: 'inherit',
      },
      borderRadius: {
        'default-border': '25px',
        'menu-btn-border': '8px',
      },
      keyframes: {
        shrinkBounce: {
          '0%': { transform: 'scale(1)' },
          '50%': { transform: 'scale(0.8)' },
          '100%': { transform: 'scale(1)' },
        },
        heartBounce: {
          '0%': { transform: 'scale(0) translateY(0)', opacity: '0' },
          '30%': { transform: 'scale(1.4) translateY(-20px)', opacity: '1' },
          '50%': { transform: 'scale(1) translateY(10px)' },
          '70%': { transform: 'scale(1.2) translateY(-5px)' },
          '100%': { transform: 'scale(1) translateY(0)' },
        },
        'gradient-flow': {
          '0%, 100%': {
            'background-position': '0% 50%',
          },
          '50%': {
            'background-position': '100% 50%',
          },
        },
        float: {
          '0%, 100%': {
            transform: 'translateY(0px)',
          },
          '50%': {
            transform: 'translateY(-20px)',
          },
        },
        'float-slow': {
          '0%, 100%': {
            transform: 'translateY(0px) translateX(0px)',
          },
          '33%': {
            transform: 'translateY(-30px) translateX(20px)',
          },
          '66%': {
            transform: 'translateY(20px) translateX(-20px)',
          },
        },
        'float-slower': {
          '0%, 100%': {
            transform: 'translateY(0px) translateX(0px)',
          },
          '25%': {
            transform: 'translateY(30px) translateX(-30px)',
          },
          '50%': {
            transform: 'translateY(-20px) translateX(20px)',
          },
          '75%': {
            transform: 'translateY(20px) translateX(30px)',
          },
        },
        'slide-horizontal': {
          '0%': {
            transform: 'translateX(-120px)',
          },
          '50%': {
            transform: 'translateX(calc(100vw - 120px))',
          },
          '100%': {
            transform: 'translateX(-120px)',
          },
        },
        shake: {
          '0%, 100%': {
            transform: 'translateX(0)',
          },
          '10%, 30%, 50%, 70%, 90%': {
            transform: 'translateX(-10px)',
          },
          '20%, 40%, 60%, 80%': {
            transform: 'translateX(10px)',
          },
        },
        emoji1Animation: {
          '0%': {
            transform: 'scale(0) rotate(0deg)',
            opacity: '0',
            filter: 'blur(10px)',
          },
          '20%': {
            transform: 'scale(1.5) rotate(-15deg) translate(-40px, -20px)',
            opacity: '1',
            filter: 'blur(0)',
          },
          '40%': {
            transform: 'scale(1.2) rotate(10deg) translate(-30px, 30px)',
          },
          '60%': {
            transform: 'scale(1.4) rotate(-5deg) translate(-20px, -10px)',
          },
          '80%': {
            transform: 'scale(1.1) rotate(5deg) translate(-10px, 10px)',
          },
          '100%': {
            transform: 'scale(0) rotate(0deg) translate(0, 0)',
            opacity: '0',
            filter: 'blur(10px)',
          },
        },
        emoji2Animation: {
          '0%': {
            transform: 'scale(0) rotate(0deg)',
            opacity: '0',
            filter: 'blur(10px)',
          },
          '20%': {
            transform: 'scale(1.5) rotate(15deg) translate(40px, -20px)',
            opacity: '1',
            filter: 'blur(0)',
          },
          '40%': {
            transform: 'scale(1.2) rotate(-10deg) translate(30px, 30px)',
          },
          '60%': {
            transform: 'scale(1.4) rotate(5deg) translate(20px, -10px)',
          },
          '80%': {
            transform: 'scale(1.1) rotate(-5deg) translate(10px, 10px)',
          },
          '100%': {
            transform: 'scale(0) rotate(0deg) translate(0, 0)',
            opacity: '0',
            filter: 'blur(10px)',
          },
        },
        emoji3Animation: {
          '0%': {
            transform: 'scale(0) rotate(0deg)',
            opacity: '0',
            filter: 'blur(10px)',
          },
          '20%': {
            transform: 'scale(0.8) rotate(-10deg) translate(0, 50px)',
            opacity: '1',
            filter: 'blur(0)',
          },
          '40%': { transform: 'scale(1.2) rotate(360deg) translate(0, -30px)' },
          '60%': { transform: 'scale(1.5) rotate(720deg) translate(0, 0)' },
          '80%': { transform: 'scale(1.2) rotate(1080deg) translate(0, 20px)' },
          '100%': {
            transform: 'scale(0) rotate(1440deg) translate(0, 0)',
            opacity: '0',
            filter: 'blur(10px)',
          },
        },
        psychedelicBackground: {
          '0%': {
            backgroundImage:
              'linear-gradient(0deg, #ff00ff, #00ffff, #ffff00, #ff00ff)',
            backgroundSize: '400% 400%',
            backgroundPosition: '0% 0%',
            transform: 'scale(1.1)',
          },
          '25%': {
            backgroundImage:
              'linear-gradient(90deg, #00ff00, #ff00ff, #00ffff, #ffff00)',
            backgroundSize: '400% 400%',
            backgroundPosition: '100% 0%',
            transform: 'scale(1)',
          },
          '50%': {
            backgroundImage:
              'linear-gradient(180deg, #ffff00, #00ff00, #ff00ff, #00ffff)',
            backgroundSize: '400% 400%',
            backgroundPosition: '100% 100%',
            transform: 'scale(1.1)',
          },
          '75%': {
            backgroundImage:
              'linear-gradient(270deg, #00ffff, #ffff00, #00ff00, #ff00ff)',
            backgroundSize: '400% 400%',
            backgroundPosition: '0% 100%',
            transform: 'scale(1)',
          },
          '100%': {
            backgroundImage:
              'linear-gradient(360deg, #ff00ff, #00ffff, #ffff00, #ff00ff)',
            backgroundSize: '400% 400%',
            backgroundPosition: '0% 0%',
            transform: 'scale(1.1)',
          },
        },
        fadeIn: {
          '0%': { opacity: '0' },
          '100%': { opacity: '1' },
        },
        redScreenFlash: {
          '0%': { opacity: '0' },
          '12.5%': { opacity: '0.9' },
          '25%': { opacity: '0' },
          '37.5%': { opacity: '0.9' },
          '50%': { opacity: '0' },
          '62.5%': { opacity: '0.9' },
          '75%': { opacity: '0' },
          '87.5%': { opacity: '0.9' },
          '100%': { opacity: '0' },
        },
        'accordion-down': {
          from: { height: '0' },
          to: { height: 'var(--radix-accordion-content-height)' },
        },
        'accordion-up': {
          from: { height: 'var(--radix-accordion-content-height)' },
          to: { height: '0' },
        },
        'slide-up-fade-out': {
          '0%': { transform: 'translateY(0)', opacity: '1' },
          '100%': { transform: 'translateY(-20px)', opacity: '0' },
        },
        successActionFeedback: {
          '0%': { opacity: '1' },
          '100%': { opacity: '0' },
        },
        failActionFeedback: {
          '0%': { opacity: '1' },
          '100%': { opacity: '0' },
        },
        shimmer: {
          '0%': { transform: 'translateX(-100%)' },
          '100%': { transform: 'translateX(100%)' },
        },
      },
      animation: {
        shrinkBounce: 'shrinkBounce 0.4s ease-in-out',
        heartBounce: 'heartBounce 4s ease-in-out forwards',
        'gradient-flow': 'gradient-flow 15s ease infinite',
        float: 'float 3s ease-in-out infinite',
        'float-slow': 'float-slow 8s ease-in-out infinite',
        'float-slower': 'float-slower 10s ease-in-out infinite',
        'slide-horizontal': 'slide-horizontal 20s ease-in-out infinite',
        shake: 'shake 0.5s ease-in-out',
        emoji1:
          'emoji1Animation 4s cubic-bezier(0.25, 0.46, 0.45, 0.94) forwards',
        emoji2:
          'emoji2Animation 4s cubic-bezier(0.25, 0.46, 0.45, 0.94) forwards',
        emoji3:
          'emoji3Animation 4s cubic-bezier(0.215, 0.61, 0.355, 1) forwards',
        psychedelic:
          'psychedelicBackground 4s cubic-bezier(0.42, 0, 0.58, 1) forwards',
        fadeIn: 'fadeIn 1s ease-in forwards',
        spin: 'spin 3s linear infinite',
        pulse: 'pulse 2s cubic-bezier(0.4, 0, 0.6, 1) infinite',
        redFlash:
          'redScreenFlash 1.5s cubic-bezier(0.25, 0.46, 0.45, 0.94) forwards',
        'slide-up-fade-out': 'slide-up-fade-out 1s ease-out forwards',
        successActionFeedback: 'successActionFeedback 0.75s ease-out forwards',
        failActionFeedback: 'failActionFeedback 0.75s ease-out forwards',
        shimmer: 'shimmer 2s ease-in-out infinite',
      },
      screens: {
        '3xl': '1920px',
      },
      gridTemplateColumns: {
        '24': 'repeat(24, minmax(0, 1fr))',
      },
    },
  },
  safelist: [
    'bg-red-600',
    'hover:bg-red-600',
    'animate-spin',
    'animate-pulse',
    'animate-psychedelic',
    'animate-redFlash',
    'bg-gradient-radial',
    'bg-gradient-conic',
  ],
  plugins: [require('tailwindcss-animate')],
} satisfies Config;
