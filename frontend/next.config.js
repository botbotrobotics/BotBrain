const path = require('path');

// Determine build edition from environment variable
const isPro = process.env.BOTBRAIN_EDITION === 'pro';

/** @type {import('next').NextConfig} */
const nextConfig = {
  webpack: (config, { isServer }) => {
    // Mock window for server-side builds
    if (isServer) {
      config.resolve.fallback = {
        ...config.resolve.fallback,
        window: false,
      };

      // Replace window references in problematic modules
      config.module.rules.push({
        test: /node_modules[/\\](apexcharts|react-apexcharts)/,
        use: [
          {
            loader: 'string-replace-loader',
            options: {
              search: 'window',
              replace: 'globalThis',
              flags: 'g'
            }
          }
        ]
      });
    }

    config.resolve.alias = {
      ...config.resolve.alias,
      uuid: require.resolve('uuid'),
    };

    // === PRO/OSS BUILD SEPARATION ===
    // Define build-time constant for Pro features
    config.plugins.push(
      new (require('webpack').DefinePlugin)({
        'process.env.IS_PRO': JSON.stringify(isPro),
        '__PRO__': JSON.stringify(isPro),
      })
    );

    // In OSS builds, redirect Pro imports to null module
    if (!isPro) {
      config.resolve.alias = {
        ...config.resolve.alias,
        '@/app-pro': path.resolve(__dirname, 'src/lib/null-module'),
        '@/components-pro': path.resolve(__dirname, 'src/lib/null-module'),
      };
    }

    return config;
  },
  eslint: {
    // Warning: This allows production builds to successfully complete even if
    // your project has ESLint errors.
    ignoreDuringBuilds: true,
  },

  // Redirect Pro routes to dashboard in OSS builds
  async redirects() {
    if (!isPro) {
      return [
        { source: '/ai', destination: '/dashboard', permanent: false },
        { source: '/ai/:path*', destination: '/dashboard', permanent: false },
        { source: '/block-programming', destination: '/dashboard', permanent: false },
        { source: '/block-programming/:path*', destination: '/dashboard', permanent: false },
        { source: '/help', destination: '/dashboard', permanent: false },
        { source: '/help/:path*', destination: '/dashboard', permanent: false },
        { source: '/weather', destination: '/dashboard', permanent: false },
        { source: '/weather/:path*', destination: '/dashboard', permanent: false },
        { source: '/map-edit', destination: '/dashboard', permanent: false },
        { source: '/map-edit/:path*', destination: '/dashboard', permanent: false },
        { source: '/soundboard', destination: '/dashboard', permanent: false },
        { source: '/soundboard/:path*', destination: '/dashboard', permanent: false },
      ];
    }
    return [];
  },

  // Security headers
  async headers() {
    return [
      {
        source: '/:path*',
        headers: [
          { key: 'X-Frame-Options', value: 'DENY' },
          { key: 'X-Content-Type-Options', value: 'nosniff' },
          { key: 'Referrer-Policy', value: 'strict-origin-when-cross-origin' },
          { key: 'X-XSS-Protection', value: '1; mode=block' },
        ],
      },
    ];
  },
};

module.exports = nextConfig;
