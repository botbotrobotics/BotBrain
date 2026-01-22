/**
 * Null Module - Used in OSS builds to replace Pro imports
 *
 * When building with BOTBRAIN_EDITION=oss, all imports from @/app-pro/*
 * and @/components-pro/* are redirected here, returning empty/null values.
 */

import React from 'react';

// Default export for module-level imports
export default {};

// Generic null component for any Pro component imports
export const ProComponent: React.FC<{ children?: React.ReactNode }> = () => null;

// Generic null hook for any Pro hook imports
export const useProHook = () => ({
  enabled: false,
  data: null,
  loading: false,
  error: null,
});

// Proxy to handle any dynamic property access
const nullProxy = new Proxy({}, {
  get: () => () => null,
});

export { nullProxy as proFeatures };
