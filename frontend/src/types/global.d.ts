export {};

declare global {
  interface Window {
    THREE: typeof import('three');
  }

  /**
   * Build-time constant that indicates whether this is a Pro build.
   * Set via webpack DefinePlugin in next.config.js
   * - true: Pro build (includes all features)
   * - false: OSS build (excludes Pro features)
   */
  const __PRO__: boolean;
}
