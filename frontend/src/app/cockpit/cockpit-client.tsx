'use client';

// This wrapper exists for potential future client-side providers specific to cockpit
// The ActiveMissionProvider has been moved to the root layout for app-wide access
export default function CockpitClient({ children }: { children: React.ReactNode }) {
  return <>{children}</>;
}
