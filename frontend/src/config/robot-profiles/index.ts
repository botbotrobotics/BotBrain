import { RobotProfile, RobotProfileRegistry, RobotProfileId } from './types';
import { Go2R1Profile } from './profiles/go2-r1';
import { TitaR1Profile } from './profiles/tita-r1';
import { G1R1Profile } from './profiles/g1-r1';

// Registry of all available robot profiles
export const robotProfiles: RobotProfileRegistry = {
  'Go2-R1': Go2R1Profile,
  'Tita-R1': TitaR1Profile,
  'G1-R1': G1R1Profile,
};

// Helper function to get a robot profile by ID
export function getRobotProfile(profileId: string): RobotProfile | undefined {
  return robotProfiles[profileId];
}

// Helper function to get all available robot profiles
export function getAllRobotProfiles(): RobotProfile[] {
  return Object.values(robotProfiles);
}

// Helper function to get robot profile names for dropdown
export function getRobotProfileOptions(): Array<{ value: string; label: string; description?: string }> {
  return Object.entries(robotProfiles).map(([key, profile]) => ({
    value: key,
    label: profile.name,
    description: `${profile.manufacturer} ${profile.model}`,
  }));
}

// Export types
export type { RobotProfile, RobotProfileRegistry, RobotProfileId };
export * from './types';