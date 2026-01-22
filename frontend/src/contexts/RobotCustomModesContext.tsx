'use client';

import { RobotStatus } from '@/types/RobotStatus';
import React, { createContext, useState, useContext, ReactNode } from 'react';

interface RobotCustomModeContextType {
  light: boolean;
  antiCollision: boolean;
  statusBeforeEmergency: RobotStatus | undefined;
  isPose: boolean; // for joystick data handling
  setLight: React.Dispatch<React.SetStateAction<boolean>>;
  setAntiCollision: React.Dispatch<React.SetStateAction<boolean>>;
  setStatusBeforeEmergency: React.Dispatch<
    React.SetStateAction<RobotStatus | undefined>
  >;
  setIsPose: React.Dispatch<React.SetStateAction<boolean>>;
}

const RobotCustomModesContext = createContext<
  RobotCustomModeContextType | undefined
>(undefined);

export const RobotCustomModeProvider: React.FC<{ children: ReactNode }> = ({
  children,
}) => {
  const [light, setLight] = useState(false);
  const [isPose, setIsPose] = useState(false);
  const [antiCollision, setAntiCollision] = useState(false);
  const [statusBeforeEmergency, setStatusBeforeEmergency] = useState<
    RobotStatus | undefined
  >(undefined);

  return (
    <RobotCustomModesContext.Provider
      value={{
        light,
        antiCollision,
        statusBeforeEmergency,
        isPose,
        setLight,
        setAntiCollision,
        setStatusBeforeEmergency,
        setIsPose,
      }}
    >
      {children}
    </RobotCustomModesContext.Provider>
  );
};

export const useRobotCustomModeContext = () => {
  const context = useContext(RobotCustomModesContext);
  if (!context) {
    throw new Error(
      'useRobotCustomModeContext deve ser usado dentro de um RobotCustomModeProvider'
    );
  }
  return context;
};
