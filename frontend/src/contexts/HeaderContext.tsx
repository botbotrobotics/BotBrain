'use client';

import React, { createContext, useState, useContext, ReactNode } from 'react';

interface HeaderTypeContextData {
  navMenuCollapsed: boolean;
  chatPopupOpen: boolean;
  joystickEnabled: boolean;
  extrasBarVisible: boolean;
  updateNavMenuCollapsed: (newValue: boolean) => void;
  setChatPopupOpen: (newValue: boolean) => void;
  setJoystickEnabled: (newValue: boolean) => void;
  setExtrasBarVisible: (newValue: boolean) => void;
}

const HeaderContext = createContext<HeaderTypeContextData | undefined>(
  undefined
);

export const HeaderProvider: React.FC<{ children: ReactNode }> = ({
  children,
}) => {
  const [navMenuCollapsed, setNavMenuCollapsed] = useState<boolean>(false);
  const [chatPopupOpen, setChatPopupOpen] = useState<boolean>(false);
  const [joystickEnabled, setJoystickEnabled] = useState<boolean>(false);
  const [extrasBarVisible, setExtrasBarVisible] = useState<boolean>(false);

  const updateNavMenuCollapsed = (newValue: boolean) => {
    setNavMenuCollapsed(newValue);
  };

  return (
    <HeaderContext.Provider
      value={{
        navMenuCollapsed,
        chatPopupOpen,
        joystickEnabled,
        extrasBarVisible,
        updateNavMenuCollapsed,
        setChatPopupOpen,
        setJoystickEnabled,
        setExtrasBarVisible,
      }}
    >
      {children}
    </HeaderContext.Provider>
  );
};

export const useHeader = (): HeaderTypeContextData => {
  const context = useContext(HeaderContext);
  if (!context) {
    throw new Error('useHeader must be used within a HeaderProvider');
  }
  return context;
};
