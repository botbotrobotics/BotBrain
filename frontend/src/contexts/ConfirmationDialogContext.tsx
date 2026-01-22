'use client';

import React, { createContext, useContext, useState, useCallback, ReactNode } from 'react';
import { RobotActionTypeName } from '@/types/RobotActionTypes';

// Configuration for which actions require confirmation
// To add/remove actions, simply edit this array
export const DANGEROUS_ACTIONS: RobotActionTypeName[] = ['getDown', 'damping'];

interface ConfirmationRequest {
  actionName: RobotActionTypeName;
  actionLabel: string;
  onConfirm: () => void;
  onCancel?: () => void;
}

interface ConfirmationDialogContextType {
  isOpen: boolean;
  currentRequest: ConfirmationRequest | null;
  requestConfirmation: (request: ConfirmationRequest) => void;
  confirm: () => void;
  cancel: () => void;
  requiresConfirmation: (actionName: RobotActionTypeName) => boolean;
}

const ConfirmationDialogContext = createContext<ConfirmationDialogContextType | undefined>(undefined);

export function ConfirmationDialogProvider({ children }: { children: ReactNode }) {
  const [isOpen, setIsOpen] = useState(false);
  const [currentRequest, setCurrentRequest] = useState<ConfirmationRequest | null>(null);

  const requiresConfirmation = useCallback((actionName: RobotActionTypeName): boolean => {
    return DANGEROUS_ACTIONS.includes(actionName);
  }, []);

  const requestConfirmation = useCallback((request: ConfirmationRequest) => {
    setCurrentRequest(request);
    setIsOpen(true);
  }, []);

  const confirm = useCallback(() => {
    if (currentRequest?.onConfirm) {
      currentRequest.onConfirm();
    }
    setIsOpen(false);
    setCurrentRequest(null);
  }, [currentRequest]);

  const cancel = useCallback(() => {
    if (currentRequest?.onCancel) {
      currentRequest.onCancel();
    }
    setIsOpen(false);
    setCurrentRequest(null);
  }, [currentRequest]);

  return (
    <ConfirmationDialogContext.Provider
      value={{
        isOpen,
        currentRequest,
        requestConfirmation,
        confirm,
        cancel,
        requiresConfirmation,
      }}
    >
      {children}
    </ConfirmationDialogContext.Provider>
  );
}

export function useConfirmationDialog() {
  const context = useContext(ConfirmationDialogContext);
  if (context === undefined) {
    throw new Error('useConfirmationDialog must be used within a ConfirmationDialogProvider');
  }
  return context;
}
