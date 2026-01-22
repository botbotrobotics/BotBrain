'use client';

import React from 'react';
import Container from './container';
import { X } from 'lucide-react';
import { MenuActionType } from '../../types/RobotActionTypes';

interface PopupProps {
  isOpen: boolean;
  title?: string | React.ReactNode;
  actions?: MenuActionType[];
  onClose: () => void;
  children?: React.ReactNode;
  className?: string;
  customContentClasses?: string;
}

export default function Popup({
  isOpen,
  title = '',
  actions,
  onClose,
  children,
  className = '',
  customContentClasses = '',
}: PopupProps) {
  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black bg-opacity-50">
      <div className="relative">
        <Container
          title={title}
          actions={actions}
          className={className}
          customContentClasses={customContentClasses}
        >
          <button
            onClick={onClose}
            className="absolute top-2 right-2 p-2 hover:bg-gray-300 rounded-full"
          >
            <X className="w-5 h-5" />
          </button>
          {children}
        </Container>
      </div>
    </div>
  );
}
