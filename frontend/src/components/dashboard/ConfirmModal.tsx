'use client';

import { useEffect, useCallback } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { AlertTriangle } from 'lucide-react';

interface ConfirmModalProps {
  isOpen: boolean;
  onClose: () => void;
  onConfirm: () => void;
  title: string;
  message: string;
  confirmText?: string;
  cancelText?: string;
  variant?: 'warning' | 'danger';
}

export function ConfirmModal({
  isOpen,
  onClose,
  onConfirm,
  title,
  message,
  confirmText = 'Confirm',
  cancelText = 'Cancel',
  variant = 'warning',
}: ConfirmModalProps) {
  // Handle ESC key
  const handleKeyDown = useCallback(
    (event: KeyboardEvent) => {
      if (event.key === 'Escape' && isOpen) {
        onClose();
      }
    },
    [isOpen, onClose]
  );

  useEffect(() => {
    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [handleKeyDown]);

  // Handle click outside
  const handleOverlayClick = (event: React.MouseEvent) => {
    if (event.target === event.currentTarget) {
      onClose();
    }
  };

  // Prevent body scroll when open
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = 'auto';
    }
    return () => {
      document.body.style.overflow = 'auto';
    };
  }, [isOpen]);

  const handleConfirm = () => {
    onConfirm();
    onClose();
  };

  const iconColors = {
    warning: {
      bg: 'bg-amber-100 dark:bg-amber-900/30',
      icon: 'text-amber-600 dark:text-amber-400',
      button: 'bg-amber-500 hover:bg-amber-600 focus:ring-amber-400',
    },
    danger: {
      bg: 'bg-red-100 dark:bg-red-900/30',
      icon: 'text-red-600 dark:text-red-400',
      button: 'bg-red-500 hover:bg-red-600 focus:ring-red-400',
    },
  };

  const colors = iconColors[variant];

  return (
    <AnimatePresence>
      {isOpen && (
        <motion.div
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          exit={{ opacity: 0 }}
          transition={{ duration: 0.2 }}
          className="fixed inset-0 bg-black/20 dark:bg-black/40 backdrop-blur-sm z-[90] flex items-center justify-center"
          onClick={handleOverlayClick}
        >
          <motion.div
            initial={{ opacity: 0, scale: 0.95 }}
            animate={{ opacity: 1, scale: 1 }}
            exit={{ opacity: 0, scale: 0.95 }}
            transition={{
              type: 'spring',
              damping: 25,
              stiffness: 300,
            }}
            className="bg-white dark:bg-botbot-dark shadow-lg rounded-lg p-6 min-w-[300px] max-w-[400px] mx-4"
            onClick={(e) => e.stopPropagation()}
          >
            {/* Warning icon */}
            <div className="flex justify-center mb-4">
              <div className={`p-3 rounded-full ${colors.bg}`}>
                <AlertTriangle className={`w-6 h-6 ${colors.icon}`} />
              </div>
            </div>

            {/* Title */}
            <h2 className="text-lg font-semibold text-center text-gray-900 dark:text-white mb-2">
              {title}
            </h2>

            {/* Message */}
            <p className="text-sm text-center text-gray-600 dark:text-gray-300 mb-6">
              {message}
            </p>

            {/* Buttons */}
            <div className="flex gap-3">
              <button
                onClick={onClose}
                className="flex-1 py-2.5 px-4 rounded-lg border border-gray-300 dark:border-gray-600
                           text-gray-700 dark:text-gray-200 font-medium
                           hover:bg-gray-100 dark:hover:bg-botbot-darker
                           transition-colors focus:outline-none focus:ring-2 focus:ring-gray-300 dark:focus:ring-gray-600"
              >
                {cancelText}
              </button>
              <button
                onClick={handleConfirm}
                className={`flex-1 py-2.5 px-4 rounded-lg text-white font-medium
                           transition-colors focus:outline-none focus:ring-2 ${colors.button}`}
              >
                {confirmText}
              </button>
            </div>
          </motion.div>
        </motion.div>
      )}
    </AnimatePresence>
  );
}
