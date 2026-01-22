'use client';

import React, { useEffect, useCallback } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { AlertTriangle } from 'lucide-react';
import { useConfirmationDialog } from '@/contexts/ConfirmationDialogContext';
import { useLanguage } from '@/contexts/LanguageContext';

export function ConfirmationDialog() {
  const { isOpen, currentRequest, confirm, cancel } = useConfirmationDialog();
  const { t } = useLanguage();

  // Handle ESC key
  const handleKeyDown = useCallback(
    (event: KeyboardEvent) => {
      if (event.key === 'Escape' && isOpen) {
        cancel();
      }
    },
    [isOpen, cancel]
  );

  useEffect(() => {
    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [handleKeyDown]);

  // Handle click outside
  const handleOverlayClick = (event: React.MouseEvent) => {
    if (event.target === event.currentTarget) {
      cancel();
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

  return (
    <AnimatePresence>
      {isOpen && currentRequest && (
        <>
          {/* Backdrop overlay with centered content */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            transition={{ duration: 0.2 }}
            className="fixed inset-0 bg-black/20 dark:bg-black/40 backdrop-blur-sm z-[90]
                       flex items-center justify-center"
            onClick={handleOverlayClick}
          >
            {/* Dialog */}
            <motion.div
              data-confirmation-dialog
              initial={{ opacity: 0, scale: 0.95 }}
              animate={{ opacity: 1, scale: 1 }}
              exit={{ opacity: 0, scale: 0.95 }}
              transition={{
                type: 'spring',
                damping: 25,
                stiffness: 300,
              }}
              className="bg-white dark:bg-botbot-dark shadow-lg rounded-lg p-6
                         min-w-[300px] max-w-[400px]"
              onClick={(e) => e.stopPropagation()}
            >
            {/* Warning icon */}
            <div className="flex justify-center mb-4">
              <div className="p-3 rounded-full bg-amber-100 dark:bg-amber-900/30">
                <AlertTriangle className="w-6 h-6 text-amber-600 dark:text-amber-400" />
              </div>
            </div>

            {/* Title */}
            <h2 className="text-lg font-semibold text-center text-gray-900 dark:text-white mb-2">
              {t('confirmationDialog', 'title')}
            </h2>

            {/* Message */}
            <p className="text-sm text-center text-gray-600 dark:text-gray-300 mb-6">
              {t('confirmationDialog', 'message').replace(
                '{action}',
                currentRequest.actionLabel
              )}
            </p>

            {/* Buttons */}
            <div className="flex gap-3">
              <button
                onClick={cancel}
                className="flex-1 py-2.5 px-4 rounded-lg border border-gray-300 dark:border-gray-600
                           text-gray-700 dark:text-gray-200 font-medium
                           hover:bg-gray-100 dark:hover:bg-botbot-darker
                           transition-colors focus:outline-none focus:ring-2 focus:ring-gray-300 dark:focus:ring-gray-600"
              >
                {t('confirmationDialog', 'cancel')}
              </button>
              <button
                onClick={confirm}
                className="flex-1 py-2.5 px-4 rounded-lg
                           bg-amber-500 hover:bg-amber-600
                           text-white font-medium
                           transition-colors focus:outline-none focus:ring-2 focus:ring-amber-400"
              >
                {t('confirmationDialog', 'confirm')}
              </button>
            </div>
            </motion.div>
          </motion.div>
        </>
      )}
    </AnimatePresence>
  );
}
