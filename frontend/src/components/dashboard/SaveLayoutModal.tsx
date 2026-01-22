'use client';

import { useState, useEffect, useCallback } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { X, Save } from 'lucide-react';
import { useDashboard } from '@/contexts/DashboardContext';
import { dashboardService } from '@/services/supabase-dashboard';
import { useLanguage } from '@/contexts/LanguageContext';

interface SaveLayoutModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSaveSuccess: () => void;
}

export function SaveLayoutModal({ isOpen, onClose, onSaveSuccess }: SaveLayoutModalProps) {
  const [layoutName, setLayoutName] = useState('');
  const [description, setDescription] = useState('');
  const [isPublic, setIsPublic] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const { widgets } = useDashboard();
  const { t } = useLanguage();

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

  const handleSave = async () => {
    if (!layoutName.trim()) {
      setError(t('myUI', 'layoutNameRequired'));
      return;
    }

    if (widgets.length === 0) {
      setError(t('myUI', 'cannotSaveEmpty'));
      return;
    }

    setIsSaving(true);
    setError(null);

    try {
      await dashboardService.createLayout({
        name: layoutName.trim(),
        description: description.trim() || undefined,
        layout_data: widgets,
        is_public: isPublic,
      });

      setLayoutName('');
      setDescription('');
      setIsPublic(false);
      onSaveSuccess();
      onClose();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to save layout');
    } finally {
      setIsSaving(false);
    }
  };

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
            className="bg-white dark:bg-botbot-dark shadow-lg rounded-lg p-6 w-full max-w-md mx-4"
            onClick={(e) => e.stopPropagation()}
          >
            {/* Header with icon */}
            <div className="flex justify-center mb-4">
              <div className="p-3 rounded-full bg-violet-100 dark:bg-violet-900/30">
                <Save className="w-6 h-6 text-violet-600 dark:text-violet-400" />
              </div>
            </div>

            <h2 className="text-lg font-semibold text-center text-gray-900 dark:text-white mb-6">
              {t('myUI', 'saveDashboardLayout')}
            </h2>

            <div className="space-y-4">
              <div>
                <label
                  htmlFor="layout-name"
                  className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1"
                >
                  {t('myUI', 'layoutName')}
                </label>
                <input
                  id="layout-name"
                  type="text"
                  value={layoutName}
                  onChange={(e) => setLayoutName(e.target.value)}
                  className="w-full px-3 py-2 border border-gray-300 dark:border-gray-600 rounded-lg bg-white dark:bg-botbot-darker text-gray-900 dark:text-white focus:ring-2 focus:ring-violet-500 dark:focus:ring-botbot-purple focus:border-transparent transition-all"
                  placeholder={t('myUI', 'layoutNamePlaceholder')}
                />
              </div>

              <div>
                <label
                  htmlFor="layout-description"
                  className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1"
                >
                  {t('myUI', 'descriptionOptional')}
                </label>
                <textarea
                  id="layout-description"
                  value={description}
                  onChange={(e) => setDescription(e.target.value)}
                  rows={3}
                  className="w-full px-3 py-2 border border-gray-300 dark:border-gray-600 rounded-lg bg-white dark:bg-botbot-darker text-gray-900 dark:text-white focus:ring-2 focus:ring-violet-500 dark:focus:ring-botbot-purple focus:border-transparent transition-all"
                  placeholder={t('myUI', 'descriptionPlaceholder')}
                />
              </div>

              <div className="flex items-center">
                <input
                  id="is-public"
                  type="checkbox"
                  checked={isPublic}
                  onChange={(e) => setIsPublic(e.target.checked)}
                  className="h-4 w-4 text-violet-600 dark:text-botbot-purple border-gray-300 dark:border-gray-600 rounded focus:ring-violet-500 dark:focus:ring-botbot-purple"
                />
                <label
                  htmlFor="is-public"
                  className="ml-2 text-sm text-gray-700 dark:text-gray-300"
                >
                  {t('myUI', 'makePublic')}
                </label>
              </div>

              {error && (
                <div className="bg-red-50 dark:bg-red-900/20 border border-red-200 dark:border-red-800 text-red-600 dark:text-red-400 px-3 py-2 rounded-lg text-sm">
                  {error}
                </div>
              )}
            </div>

            <div className="flex gap-3 mt-6">
              <button
                onClick={onClose}
                className="flex-1 py-2.5 px-4 rounded-lg border border-gray-300 dark:border-gray-600
                           text-gray-700 dark:text-gray-200 font-medium
                           hover:bg-gray-100 dark:hover:bg-botbot-darker
                           transition-colors focus:outline-none focus:ring-2 focus:ring-gray-300 dark:focus:ring-gray-600"
              >
                {t('myUI', 'cancel')}
              </button>
              <button
                onClick={handleSave}
                disabled={isSaving || !layoutName.trim()}
                className="flex-1 py-2.5 px-4 rounded-lg
                           bg-violet-600 hover:bg-violet-700 dark:bg-botbot-purple dark:hover:bg-botbot-purple/90
                           text-white font-medium
                           transition-colors focus:outline-none focus:ring-2 focus:ring-violet-400
                           disabled:opacity-50 disabled:cursor-not-allowed"
              >
                {isSaving ? t('myUI', 'saving') : t('myUI', 'saveLayout')}
              </button>
            </div>
          </motion.div>
        </motion.div>
      )}
    </AnimatePresence>
  );
}
