'use client';

import { useState, useEffect } from 'react';
import {
  ChevronDown,
  Plus,
  Save,
  Trash2,
  Loader2,
  Layout,
  FolderOpen,
  Edit2,
  Star,
} from 'lucide-react';
import { useDashboard, WidgetType } from '@/contexts/DashboardContext';
import { useLanguage } from '@/contexts/LanguageContext';
import layouts, { LayoutType } from '@/utils/my-ui/layouts';
import { dashboardService, DashboardLayout } from '@/services/supabase-dashboard';
import { SaveLayoutModal } from './SaveLayoutModal';
import { EditLayoutModal } from './EditLayoutModal';
import { AddWidgetModal } from './AddWidgetModal';
import { ConfirmModal } from './ConfirmModal';
import { useSupabase } from '@/contexts/SupabaseProvider';

export function WidgetSelector() {
  const [showAddWidgetModal, setShowAddWidgetModal] = useState(false);
  const [layoutIsOpen, setLayoutIsOpen] = useState(false);
  const { addWidget, clearWidgets, widgets } = useDashboard();
  const { t } = useLanguage();
  const { user } = useSupabase();

  // New states for Supabase integration
  const [savedLayouts, setSavedLayouts] = useState<DashboardLayout[]>([]);
  const [publicLayouts, setPublicLayouts] = useState<DashboardLayout[]>([]);
  const [isLoadingLayouts, setIsLoadingLayouts] = useState(false);
  const [isLoadingPublicLayouts, setIsLoadingPublicLayouts] = useState(false);
  const [showSaveModal, setShowSaveModal] = useState(false);
  const [showEditModal, setShowEditModal] = useState(false);
  const [editingLayout, setEditingLayout] = useState<DashboardLayout | null>(null);
  const [activeTab, setActiveTab] = useState<'default' | 'saved' | 'public'>('default');
  const [deletingId, setDeletingId] = useState<string | null>(null);

  // Confirm modal states
  const [showClearConfirm, setShowClearConfirm] = useState(false);
  const [showDeleteConfirm, setShowDeleteConfirm] = useState(false);
  const [showUpdateConfirm, setShowUpdateConfirm] = useState(false);
  const [pendingDeleteId, setPendingDeleteId] = useState<string | null>(null);
  const [pendingUpdateLayout, setPendingUpdateLayout] = useState<DashboardLayout | null>(null);

  const layoutOptions: { value: LayoutType; label: string }[] = [
    { value: 'Default', label: t('myUI', 'layoutDefault') },
    { value: 'Command Input', label: t('myUI', 'layoutCommandInput') },
    { value: 'Information', label: t('myUI', 'layoutInformation') },
    { value: 'Image Visualization', label: t('myUI', 'layoutImageVisualization') },
  ];

  useEffect(() => {
    if (user) {
      fetchUserLayouts();
      loadFavoriteOnStart();
    }
    fetchPublicLayouts();
  }, [user]);

  const fetchUserLayouts = async () => {
    setIsLoadingLayouts(true);
    try {
      const userLayouts = await dashboardService.getUserLayouts();
      setSavedLayouts(userLayouts);
    } catch (error) {
      console.error('Failed to fetch layouts:', error);
    } finally {
      setIsLoadingLayouts(false);
    }
  };

  const fetchPublicLayouts = async () => {
    setIsLoadingPublicLayouts(true);
    try {
      const layouts = await dashboardService.getPublicLayouts();
      setPublicLayouts(layouts);
    } catch (error) {
      console.error('Failed to fetch public layouts:', error);
    } finally {
      setIsLoadingPublicLayouts(false);
    }
  };

  const loadFavoriteOnStart = async () => {
    try {
      const favoriteLayout = await dashboardService.getFavoriteLayout();
      if (favoriteLayout) {
        clearWidgets();
        setTimeout(() => {
          favoriteLayout.layout_data.forEach((widget) => {
            addWidget(widget.type, widget);
          });
        }, 100);
      }
    } catch (error) {
      console.error('Failed to load favorite layout:', error);
    }
  };

  const handleToggleFavorite = async (layoutId: string) => {
    try {
      await dashboardService.toggleFavorite(layoutId);
      await fetchUserLayouts();
    } catch (error) {
      console.error('Failed to toggle favorite:', error);
    }
  };

  const handleAddWidget = (type: WidgetType) => {
    addWidget(type);
    setShowAddWidgetModal(false);
  };

  const updateLayout = (layout: string) => {
    clearWidgets();
    
    // Simple timeout to ensure DOM is ready
    setTimeout(() => {
      const selectedLayout = layouts[layout as LayoutType];
      selectedLayout.forEach((widget) => {
        addWidget(widget.type, widget);
      });
      setLayoutIsOpen(false);
    }, 50);
  };

  const loadSavedLayout = async (layout: DashboardLayout) => {
    clearWidgets();
    
    // Simple timeout to ensure DOM is ready
    setTimeout(() => {
      layout.layout_data.forEach((widget) => {
        addWidget(widget.type, widget);
      });
      setLayoutIsOpen(false);
    }, 50);
  };

  const handleDeleteLayout = (layoutId: string) => {
    setPendingDeleteId(layoutId);
    setShowDeleteConfirm(true);
  };

  const confirmDeleteLayout = async () => {
    if (!pendingDeleteId) return;
    setDeletingId(pendingDeleteId);
    try {
      await dashboardService.deleteLayout(pendingDeleteId);
      setSavedLayouts(prev => prev.filter(l => l.id !== pendingDeleteId));
    } catch (error) {
      console.error('Failed to delete layout:', error);
    } finally {
      setDeletingId(null);
      setPendingDeleteId(null);
    }
  };

  const handleEditLayout = (layout: DashboardLayout) => {
    setEditingLayout(layout);
    setShowEditModal(true);
  };

  const handleUpdateLayoutData = (layout: DashboardLayout) => {
    if (!user) {
      alert(t('myUI', 'loginToUpdate'));
      return;
    }
    setPendingUpdateLayout(layout);
    setShowUpdateConfirm(true);
  };

  const confirmUpdateLayoutData = async () => {
    if (!pendingUpdateLayout) return;
    try {
      await dashboardService.updateLayout(pendingUpdateLayout.id, {
        layout_data: widgets
      });
      fetchUserLayouts();
    } catch (error) {
      console.error('Failed to update layout data:', error);
    } finally {
      setPendingUpdateLayout(null);
    }
  };

  const handleSaveToSupabase = () => {
    if (!user) {
      alert(t('myUI', 'loginToSave'));
      return;
    }
    setShowSaveModal(true);
  };


  return (
    <div className="relative">
      <div className="flex space-x-1">
        <button
          onClick={() => setShowAddWidgetModal(true)}
          className="inline-flex items-center justify-center rounded-md px-3 py-1.5 text-sm font-medium bg-primary dark:bg-botbot-purple text-white hover:bg-primary/90 dark:hover:bg-botbot-purple/90 focus:outline-none"
        >
          <Plus className="mr-1 h-4 w-4" />
          {t('myUI', 'addWidget')}
        </button>

        <button
          onClick={() => setLayoutIsOpen(!layoutIsOpen)}
          className="inline-flex items-center justify-center rounded-md px-3 py-1.5 text-sm font-medium bg-primary dark:bg-botbot-purple text-white hover:bg-primary/90 dark:hover:bg-botbot-purple/90 focus:outline-none"
        >
          {t('myUI', 'layout')}
          <ChevronDown className="ml-1 h-4 w-4" />
        </button>


        {user && (
          <button
            onClick={handleSaveToSupabase}
            className="inline-flex items-center justify-center rounded-md px-3 py-1.5 text-sm font-medium bg-green-600 dark:bg-green-700 text-white hover:bg-green-700 dark:hover:bg-green-800 focus:outline-none"
            title={t('myUI', 'saveToCloudTooltip')}
          >
            <FolderOpen className="h-4 w-4 mr-1" />
            {t('myUI', 'saveToCloud')}
          </button>
        )}

        <button
          onClick={() => {
            if (widgets.length > 0) {
              setShowClearConfirm(true);
            } else {
              clearWidgets();
            }
          }}
          className="inline-flex items-center justify-center rounded-md px-3 py-1.5 text-sm font-medium bg-gray-600 dark:bg-gray-700 text-white hover:bg-gray-700 dark:hover:bg-gray-800 focus:outline-none"
          title="Clear current layout and start fresh"
        >
          <Layout className="h-4 w-4 mr-1" />
          New Layout
        </button>

      </div>

      {layoutIsOpen && (
        <div className="absolute top-full left-[11.5rem] mt-1 w-80 rounded-md shadow-lg bg-white dark:bg-botbot-dark ring-1 ring-black ring-opacity-5 focus:outline-none z-50">
          <div className="flex border-b border-gray-200 dark:border-gray-700">
            <button
              onClick={() => setActiveTab('default')}
              className={`flex-1 px-4 py-2 text-sm font-medium ${
                activeTab === 'default'
                  ? 'text-primary dark:text-botbot-purple border-b-2 border-primary dark:border-botbot-purple'
                  : 'text-gray-500 dark:text-gray-400 hover:text-gray-700 dark:hover:text-gray-300'
              }`}
            >
              {t('myUI', 'defaultLayouts')}
            </button>
            <button
              onClick={() => setActiveTab('saved')}
              className={`flex-1 px-4 py-2 text-sm font-medium ${
                activeTab === 'saved'
                  ? 'text-primary dark:text-botbot-purple border-b-2 border-primary dark:border-botbot-purple'
                  : 'text-gray-500 dark:text-gray-400 hover:text-gray-700 dark:hover:text-gray-300'
              }`}
            >
              {t('myUI', 'myLayouts')}
            </button>
            <button
              onClick={() => setActiveTab('public')}
              className={`flex-1 px-4 py-2 text-sm font-medium ${
                activeTab === 'public'
                  ? 'text-primary dark:text-botbot-purple border-b-2 border-primary dark:border-botbot-purple'
                  : 'text-gray-500 dark:text-gray-400 hover:text-gray-700 dark:hover:text-gray-300'
              }`}
            >
              {t('myUI', 'publicLayouts')}
            </button>
          </div>
          
          <div className="max-h-64 overflow-y-auto">
            {activeTab === 'default' ? (
              <div className="py-1" role="menu" aria-orientation="vertical">
                {layoutOptions.map((option) => (
                  <button
                    key={option.value}
                    onClick={() => updateLayout(option.value)}
                    className="text-left w-full px-4 py-2 text-sm text-gray-700 dark:text-gray-200 hover:bg-gray-100 dark:hover:bg-botbot-darker flex items-center"
                    role="menuitem"
                  >
                    <Layout className="w-4 h-4 mr-2 text-gray-400" />
                    {option.label}
                  </button>
                ))}
              </div>
            ) : activeTab === 'saved' ? (
              <div className="py-1" role="menu" aria-orientation="vertical">
                {isLoadingLayouts ? (
                  <div className="px-4 py-8 text-center">
                    <Loader2 className="w-6 h-6 animate-spin mx-auto text-gray-400" />
                    <p className="text-sm text-gray-500 dark:text-gray-400 mt-2">{t('myUI', 'loadingLayouts')}</p>
                  </div>
                ) : savedLayouts.length === 0 ? (
                  <div className="px-4 py-8 text-center">
                    <p className="text-sm text-gray-500 dark:text-gray-400">{t('myUI', 'noSavedLayouts')}</p>
                    <p className="text-xs text-gray-400 dark:text-gray-500 mt-1">{t('myUI', 'noSavedLayoutsDescription')}</p>
                  </div>
                ) : (
                  savedLayouts.map((layout) => (
                    <div
                      key={layout.id}
                      className="group flex items-center justify-between px-4 py-2 hover:bg-gray-100 dark:hover:bg-botbot-darker"
                    >
                      <button
                        onClick={() => loadSavedLayout(layout)}
                        className="flex-1 text-left flex items-start flex-col"
                      >
                        <div className="flex items-center gap-2">
                          <span className="text-sm text-gray-700 dark:text-gray-200">
                            {layout.name}
                          </span>
                        </div>
                        <div className="flex items-center gap-2">
                          {layout.description && (
                            <span className="text-xs text-gray-500 dark:text-gray-400 line-clamp-1 flex-1">
                              {layout.description}
                            </span>
                          )}
                          <span className="text-xs text-gray-400 dark:text-gray-500">
                            {new Date(layout.created_at).toLocaleDateString()}
                          </span>
                          {layout.is_public && (
                            <span className="text-xs px-1.5 py-0.5 bg-green-100 dark:bg-green-900/30 text-green-700 dark:text-green-400 rounded">
                              {t('myUI', 'public')}
                            </span>
                          )}
                        </div>
                      </button>
                      <div className="flex items-center space-x-1">
                        <button
                          onClick={() => handleToggleFavorite(layout.id)}
                          className={`p-1 transition-all ${
                            layout.is_favorite
                              ? 'text-yellow-500 hover:text-yellow-600'
                              : 'text-gray-400 hover:text-yellow-500 opacity-0 group-hover:opacity-100'
                          }`}
                          title={layout.is_favorite ? t('myUI', 'removeFavorite') : t('myUI', 'setAsFavorite')}
                        >
                          <Star className={`w-4 h-4 ${layout.is_favorite ? 'fill-current' : ''}`} />
                        </button>
                        <button
                          onClick={() => handleUpdateLayoutData(layout)}
                          className="p-1 text-gray-400 hover:text-blue-600 dark:hover:text-blue-400 opacity-0 group-hover:opacity-100 transition-opacity"
                          title={t('myUI', 'updateLayoutTooltip')}
                        >
                          <Save className="w-4 h-4" />
                        </button>
                        <button
                          onClick={() => handleEditLayout(layout)}
                          className="p-1 text-gray-400 hover:text-gray-600 dark:hover:text-gray-300 opacity-0 group-hover:opacity-100 transition-opacity"
                          title={t('myUI', 'editLayoutTooltip')}
                        >
                          <Edit2 className="w-4 h-4" />
                        </button>
                        <button
                          onClick={() => handleDeleteLayout(layout.id)}
                          disabled={deletingId === layout.id}
                          className="p-1 text-gray-400 hover:text-red-600 dark:hover:text-red-400 opacity-0 group-hover:opacity-100 transition-opacity"
                          title={t('myUI', 'deleteLayoutTooltip')}
                        >
                          {deletingId === layout.id ? (
                            <Loader2 className="w-4 h-4 animate-spin" />
                          ) : (
                            <Trash2 className="w-4 h-4" />
                          )}
                        </button>
                      </div>
                    </div>
                  ))
                )}
              </div>
            ) : (
              <div className="py-1" role="menu" aria-orientation="vertical">
                {isLoadingPublicLayouts ? (
                  <div className="px-4 py-8 text-center">
                    <Loader2 className="w-6 h-6 animate-spin mx-auto text-gray-400" />
                    <p className="text-sm text-gray-500 dark:text-gray-400 mt-2">{t('myUI', 'loadingPublicLayouts')}</p>
                  </div>
                ) : publicLayouts.length === 0 ? (
                  <div className="px-4 py-8 text-center">
                    <p className="text-sm text-gray-500 dark:text-gray-400">{t('myUI', 'noPublicLayouts')}</p>
                  </div>
                ) : (
                  publicLayouts.map((layout) => (
                    <button
                      key={layout.id}
                      onClick={() => loadSavedLayout(layout)}
                      className="w-full text-left px-4 py-2 hover:bg-gray-100 dark:hover:bg-botbot-darker"
                    >
                      <div className="flex items-start flex-col">
                        <span className="text-sm text-gray-700 dark:text-gray-200">
                          {layout.name}
                        </span>
                        <div className="flex items-center gap-2 w-full">
                          {layout.description && (
                            <span className="text-xs text-gray-500 dark:text-gray-400 line-clamp-1 flex-1">
                              {layout.description}
                            </span>
                          )}
                          <span className="text-xs text-gray-400 dark:text-gray-500">
                            {new Date(layout.created_at).toLocaleDateString()}
                          </span>
                        </div>
                      </div>
                    </button>
                  ))
                )}
              </div>
            )}
          </div>
        </div>
      )}

      <SaveLayoutModal
        isOpen={showSaveModal}
        onClose={() => setShowSaveModal(false)}
        onSaveSuccess={() => {
          fetchUserLayouts();
        }}
      />

      <EditLayoutModal
        layout={editingLayout}
        isOpen={showEditModal}
        onClose={() => {
          setShowEditModal(false);
          setEditingLayout(null);
        }}
        onUpdateSuccess={() => {
          fetchUserLayouts();
        }}
      />

      <AddWidgetModal
        isOpen={showAddWidgetModal}
        onClose={() => setShowAddWidgetModal(false)}
        onAddWidget={handleAddWidget}
      />

      {/* Clear Layout Confirmation */}
      <ConfirmModal
        isOpen={showClearConfirm}
        onClose={() => setShowClearConfirm(false)}
        onConfirm={clearWidgets}
        title={t('myUI', 'newLayoutTitle') || 'New Layout'}
        message={t('myUI', 'clearLayoutConfirm') || 'Are you sure you want to clear the current layout? All widgets will be removed.'}
        confirmText={t('myUI', 'clear') || 'Clear'}
        cancelText={t('myUI', 'cancel') || 'Cancel'}
        variant="warning"
      />

      {/* Delete Layout Confirmation */}
      <ConfirmModal
        isOpen={showDeleteConfirm}
        onClose={() => {
          setShowDeleteConfirm(false);
          setPendingDeleteId(null);
        }}
        onConfirm={confirmDeleteLayout}
        title={t('myUI', 'deleteLayoutTitle') || 'Delete Layout'}
        message={t('myUI', 'deleteLayoutConfirm') || 'Are you sure you want to delete this layout? This action cannot be undone.'}
        confirmText={t('myUI', 'delete') || 'Delete'}
        cancelText={t('myUI', 'cancel') || 'Cancel'}
        variant="danger"
      />

      {/* Update Layout Confirmation */}
      <ConfirmModal
        isOpen={showUpdateConfirm}
        onClose={() => {
          setShowUpdateConfirm(false);
          setPendingUpdateLayout(null);
        }}
        onConfirm={confirmUpdateLayoutData}
        title={t('myUI', 'updateLayoutTitle') || 'Update Layout'}
        message={t('myUI', 'updateLayoutConfirm') || 'Are you sure you want to update this layout with the current widgets?'}
        confirmText={t('myUI', 'update') || 'Update'}
        cancelText={t('myUI', 'cancel') || 'Cancel'}
        variant="warning"
      />
    </div>
  );
}
