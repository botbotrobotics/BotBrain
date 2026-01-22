'use client';

import React, { createContext, useContext, useState, useEffect } from 'react';
import { auditLogger } from '@/utils/audit-logger';

// Add a simple uuid generator
export function generateId() {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function (c) {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

export type WidgetType =
  | 'camera'
  | 'gauge'
  | 'visualization3d'
  | 'info'
  | 'chat'
  | 'button'
  | 'buttonGroup'
  | 'sidewaysgauge'
  | 'joystick'
  | 'audio'
  | 'microphone'
  | 'map'
  | 'aiStream'
  | 'recentDetections'
  | 'ttsPresets'
  | 'mapsManagement'
  | 'soundClips'
  | 'recorder'
  | 'delivery'
  | 'missions';

export interface Widget {
  id: string;
  type: WidgetType;
  position: { x: number; y: number };  // Made position required
  size: { width: number; height: number };
  props?: Record<string, any>;
}

interface DashboardContextType {
  widgets: Widget[];
  addWidget: (type: WidgetType, initialProps?: Record<string, any>) => void;
  removeWidget: (id: string) => void;
  updateWidgetPosition: (
    id: string,
    position: { x: number; y: number }
  ) => void;
  updateWidgetSize: (
    id: string,
    size: { width: number; height: number }
  ) => void;
  updateWidgetProps: (id: string, props: Record<string, any>) => void;
  clearWidgets: () => void;
  saveLayout: () => boolean;
  loadLayout: () => boolean;
}

const DashboardContext = createContext<DashboardContextType | undefined>(
  undefined
);

const LOCAL_STORAGE_KEY = 'dashboard-widgets';

export function DashboardProvider({ children }: { children: React.ReactNode }) {
  const [widgets, setWidgets] = useState<Widget[]>([]);

  // Load widgets from localStorage on initial render
  useEffect(() => {
    loadSavedWidgets();
  }, []);

  // Auto-save widgets to localStorage whenever they change
  useEffect(() => {
    if (widgets.length > 0) {
      try {
        localStorage.setItem(LOCAL_STORAGE_KEY, JSON.stringify(widgets));
      } catch (error) {
        console.error('Failed to auto-save widgets to localStorage', error);
      }
    }
  }, [widgets]);

  const loadSavedWidgets = () => {
    try {
      const savedWidgets = localStorage.getItem(LOCAL_STORAGE_KEY);
      if (savedWidgets) {
        const parsed: Widget[] = JSON.parse(savedWidgets);
        // Deduplicate widgets by ID to prevent React key errors
        const uniqueWidgets = parsed.filter(
          (widget, index, self) =>
            index === self.findIndex((w) => w.id === widget.id)
        );
        // If duplicates were found, save the cleaned version
        if (uniqueWidgets.length !== parsed.length) {
          console.warn('Removed duplicate widgets from saved layout');
          localStorage.setItem(LOCAL_STORAGE_KEY, JSON.stringify(uniqueWidgets));
        }
        setWidgets(uniqueWidgets);
        return true;
      }
      return false;
    } catch (error) {
      console.error('Failed to load widgets from localStorage', error);
    }

    return false;
  };

  const addWidget = (type: WidgetType, widgetData?: Record<string, any>) => {
    // Default position and size
    let defaultPosition = { x: 50, y: 50 };
    let defaultSize = { width: 400, height: 300 };
    
    // If no position provided, generate random position
    if (!widgetData?.position) {
      const container = document.querySelector('[data-dashboard-container]');
      if (container) {
        const rect = container.getBoundingClientRect();
        const maxX = Math.max(0, rect.width - defaultSize.width);
        const maxY = Math.max(0, rect.height - defaultSize.height);
        defaultPosition = {
          x: Math.floor(Math.random() * maxX),
          y: Math.floor(Math.random() * maxY)
        };
      }
    }

    const newWidget: Widget = {
      id: widgetData?.id ?? generateId(),
      type,
      position: widgetData?.position ?? defaultPosition,
      size: widgetData?.size ?? defaultSize,
      props: widgetData?.props,
    };

    setWidgets((prev) => [...prev, newWidget]);
    
    // Log widget addition
    auditLogger.log({
      event_type: 'system',
      event_action: 'settings_updated',
      event_details: {
        action: 'widget_added',
        widget_type: type,
        widget_id: newWidget.id,
        widget_props: widgetData?.props
      }
    });
  };

  const removeWidget = (id: string) => {
    const widgetToRemove = widgets.find(w => w.id === id);
    setWidgets((prev) => prev.filter((widget) => widget.id !== id));
    
    // Log widget removal
    if (widgetToRemove) {
      auditLogger.log({
        event_type: 'system',
        event_action: 'settings_updated',
        event_details: {
          action: 'widget_removed',
          widget_type: widgetToRemove.type,
          widget_id: id
        }
      });
    }
  };

  const updateWidgetPosition = (
    id: string,
    position: { x: number; y: number }
  ) => {
    setWidgets((prev) =>
      prev.map((widget) =>
        widget.id === id ? { ...widget, position } : widget
      )
    );
  };

  const updateWidgetSize = (
    id: string,
    size: { width: number; height: number }
  ) => {
    setWidgets((prev) =>
      prev.map((widget) => (widget.id === id ? { ...widget, size } : widget))
    );
  };

  const updateWidgetProps = (id: string, props: Record<string, any>) => {
    setWidgets((prev) =>
      prev.map((widget) =>
        widget.id === id
          ? { ...widget, props: { ...widget.props, ...props } }
          : widget
      )
    );
  };

  const clearWidgets = () => {
    setWidgets([]);
  };

  const saveLayout = () => {
    try {
      localStorage.setItem(LOCAL_STORAGE_KEY, JSON.stringify(widgets));
      return true;
    } catch (error) {
      console.error('Failed to save widgets to localStorage', error);
    }
    return false;
  };

  const loadLayout = () => {
    return loadSavedWidgets();
  };

  return (
    <DashboardContext.Provider
      value={{
        widgets,
        addWidget,
        removeWidget,
        updateWidgetPosition,
        updateWidgetSize,
        updateWidgetProps,
        clearWidgets,
        saveLayout,
        loadLayout,
      }}
    >
      {children}
    </DashboardContext.Provider>
  );
}

export function useDashboard() {
  const context = useContext(DashboardContext);
  if (context === undefined) {
    throw new Error('useDashboard must be used within a DashboardProvider');
  }
  return context;
}
