"use client"

import type React from "react"

import { createContext, useContext, useReducer } from "react"

export interface Notification {
  id: string
  title: string
  message: string
  type: "info" | "success" | "warning" | "error"
  timestamp: Date
  read: boolean
}

interface NotificationsState {
  notifications: Notification[]
}

type NotificationsAction =
  | { type: "ADD_NOTIFICATION"; payload: Omit<Notification, "id" | "timestamp" | "read"> }
  | { type: "REMOVE_NOTIFICATION"; payload: string }
  | { type: "MARK_AS_READ"; payload: string }
  | { type: "CLEAR_ALL" }

const NotificationsContext = createContext<{
  state: NotificationsState
  dispatch: React.Dispatch<NotificationsAction>
} | null>(null)

function notificationsReducer(state: NotificationsState, action: NotificationsAction): NotificationsState {
  switch (action.type) {
    case "ADD_NOTIFICATION":
      return {
        ...state,
        notifications: [
          {
            ...action.payload,
            id: Math.random().toString(36).substring(7),
            timestamp: new Date(),
            read: false,
          },
          ...state.notifications,
        ],
      }
    case "REMOVE_NOTIFICATION":
      return {
        ...state,
        notifications: state.notifications.filter((notification) => notification.id !== action.payload),
      }
    case "MARK_AS_READ":
      return {
        ...state,
        notifications: state.notifications.map((notification) =>
          notification.id === action.payload ? { ...notification, read: true } : notification,
        ),
      }
    case "CLEAR_ALL":
      return {
        ...state,
        notifications: [],
      }
    default:
      return state
  }
}

export function NotificationsProvider({ children }: { children: React.ReactNode }) {
  const [state, dispatch] = useReducer(notificationsReducer, {
    notifications: [],
  })

  return <NotificationsContext.Provider value={{ state, dispatch }}>{children}</NotificationsContext.Provider>
}

export function useNotifications() {
  const context = useContext(NotificationsContext)
  if (!context) {
    throw new Error("useNotifications must be used within a NotificationsProvider")
  }
  return context
}

