'use client'

import { createContext, useContext, useEffect, useState } from 'react'
import { User, SupabaseClient } from '@supabase/supabase-js'
import { createSupabaseClient } from '@/utils/supabase-client'
import { useRouter } from 'next/navigation'

type SupabaseContextType = {
  user: User | null
  loading: boolean
  signOut: () => Promise<void>
  supabase: SupabaseClient | null
}

const SupabaseContext = createContext<SupabaseContextType | null>(null)

export function SupabaseProvider({ children }: { children: React.ReactNode }) {
  const [user, setUser] = useState<User | null>(null)
  const [loading, setLoading] = useState(true)
  const [supabase, setSupabase] = useState<SupabaseClient | null>(null)
  const router = useRouter()
  
  // Initialize Supabase client only on the client side
  useEffect(() => {
    try {
      const client = createSupabaseClient()
      setSupabase(client)
    } catch (error) {
      console.warn('Failed to create Supabase client:', error)
      setLoading(false)
    }
  }, [])

  useEffect(() => {
    if (!supabase) {
      return
    }

    let isMounted = true

    // Get initial session
    supabase.auth.getSession().then(({ data: { session } }) => {
      if (isMounted) {
        setUser(session?.user ?? null)
        setLoading(false)
      }
    })

    // Listen for auth changes
    const { data: { subscription } } = supabase.auth.onAuthStateChange(async (_event, session) => {
      if (isMounted) {
        setUser(session?.user ?? null)
        
        // Only set loading to false after the user state has been updated
        if (_event === 'SIGNED_IN') {
          // Small delay to ensure state propagation
          setTimeout(() => {
            if (isMounted) {
              setLoading(false)
            }
          }, 100)
        } else {
          setLoading(false)
        }
        
        // If user logs out, redirect to login
        if (!session && _event === 'SIGNED_OUT') {
          router.push('/')
        }
      }
    })

    return () => {
      isMounted = false
      subscription.unsubscribe()
    }
  }, [router, supabase])

  const signOut = async () => {
    if (!supabase) return
    await fetch('/api/auth/logout', { method: 'POST' })
    await supabase.auth.signOut()
    router.push('/')
  }

  return (
    <SupabaseContext.Provider value={{ user, loading, signOut, supabase }}>
      {children}
    </SupabaseContext.Provider>
  )
}

export const useSupabase = () => {
  const context = useContext(SupabaseContext)
  if (!context) {
    throw new Error('useSupabase must be used within a SupabaseProvider')
  }
  return context
} 