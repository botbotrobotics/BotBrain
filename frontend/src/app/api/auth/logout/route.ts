import { createServerClient, type CookieOptions } from '@supabase/ssr'
import { cookies } from 'next/headers'
import { NextResponse } from 'next/server'

export async function POST(request: Request) {
  const cookieStore = await cookies()

  const supabase = createServerClient(
    process.env.NEXT_PUBLIC_SUPABASE_URL!,
    process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY!,
    {
      cookies: {
        get(name: string) {
          return cookieStore.get(name)?.value
        },
        set(name: string, value: string, options: CookieOptions) {
          cookieStore.set({ name, value, ...options })
        },
        remove(name: string, options: CookieOptions) {
          cookieStore.set({ name, value: '', ...options })
        },
      },
    }
  )

  // Get the current user before signing out
  const { data: { user } } = await supabase.auth.getUser()
  
  if (user) {
    // Log the logout event
    try {
      const ip = request.headers.get('x-forwarded-for') || request.headers.get('x-real-ip') || 'unknown'
      const userAgent = request.headers.get('user-agent') || 'unknown'
      
      await supabase
        .from('audit_logs')
        .insert({
          user_id: user.id,
          event_type: 'auth',
          event_action: 'logout',
          event_details: {},
          ip_address: ip.split(',')[0].trim(),
          user_agent: userAgent,
        })
    } catch (logError) {
      console.error('Failed to log logout event:', logError)
    }
  }

  const { error } = await supabase.auth.signOut()

  if (error) {
    return NextResponse.json({ error: error.message }, { status: 500 })
  }

  const response = NextResponse.json({ success: true })
  
  // Clear remember-me cookie
  response.cookies.delete('remember-me')
  
  return response
} 