import { createServerClient, type CookieOptions } from '@supabase/ssr'
import { cookies } from 'next/headers'
import { NextResponse } from 'next/server'

export async function POST(request: Request) {
  const { email, password, remember } = await request.json()

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

  const { data, error } = await supabase.auth.signInWithPassword({
    email,
    password,
  })

  if (error) {
    return NextResponse.json({ error: error.message }, { status: 401 })
  }

  if (data.session) {
    // Log successful login
    try {
      const ip = request.headers.get('x-forwarded-for') || request.headers.get('x-real-ip') || 'unknown'
      const userAgent = request.headers.get('user-agent') || 'unknown'
      
      await supabase
        .from('audit_logs')
        .insert({
          user_id: data.user.id,
          event_type: 'auth',
          event_action: 'login',
          event_details: { 
            email,
            remember_me: remember || false 
          },
          ip_address: ip.split(',')[0].trim(), // Get first IP if multiple
          user_agent: userAgent,
        })
    } catch (logError) {
      console.error('Failed to log login event:', logError)
      // Don't fail the login if logging fails
    }

    const response = NextResponse.json({ 
      user: data.user, 
      session: data.session,
      success: true 
    })
    
    // Set custom cookie for remember me functionality
    if (remember) {
      response.cookies.set('remember-me', 'true', {
        httpOnly: true,
        secure: process.env.NODE_ENV === 'production',
        sameSite: 'strict',
        maxAge: 30 * 24 * 60 * 60, // 30 days
      })
    }
    
    return response
  }

  return NextResponse.json({ error: 'Login failed' }, { status: 401 })
} 