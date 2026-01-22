import { NextRequest, NextResponse } from 'next/server';
import { createSupabaseServerClient } from '@/utils/supabase-server';

export async function middleware(request: NextRequest) {
  const path = request.nextUrl.pathname;
  
  // Create Supabase client
  const { supabase, response } = createSupabaseServerClient(request);
  
  // Get the session
  const { data: { session } } = await supabase.auth.getSession();

  // Define protected and public paths
  const isAuthRoute = path === '/';
  const isProtectedRoute = path !== '/';

  // If on login page and authenticated, redirect to dashboard
  if (isAuthRoute && session) {
    const redirectResponse = NextResponse.redirect(new URL('/dashboard', request.url));
    // Set a header to indicate fresh login for client-side handling
    redirectResponse.headers.set('x-auth-redirect', 'true');
    return redirectResponse;
  }

  // If trying to access protected route without session, redirect to login
  if (isProtectedRoute && !session) {
    return NextResponse.redirect(new URL('/', request.url));
  }

  return response;
}

// Configure which routes should be processed by this middleware
export const config = {
  matcher: [
    /*
     * Match all request paths except for:
     * - api routes
     * - _next (Next.js internals)
     * - static files (images, etc.)
     */
    '/((?!api|_next/static|_next/image|.*\\.png$|.*\\.jpg$|favicon.ico).*)',
  ],
};

