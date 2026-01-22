import { NextRequest, NextResponse } from 'next/server';

export async function GET(req: NextRequest) {
  try {
    const searchParams = req.nextUrl.searchParams;
    const latStr = searchParams.get('lat');
    const lonStr = searchParams.get('lon');
    const altitudeStr = searchParams.get('altitude') || '0';

    if (!latStr || !lonStr) {
      return NextResponse.json(
        { error: 'Latitude and longitude are required' },
        { status: 400 }
      );
    }

    // Validate coordinates are valid numbers within range
    const lat = parseFloat(latStr);
    const lon = parseFloat(lonStr);
    const altitude = parseFloat(altitudeStr);

    if (!Number.isFinite(lat) || !Number.isFinite(lon) || !Number.isFinite(altitude)) {
      return NextResponse.json(
        { error: 'Invalid coordinate format' },
        { status: 400 }
      );
    }

    if (lat < -90 || lat > 90 || lon < -180 || lon > 180) {
      return NextResponse.json(
        { error: 'Coordinates out of valid range' },
        { status: 400 }
      );
    }

    // MET Norway API endpoint
    const metApiUrl = `https://api.met.no/weatherapi/locationforecast/2.0/complete?lat=${lat}&lon=${lon}&altitude=${altitude}`;

    // MET Norway requires a User-Agent header with contact information
    const response = await fetch(metApiUrl, {
      headers: {
        'User-Agent': 'BotBot Robotics Dashboard (https://github.com/botbotrobotics)',
        'Accept': 'application/json',
      },
      // Cache for 5 minutes
      next: { revalidate: 300 }
    });

    if (!response.ok) {
      throw new Error(`MET Norway API returned ${response.status}`);
    }

    const data = await response.json();

    // Return the weather data
    return NextResponse.json(data, {
      headers: {
        'Cache-Control': 'public, s-maxage=300, stale-while-revalidate=600',
      },
    });

  } catch (error) {
    console.error('Weather API error:', error);
    return NextResponse.json(
      { error: 'Failed to fetch weather data' },
      { status: 500 }
    );
  }
}