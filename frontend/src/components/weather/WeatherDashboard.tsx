'use client';

import { useState, useEffect, useCallback } from 'react';
import {
  CloudRain,
  Sun,
  Cloud,
  CloudSnow,
  Wind,
  Thermometer,
  Eye,
  Droplets,
  AlertTriangle,
  CheckCircle,
  XCircle,
  Info,
  MapPin,
  Clock,
  TrendingUp,
  TrendingDown,
  Gauge,
  Sunrise,
  Sunset,
  CloudDrizzle,
  CloudLightning,
  Navigation,
  Activity,
  Waves,
  Zap,
  Moon,
  CloudFog,
  Snowflake,
  RefreshCw,
  Umbrella,
  ArrowUp,
  ArrowDown
} from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';

interface WeatherData {
  properties: {
    timeseries: Array<{
      time: string;
      data: {
        instant: {
          details: {
            air_pressure_at_sea_level?: number;
            air_temperature?: number;
            cloud_area_fraction?: number;
            cloud_area_fraction_high?: number;
            cloud_area_fraction_low?: number;
            cloud_area_fraction_medium?: number;
            dew_point_temperature?: number;
            relative_humidity?: number;
            wind_from_direction?: number;
            wind_speed?: number;
            wind_speed_of_gust?: number;
            precipitation_amount?: number;
            fog_area_fraction?: number;
            ultraviolet_index_clear_sky?: number;
          };
        };
        next_1_hours?: {
          summary: {
            symbol_code: string;
          };
          details: {
            precipitation_amount?: number;
            precipitation_amount_max?: number;
            precipitation_amount_min?: number;
            probability_of_precipitation?: number;
            probability_of_thunder?: number;
          };
        };
        next_6_hours?: {
          summary: {
            symbol_code: string;
          };
          details: {
            air_temperature_max?: number;
            air_temperature_min?: number;
            precipitation_amount?: number;
            probability_of_precipitation?: number;
          };
        };
        next_12_hours?: {
          summary: {
            symbol_code: string;
          };
          details: {
            probability_of_precipitation?: number;
          };
        };
      };
    }>;
  };
}

interface LocationData {
  latitude: number;
  longitude: number;
  altitude?: number;
  placeName?: string;
}

interface DeploymentStatus {
  safe: boolean;
  warnings: string[];
  score: number;
}

// Wind direction compass component
const WindCompass: React.FC<{ direction: number; speed: number; gust?: number; useMetric: boolean }> = ({ direction, speed, gust, useMetric }) => {
  const cardinalDirections = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
  const getCardinal = (deg: number) => {
    const index = Math.round(deg / 45) % 8;
    return cardinalDirections[index];
  };

  const formatSpeed = (s: number) => useMetric ? s.toFixed(1) : (s * 2.237).toFixed(1);
  const unit = useMetric ? 'm/s' : 'mph';

  return (
    <div className="relative w-32 h-32 mx-auto">
      {/* Compass circle */}
      <div className="absolute inset-0 rounded-full bg-gradient-to-br from-blue-100 to-blue-200 dark:from-blue-900/30 dark:to-blue-800/30 shadow-inner">
        {/* Cardinal directions */}
        {cardinalDirections.map((dir, i) => {
          const angle = i * 45;
          const isActive = Math.abs((direction - angle + 360) % 360) < 22.5 ||
                          Math.abs((direction - angle - 360) % 360) < 22.5;
          return (
            <div
              key={dir}
              className={`absolute text-xs font-bold transition-all ${
                isActive ? 'text-blue-700 dark:text-blue-400 scale-110' : 'text-gray-400 dark:text-gray-500'
              }`}
              style={{
                left: '50%',
                top: '50%',
                transform: `translate(-50%, -50%) rotate(${angle}deg) translateY(-48px) rotate(-${angle}deg)`
              }}
            >
              {dir}
            </div>
          );
        })}

        {/* Wind arrow */}
        <motion.div
          className="absolute inset-0 flex items-center justify-center"
          animate={{ rotate: direction }}
          transition={{ type: "spring", stiffness: 100 }}
        >
          <Navigation className="w-8 h-8 text-blue-600 dark:text-blue-400 fill-current -rotate-45" />
        </motion.div>

        {/* Center info */}
        <div className="absolute inset-0 flex flex-col items-center justify-center">
          <span className="text-lg font-bold text-gray-800 dark:text-white">{formatSpeed(speed)}</span>
          <span className="text-xs text-gray-600 dark:text-gray-400">{unit}</span>
          {gust && gust > speed && (
            <span className="text-xs text-orange-600 dark:text-orange-400">↑{formatSpeed(gust)}</span>
          )}
        </div>
      </div>

      <div className="text-center mt-2">
        <span className="text-sm font-medium text-gray-700 dark:text-gray-300">
          {getCardinal(direction)} • {direction}°
        </span>
      </div>
    </div>
  );
};

// UV Index indicator
const UVIndexIndicator: React.FC<{ index: number }> = ({ index }) => {
  const getUVLevel = (uv: number) => {
    if (uv <= 2) return { level: 'Low', color: 'green', bg: 'from-green-400 to-green-500' };
    if (uv <= 5) return { level: 'Moderate', color: 'yellow', bg: 'from-yellow-400 to-yellow-500' };
    if (uv <= 7) return { level: 'High', color: 'orange', bg: 'from-orange-400 to-orange-500' };
    if (uv <= 10) return { level: 'Very High', color: 'red', bg: 'from-red-400 to-red-500' };
    return { level: 'Extreme', color: 'purple', bg: 'from-purple-400 to-purple-500' };
  };

  const uvInfo = getUVLevel(index);

  return (
    <div className="relative h-8 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden">
      <div className="absolute inset-0 flex">
        {[...Array(11)].map((_, i) => (
          <div
            key={i}
            className={`flex-1 ${i === 0 ? 'rounded-l-full' : ''} ${i === 10 ? 'rounded-r-full' : ''}`}
            style={{
              background: i <= 2 ? '#22c55e' : i <= 5 ? '#eab308' : i <= 7 ? '#f97316' : i <= 10 ? '#ef4444' : '#a855f7'
            }}
          />
        ))}
      </div>
      <motion.div
        className="absolute top-1/2 -translate-y-1/2 w-1 h-6 bg-white rounded-full shadow-lg"
        animate={{ left: `${(index / 11) * 100}%` }}
        transition={{ type: "spring", stiffness: 100 }}
      />
      <div className="absolute inset-0 flex items-center justify-center">
        <span className="text-xs font-bold text-white drop-shadow-md">{index} - {uvInfo.level}</span>
      </div>
    </div>
  );
};

const WeatherDashboard: React.FC = () => {
  const [weatherData, setWeatherData] = useState<WeatherData | null>(null);
  const [location, setLocation] = useState<LocationData | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [lastUpdate, setLastUpdate] = useState<Date | null>(null);
  const [deploymentStatus, setDeploymentStatus] = useState<DeploymentStatus | null>(null);
  const [isRefreshing, setIsRefreshing] = useState(false);
  const [useMetric, setUseMetric] = useState(true);

  // Unit conversion functions
  const celsiusToFahrenheit = (celsius: number): number => (celsius * 9/5) + 32;
  const metersPerSecToMph = (mps: number): number => mps * 2.237;
  const mmToInches = (mm: number): number => mm * 0.0393701;
  const hpaToInHg = (hpa: number): number => hpa * 0.02953;

  const formatTemp = (temp: number | undefined): string => {
    if (temp === undefined) return '--';
    return useMetric ? `${temp.toFixed(1)}°C` : `${celsiusToFahrenheit(temp).toFixed(1)}°F`;
  };

  const formatTempShort = (temp: number | undefined): string => {
    if (temp === undefined) return '--';
    return useMetric ? `${temp.toFixed(0)}°` : `${celsiusToFahrenheit(temp).toFixed(0)}°`;
  };

  const formatTempValue = (temp: number | undefined): string => {
    if (temp === undefined) return '--';
    return useMetric ? temp.toFixed(1) : celsiusToFahrenheit(temp).toFixed(1);
  };

  const formatWind = (speed: number | undefined): string => {
    if (speed === undefined) return '--';
    return useMetric ? `${speed.toFixed(1)} m/s` : `${metersPerSecToMph(speed).toFixed(1)} mph`;
  };

  const formatWindShort = (speed: number | undefined): string => {
    if (speed === undefined) return '--';
    return useMetric ? speed.toFixed(0) : metersPerSecToMph(speed).toFixed(0);
  };

  const formatWindValue = (speed: number | undefined): string => {
    if (speed === undefined) return '--';
    return useMetric ? speed.toFixed(1) : metersPerSecToMph(speed).toFixed(1);
  };

  const formatPrecip = (amount: number | undefined): string => {
    if (amount === undefined) return '--';
    return useMetric ? `${amount.toFixed(1)}mm` : `${mmToInches(amount).toFixed(2)}"`;
  };

  const formatPrecipShort = (amount: number | undefined): string => {
    if (amount === undefined) return '--';
    return useMetric ? `${amount.toFixed(0)}mm` : `${mmToInches(amount).toFixed(1)}"`;
  };

  const formatPrecipValue = (amount: number | undefined): string => {
    if (amount === undefined) return '--';
    return useMetric ? amount.toFixed(1) : mmToInches(amount).toFixed(2);
  };

  const formatPressure = (pressure: number | undefined): string => {
    if (pressure === undefined) return '--';
    return useMetric ? `${pressure.toFixed(0)} hPa` : `${hpaToInHg(pressure).toFixed(2)} inHg`;
  };

  const getWindUnit = (): string => useMetric ? 'm/s' : 'mph';
  const getTempUnit = (): string => useMetric ? '°C' : '°F';
  const getPrecipUnit = (): string => useMetric ? 'mm' : 'in';

  // Get user's location
  useEffect(() => {
    if (navigator.geolocation) {
      navigator.geolocation.getCurrentPosition(
        async (position) => {
          const loc: LocationData = {
            latitude: position.coords.latitude,
            longitude: position.coords.longitude,
            altitude: position.coords.altitude || 0,
          };

          try {
            const response = await fetch(
              `https://nominatim.openstreetmap.org/reverse?format=json&lat=${loc.latitude}&lon=${loc.longitude}`,
              {
                headers: {
                  'User-Agent': 'BotBot Robotics Dashboard'
                }
              }
            );
            const data = await response.json();
            loc.placeName = data.address?.city || data.address?.town || data.address?.village || 'Unknown Location';
          } catch (err) {
            loc.placeName = `${loc.latitude.toFixed(4)}, ${loc.longitude.toFixed(4)}`;
          }

          setLocation(loc);
        },
        (err) => {
          setError('Unable to get location. Please enable location services.');
          setLoading(false);
        }
      );
    } else {
      setError('Geolocation is not supported by your browser.');
      setLoading(false);
    }
  }, []);

  // Fetch weather data
  const fetchWeatherData = useCallback(async () => {
    if (!location) return;

    try {
      setIsRefreshing(true);
      const response = await fetch(
        `/api/weather?lat=${location.latitude}&lon=${location.longitude}&altitude=${location.altitude || 0}`,
        {
          headers: {
            'Accept': 'application/json',
          }
        }
      );

      if (!response.ok) {
        throw new Error('Failed to fetch weather data');
      }

      const data: WeatherData = await response.json();
      setWeatherData(data);
      setLastUpdate(new Date());

      if (data.properties.timeseries.length > 0) {
        const current = data.properties.timeseries[0].data;
        const status = calculateDeploymentStatus(current, useMetric);
        setDeploymentStatus(status);
      }

      setError(null);
    } catch (err) {
      setError('Failed to fetch weather data. Please try again.');
      console.error('Weather fetch error:', err);
    } finally {
      setLoading(false);
      setIsRefreshing(false);
    }
  }, [location]);

  useEffect(() => {
    if (location) {
      fetchWeatherData();
      const interval = setInterval(fetchWeatherData, 5 * 60 * 1000);
      return () => clearInterval(interval);
    }
  }, [location, fetchWeatherData]);

  // Update deployment status when units change
  useEffect(() => {
    if (weatherData && weatherData.properties.timeseries.length > 0) {
      const current = weatherData.properties.timeseries[0].data;
      const status = calculateDeploymentStatus(current, useMetric);
      setDeploymentStatus(status);
    }
  }, [useMetric, weatherData]);

  const calculateDeploymentStatus = (data: any, isMetric: boolean): DeploymentStatus => {
    const warnings: string[] = [];
    let score = 100;

    const temp = data.instant.details.air_temperature;
    const windSpeed = data.instant.details.wind_speed;
    const windGust = data.instant.details.wind_speed_of_gust;
    const humidity = data.instant.details.relative_humidity;
    const precipitation = data.next_1_hours?.details?.precipitation_amount || 0;
    const precipitationProb = data.next_1_hours?.details?.probability_of_precipitation || 0;
    const thunderProb = data.next_1_hours?.details?.probability_of_thunder || 0;
    const visibility = data.instant.details.fog_area_fraction ? (100 - data.instant.details.fog_area_fraction) : 100;

    // Temperature checks
    if (temp !== undefined) {
      const tempDisplay = isMetric ? `${temp.toFixed(1)}°C` : `${celsiusToFahrenheit(temp).toFixed(1)}°F`;
      if (temp > 55) {
        warnings.push(`🔥 Extreme heat: ${tempDisplay} - Risk of overheating`);
        score -= 40;
      } else if (temp > 40) {
        warnings.push(`🌡️ High temperature: ${tempDisplay} - Monitor closely`);
        score -= 20;
      } else if (temp < -10) {
        warnings.push(`🥶 Extreme cold: ${tempDisplay} - Battery performance reduced`);
        score -= 30;
      } else if (temp < 0) {
        warnings.push(`❄️ Below freezing: ${tempDisplay} - Ice hazard`);
        score -= 15;
      }
    }

    // Precipitation and thunder checks
    if (thunderProb > 20) {
      warnings.push(`⚡ Thunder risk: ${thunderProb}% - Electrical hazard`);
      score -= 30;
    }
    if (precipitation > 0 || precipitationProb > 30) {
      const precipDisplay = isMetric ? `${precipitation.toFixed(1)}mm` : `${mmToInches(precipitation).toFixed(2)}"`;
      if (precipitation > 5) {
        warnings.push(`🌧️ Heavy rain: ${precipDisplay} - Not recommended`);
        score -= 35;
      } else if (precipitation > 1) {
        warnings.push(`💧 Light rain: ${precipDisplay} - Use caution`);
        score -= 20;
      } else if (precipitationProb > 50) {
        warnings.push(`☔ Rain probability: ${precipitationProb}%`);
        score -= 15;
      }
    }

    // Wind checks including gusts
    if (windGust !== undefined && windGust > 20) {
      const gustDisplay = isMetric ? `${windGust.toFixed(1)} m/s` : `${metersPerSecToMph(windGust).toFixed(1)} mph`;
      warnings.push(`💨 Strong gusts: ${gustDisplay} - Severe stability risk`);
      score -= 30;
    } else if (windSpeed !== undefined) {
      const windDisplay = isMetric ? `${windSpeed.toFixed(1)} m/s` : `${metersPerSecToMph(windSpeed).toFixed(1)} mph`;
      if (windSpeed > 15) {
        warnings.push(`🌬️ Strong winds: ${windDisplay} - Stability issues`);
        score -= 25;
      } else if (windSpeed > 10) {
        warnings.push(`🍃 Moderate winds: ${windDisplay} - May affect navigation`);
        score -= 10;
      }
    }

    // Visibility checks
    if (visibility < 70) {
      warnings.push(`🌫️ Poor visibility: ${visibility.toFixed(0)}% - Limited sensor range`);
      score -= 20;
    }

    // Time of day
    const hour = new Date().getHours();
    if (hour < 6 || hour > 20) {
      warnings.push('🌙 Low light conditions - Reduced camera effectiveness');
      score -= 10;
    }

    return {
      safe: score >= 70,
      warnings,
      score: Math.max(0, score)
    };
  };

  const getWeatherIcon = (symbolCode: string, size: string = 'w-8 h-8') => {
    const iconClass = `${size}`;

    if (symbolCode.includes('thunder')) return <Zap className={`${iconClass} text-purple-500`} />;
    if (symbolCode.includes('snow') || symbolCode.includes('sleet')) return <Snowflake className={`${iconClass} text-blue-300`} />;
    if (symbolCode.includes('rain')) {
      if (symbolCode.includes('heavy')) return <CloudRain className={`${iconClass} text-blue-600`} />;
      return <CloudDrizzle className={`${iconClass} text-blue-500`} />;
    }
    if (symbolCode.includes('fog')) return <CloudFog className={`${iconClass} text-gray-400`} />;
    if (symbolCode.includes('clear')) {
      const hour = new Date().getHours();
      if (hour >= 6 && hour <= 20) return <Sun className={`${iconClass} text-yellow-500`} />;
      return <Moon className={`${iconClass} text-indigo-400`} />;
    }
    if (symbolCode.includes('partly')) return <Cloud className={`${iconClass} text-gray-500`} />;
    if (symbolCode.includes('cloudy')) return <Cloud className={`${iconClass} text-gray-600`} />;
    return <Cloud className={`${iconClass} text-gray-500`} />;
  };

  const formatTime = (timeString: string) => {
    const date = new Date(timeString);
    return date.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit' });
  };

  const formatDate = (timeString: string) => {
    const date = new Date(timeString);
    return date.toLocaleDateString('en-US', { weekday: 'short', month: 'short', day: 'numeric' });
  };

  if (loading) {
    return (
      <div className="flex items-center justify-center h-96">
        <div className="relative">
          <div className="animate-spin rounded-full h-16 w-16 border-t-2 border-b-2 border-blue-500"></div>
          <Sun className="absolute inset-0 m-auto w-8 h-8 text-yellow-500 animate-pulse" />
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="bg-gradient-to-r from-red-50 to-red-100 dark:from-red-900/20 dark:to-red-800/20 p-6 rounded-xl shadow-lg border border-red-200 dark:border-red-800">
        <div className="flex items-center">
          <AlertTriangle className="w-6 h-6 text-red-500 mr-3" />
          <span className="text-red-700 dark:text-red-400 font-medium">{error}</span>
        </div>
      </div>
    );
  }

  if (!weatherData) return null;

  const currentWeather = weatherData.properties.timeseries[0];
  const hourlyForecast = weatherData.properties.timeseries.slice(0, 24);
  const dailyForecast = weatherData.properties.timeseries.filter((_, i) => i % 6 === 0).slice(0, 8);

  return (
    <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
      {/* Left Column */}
      <div className="space-y-6">
        {/* Deployment Status Card */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          className={`relative overflow-hidden p-6 rounded-2xl shadow-xl border-2 ${
            deploymentStatus?.safe
              ? 'bg-gradient-to-br from-green-50 to-emerald-100 dark:from-green-900/20 dark:to-emerald-900/30 border-green-400 dark:border-green-600'
              : 'bg-gradient-to-br from-amber-50 to-orange-100 dark:from-amber-900/20 dark:to-orange-900/30 border-amber-400 dark:border-amber-600'
          }`}
        >
          <div className="absolute top-0 right-0 w-32 h-32 transform translate-x-16 -translate-y-8">
            <div className={`w-full h-full rounded-full ${deploymentStatus?.safe ? 'bg-green-200/30' : 'bg-amber-200/30'} blur-3xl`} />
          </div>

          <div className="relative">
            <div className="flex items-center justify-between mb-4">
              <h2 className="text-xl font-bold flex items-center">
                {deploymentStatus?.safe ? (
                  <CheckCircle className="w-7 h-7 text-green-600 dark:text-green-400 mr-2" />
                ) : (
                  <AlertTriangle className="w-7 h-7 text-amber-600 dark:text-amber-400 mr-2" />
                )}
                Deployment Analysis
              </h2>
              <div className="flex flex-col items-end">
                <div className="flex items-center">
                  <Activity className="w-5 h-5 mr-2 text-gray-600 dark:text-gray-400" />
                  <span className="text-3xl font-bold">{deploymentStatus?.score}%</span>
                </div>
                <span className="text-xs text-gray-600 dark:text-gray-400">Safety Score</span>
              </div>
            </div>

            <div className="mb-4">
              <div className="h-3 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden">
                <motion.div
                  className={`h-full rounded-full ${
                    deploymentStatus?.safe
                      ? 'bg-gradient-to-r from-green-400 to-emerald-500'
                      : 'bg-gradient-to-r from-amber-400 to-orange-500'
                  }`}
                  initial={{ width: 0 }}
                  animate={{ width: `${deploymentStatus?.score}%` }}
                  transition={{ duration: 1, ease: "easeOut" }}
                />
              </div>
            </div>

            <div className="mb-4">
              <span className={`inline-flex items-center px-3 py-1 rounded-full text-sm font-semibold ${
                deploymentStatus?.safe
                  ? 'bg-green-200 dark:bg-green-800/50 text-green-800 dark:text-green-200'
                  : 'bg-amber-200 dark:bg-amber-800/50 text-amber-800 dark:text-amber-200'
              }`}>
                {deploymentStatus?.safe ? '✅ Safe to Deploy' : '⚠️ Deploy with Caution'}
              </span>
            </div>

            {deploymentStatus?.warnings && deploymentStatus.warnings.length > 0 && (
              <div className="space-y-2">
                <h3 className="font-semibold text-sm text-gray-700 dark:text-gray-300">Active Warnings:</h3>
                <div className="space-y-1 max-h-32 overflow-y-auto">
                  {deploymentStatus.warnings.map((warning, idx) => (
                    <motion.div
                      key={idx}
                      initial={{ opacity: 0, x: -20 }}
                      animate={{ opacity: 1, x: 0 }}
                      transition={{ delay: idx * 0.1 }}
                      className="text-sm text-gray-700 dark:text-gray-300 bg-white/50 dark:bg-black/20 rounded-lg px-3 py-1"
                    >
                      {warning}
                    </motion.div>
                  ))}
                </div>
              </div>
            )}
          </div>
        </motion.div>

        {/* Current Weather Card */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.1 }}
          className="bg-gradient-to-br from-white to-gray-50 dark:from-botbot-darker dark:to-botbot-dark rounded-2xl shadow-xl p-6 border border-gray-200 dark:border-gray-700"
        >
          <div className="flex justify-between items-start mb-6">
            <div>
              <h2 className="text-2xl font-bold text-gray-800 dark:text-white mb-2">Current Conditions</h2>
              <div className="flex items-center text-gray-600 dark:text-gray-400">
                <MapPin className="w-4 h-4 mr-1" />
                <span className="text-sm font-medium">{location?.placeName}</span>
              </div>
            </div>
            <div className="flex flex-col items-end gap-2">
              {/* Unit Toggle */}
              <div className="flex items-center gap-2 bg-gray-100 dark:bg-gray-800 rounded-lg p-1">
                <button
                  onClick={() => setUseMetric(true)}
                  className={`px-3 py-1 rounded-md text-xs font-medium transition-all ${
                    useMetric
                      ? 'bg-white dark:bg-gray-700 text-blue-600 dark:text-blue-400 shadow-sm'
                      : 'text-gray-600 dark:text-gray-400 hover:text-gray-800 dark:hover:text-gray-200'
                  }`}
                >
                  Metric
                </button>
                <button
                  onClick={() => setUseMetric(false)}
                  className={`px-3 py-1 rounded-md text-xs font-medium transition-all ${
                    !useMetric
                      ? 'bg-white dark:bg-gray-700 text-blue-600 dark:text-blue-400 shadow-sm'
                      : 'text-gray-600 dark:text-gray-400 hover:text-gray-800 dark:hover:text-gray-200'
                  }`}
                >
                  Imperial
                </button>
              </div>
              <div className="flex items-center gap-2">
                <button
                  onClick={() => !isRefreshing && fetchWeatherData()}
                  className="p-2 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors"
                  disabled={isRefreshing}
                >
                  <RefreshCw className={`w-4 h-4 text-gray-600 dark:text-gray-400 ${isRefreshing ? 'animate-spin' : ''}`} />
                </button>
                <div className="flex items-center text-gray-600 dark:text-gray-400">
                  <Clock className="w-4 h-4 mr-1" />
                  <span className="text-xs">
                    {lastUpdate?.toLocaleTimeString()}
                  </span>
                </div>
              </div>
            </div>
          </div>

          <div className="grid grid-cols-2 gap-4 mb-6">
            <div className="flex items-center justify-center">
              {currentWeather.data.next_1_hours && getWeatherIcon(currentWeather.data.next_1_hours.summary.symbol_code, 'w-24 h-24')}
            </div>
            <div className="flex flex-col justify-center">
              <div className="text-5xl font-bold text-gray-800 dark:text-white">
                {formatTempValue(currentWeather.data.instant.details.air_temperature)}°
              </div>
              <div className="text-sm text-gray-600 dark:text-gray-400 mt-2">
                Feels like {formatTempShort(
                  currentWeather.data.instant.details.air_temperature !== undefined &&
                  currentWeather.data.instant.details.wind_speed !== undefined
                    ? currentWeather.data.instant.details.air_temperature -
                      (currentWeather.data.instant.details.wind_speed * 0.7)
                    : undefined
                )}
              </div>
              {currentWeather.data.next_6_hours && (
                <div className="flex items-center gap-3 mt-2 text-sm">
                  <span className="flex items-center text-blue-600 dark:text-blue-400">
                    <ArrowDown className="w-3 h-3 mr-1" />
                    {formatTempShort(currentWeather.data.next_6_hours.details.air_temperature_min)}
                  </span>
                  <span className="flex items-center text-red-600 dark:text-red-400">
                    <ArrowUp className="w-3 h-3 mr-1" />
                    {formatTempShort(currentWeather.data.next_6_hours.details.air_temperature_max)}
                  </span>
                </div>
              )}
            </div>
          </div>

          <div className="grid grid-cols-2 gap-4">
            {/* Precipitation */}
            <div className="bg-gradient-to-br from-blue-50 to-cyan-50 dark:from-blue-900/30 dark:to-cyan-900/30 p-4 rounded-xl">
              <div className="flex items-center justify-between mb-2">
                <Umbrella className="w-5 h-5 text-blue-600" />
                <span className="text-xl font-bold text-blue-700 dark:text-blue-400">
                  {formatPrecipValue(currentWeather.data.next_1_hours?.details?.precipitation_amount || 0)}{getPrecipUnit()}
                </span>
              </div>
              <p className="text-xs text-gray-700 dark:text-gray-300">Next Hour</p>
              {currentWeather.data.next_1_hours?.details?.probability_of_precipitation !== undefined && (
                <div className="mt-2">
                  <div className="h-2 bg-blue-200 dark:bg-blue-800 rounded-full overflow-hidden">
                    <div
                      className="h-full bg-gradient-to-r from-blue-400 to-blue-600 rounded-full"
                      style={{ width: `${currentWeather.data.next_1_hours.details.probability_of_precipitation}%` }}
                    />
                  </div>
                  <span className="text-xs text-gray-600 dark:text-gray-400">
                    {currentWeather.data.next_1_hours.details.probability_of_precipitation}% chance
                  </span>
                </div>
              )}
            </div>

            {/* Humidity */}
            <div className="bg-gradient-to-br from-purple-50 to-indigo-50 dark:from-purple-900/30 dark:to-indigo-900/30 p-4 rounded-xl">
              <div className="flex items-center justify-between mb-2">
                <Droplets className="w-5 h-5 text-purple-600" />
                <span className="text-xl font-bold text-purple-700 dark:text-purple-400">
                  {currentWeather.data.instant.details.relative_humidity?.toFixed(0)}%
                </span>
              </div>
              <p className="text-xs text-gray-700 dark:text-gray-300">Humidity</p>
              {currentWeather.data.instant.details.dew_point_temperature !== undefined && (
                <p className="text-xs text-gray-600 dark:text-gray-400 mt-1">
                  Dew point: {formatTemp(currentWeather.data.instant.details.dew_point_temperature)}
                </p>
              )}
            </div>
          </div>
        </motion.div>

        {/* Hourly Forecast */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.2 }}
          className="bg-gradient-to-br from-white to-gray-50 dark:from-botbot-darker dark:to-botbot-dark rounded-2xl shadow-xl p-6 border border-gray-200 dark:border-gray-700"
        >
          <h3 className="text-xl font-bold text-gray-800 dark:text-white mb-4">24-Hour Forecast</h3>
          <div className="overflow-x-auto">
            <div className="flex gap-3 min-w-max pb-2">
              {hourlyForecast.map((hour, idx) => (
                <motion.div
                  key={idx}
                  initial={{ opacity: 0, y: 20 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ delay: idx * 0.02 }}
                  className="flex flex-col items-center p-3 bg-gradient-to-b from-gray-50 to-gray-100 dark:from-gray-800 dark:to-gray-900 rounded-xl min-w-[90px] hover:shadow-lg transition-shadow"
                >
                  <span className="text-xs text-gray-600 dark:text-gray-400 mb-2 font-medium">
                    {formatTime(hour.time)}
                  </span>
                  {hour.data.next_1_hours && getWeatherIcon(hour.data.next_1_hours.summary.symbol_code)}
                  <span className="text-sm font-bold mt-2 text-gray-800 dark:text-white">
                    {formatTempShort(hour.data.instant.details.air_temperature)}
                  </span>
                  {hour.data.next_1_hours?.details?.precipitation_amount !== undefined &&
                   hour.data.next_1_hours.details.precipitation_amount > 0 && (
                    <div className="flex items-center mt-1">
                      <Droplets className="w-3 h-3 mr-1 text-blue-500" />
                      <span className="text-xs text-blue-600 dark:text-blue-400">
                        {formatPrecipValue(hour.data.next_1_hours.details.precipitation_amount)}
                      </span>
                    </div>
                  )}
                  <div className="flex items-center mt-1">
                    <Wind className="w-3 h-3 mr-1 text-gray-500" />
                    <span className="text-xs text-gray-600 dark:text-gray-400">
                      {formatWindShort(hour.data.instant.details.wind_speed)}
                    </span>
                  </div>
                </motion.div>
              ))}
            </div>
          </div>
        </motion.div>
      </div>

      {/* Right Column */}
      <div className="space-y-6">
        {/* Wind & Pressure Card */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.15 }}
          className="bg-gradient-to-br from-white to-gray-50 dark:from-botbot-darker dark:to-botbot-dark rounded-2xl shadow-xl p-6 border border-gray-200 dark:border-gray-700"
        >
          <h3 className="text-xl font-bold text-gray-800 dark:text-white mb-6">Wind & Atmosphere</h3>

          <div className="grid grid-cols-2 gap-6">
            <div className="flex flex-col items-center">
              <WindCompass
                direction={currentWeather.data.instant.details.wind_from_direction || 0}
                speed={currentWeather.data.instant.details.wind_speed || 0}
                gust={currentWeather.data.instant.details.wind_speed_of_gust}
                useMetric={useMetric}
              />
            </div>

            <div className="space-y-4">
              <div className="bg-gradient-to-br from-green-50 to-emerald-50 dark:from-green-900/30 dark:to-emerald-900/30 p-4 rounded-xl">
                <div className="flex items-center justify-between mb-2">
                  <Gauge className="w-5 h-5 text-green-600" />
                  <span className="text-xl font-bold text-green-700 dark:text-green-400">
                    {formatPressure(currentWeather.data.instant.details.air_pressure_at_sea_level)}
                  </span>
                </div>
                <p className="text-xs text-gray-700 dark:text-gray-300">Pressure</p>
              </div>

              {currentWeather.data.instant.details.wind_speed_of_gust && (
                <div className="bg-gradient-to-br from-orange-50 to-amber-50 dark:from-orange-900/30 dark:to-amber-900/30 p-4 rounded-xl">
                  <div className="flex items-center justify-between mb-2">
                    <Waves className="w-5 h-5 text-orange-600" />
                    <span className="text-xl font-bold text-orange-700 dark:text-orange-400">
                      {formatWind(currentWeather.data.instant.details.wind_speed_of_gust)}
                    </span>
                  </div>
                  <p className="text-xs text-gray-700 dark:text-gray-300">Wind Gusts</p>
                </div>
              )}
            </div>
          </div>
        </motion.div>

        {/* UV Index & Visibility */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.2 }}
          className="bg-gradient-to-br from-white to-gray-50 dark:from-botbot-darker dark:to-botbot-dark rounded-2xl shadow-xl p-6 border border-gray-200 dark:border-gray-700"
        >
          <h3 className="text-xl font-bold text-gray-800 dark:text-white mb-4">Visibility & UV</h3>

          <div className="space-y-4">
            {currentWeather.data.instant.details.ultraviolet_index_clear_sky !== undefined && (
              <div>
                <div className="flex items-center justify-between mb-2">
                  <span className="text-sm font-medium text-gray-700 dark:text-gray-300">UV Index</span>
                  <Sun className="w-5 h-5 text-yellow-500" />
                </div>
                <UVIndexIndicator index={currentWeather.data.instant.details.ultraviolet_index_clear_sky} />
              </div>
            )}

            <div className="grid grid-cols-2 gap-4">
              <div className="bg-gradient-to-br from-sky-50 to-blue-50 dark:from-sky-900/30 dark:to-blue-900/30 p-4 rounded-xl">
                <div className="flex items-center justify-between mb-2">
                  <Eye className="w-5 h-5 text-sky-600" />
                  <span className="text-xl font-bold text-sky-700 dark:text-sky-400">
                    {currentWeather.data.instant.details.fog_area_fraction
                      ? `${(100 - currentWeather.data.instant.details.fog_area_fraction).toFixed(0)}%`
                      : '100%'}
                  </span>
                </div>
                <p className="text-xs text-gray-700 dark:text-gray-300">Visibility</p>
              </div>

              <div className="bg-gradient-to-br from-gray-50 to-slate-50 dark:from-gray-900/30 dark:to-slate-900/30 p-4 rounded-xl">
                <div className="flex items-center justify-between mb-2">
                  <Cloud className="w-5 h-5 text-gray-600" />
                  <span className="text-xl font-bold text-gray-700 dark:text-gray-400">
                    {currentWeather.data.instant.details.cloud_area_fraction?.toFixed(0)}%
                  </span>
                </div>
                <p className="text-xs text-gray-700 dark:text-gray-300">Cloud Cover</p>
              </div>
            </div>
          </div>
        </motion.div>

        {/* Cloud Layers */}
        {(currentWeather.data.instant.details.cloud_area_fraction_low !== undefined ||
          currentWeather.data.instant.details.cloud_area_fraction_medium !== undefined ||
          currentWeather.data.instant.details.cloud_area_fraction_high !== undefined) && (
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.25 }}
            className="bg-gradient-to-br from-white to-gray-50 dark:from-botbot-darker dark:to-botbot-dark rounded-2xl shadow-xl p-6 border border-gray-200 dark:border-gray-700"
          >
            <h3 className="text-xl font-bold text-gray-800 dark:text-white mb-4">Cloud Layers</h3>
            <div className="space-y-3">
              {currentWeather.data.instant.details.cloud_area_fraction_high !== undefined && (
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600 dark:text-gray-400">High altitude</span>
                  <div className="flex items-center">
                    <div className="w-32 h-2 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden mr-3">
                      <div
                        className="h-full bg-gradient-to-r from-gray-400 to-gray-500 rounded-full"
                        style={{ width: `${currentWeather.data.instant.details.cloud_area_fraction_high}%` }}
                      />
                    </div>
                    <span className="text-sm font-medium w-12 text-right">
                      {currentWeather.data.instant.details.cloud_area_fraction_high}%
                    </span>
                  </div>
                </div>
              )}
              {currentWeather.data.instant.details.cloud_area_fraction_medium !== undefined && (
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600 dark:text-gray-400">Medium altitude</span>
                  <div className="flex items-center">
                    <div className="w-32 h-2 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden mr-3">
                      <div
                        className="h-full bg-gradient-to-r from-gray-500 to-gray-600 rounded-full"
                        style={{ width: `${currentWeather.data.instant.details.cloud_area_fraction_medium}%` }}
                      />
                    </div>
                    <span className="text-sm font-medium w-12 text-right">
                      {currentWeather.data.instant.details.cloud_area_fraction_medium}%
                    </span>
                  </div>
                </div>
              )}
              {currentWeather.data.instant.details.cloud_area_fraction_low !== undefined && (
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600 dark:text-gray-400">Low altitude</span>
                  <div className="flex items-center">
                    <div className="w-32 h-2 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden mr-3">
                      <div
                        className="h-full bg-gradient-to-r from-gray-600 to-gray-700 rounded-full"
                        style={{ width: `${currentWeather.data.instant.details.cloud_area_fraction_low}%` }}
                      />
                    </div>
                    <span className="text-sm font-medium w-12 text-right">
                      {currentWeather.data.instant.details.cloud_area_fraction_low}%
                    </span>
                  </div>
                </div>
              )}
            </div>
          </motion.div>
        )}

        {/* 7-Day Forecast */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.3 }}
          className="bg-gradient-to-br from-white to-gray-50 dark:from-botbot-darker dark:to-botbot-dark rounded-2xl shadow-xl p-6 border border-gray-200 dark:border-gray-700"
        >
          <h3 className="text-xl font-bold text-gray-800 dark:text-white mb-4">7-Day Outlook</h3>
          <div className="space-y-2">
            {dailyForecast.map((day, idx) => (
              <motion.div
                key={idx}
                initial={{ opacity: 0, x: 20 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.3 + idx * 0.05 }}
                className="flex items-center justify-between p-3 bg-gradient-to-r from-gray-50 to-gray-100 dark:from-gray-800 dark:to-gray-900 rounded-xl hover:shadow-md transition-shadow"
              >
                <div className="flex items-center flex-1">
                  <span className="text-sm font-medium w-24 text-gray-700 dark:text-gray-300">
                    {formatDate(day.time)}
                  </span>
                  {day.data.next_6_hours && getWeatherIcon(day.data.next_6_hours.summary.symbol_code)}
                </div>

                <div className="flex items-center gap-3">
                  {day.data.next_6_hours?.details?.precipitation_amount !== undefined &&
                   day.data.next_6_hours.details.precipitation_amount > 0 && (
                    <div className="flex items-center">
                      <Droplets className="w-4 h-4 mr-1 text-blue-500" />
                      <span className="text-sm text-blue-600 dark:text-blue-400">
                        {formatPrecipShort(day.data.next_6_hours.details.precipitation_amount)}
                      </span>
                    </div>
                  )}

                  <div className="flex items-center">
                    <Wind className="w-4 h-4 mr-1 text-gray-500" />
                    <span className="text-sm text-gray-600 dark:text-gray-400">
                      {formatWindShort(day.data.instant.details.wind_speed)}
                    </span>
                  </div>

                  <div className="flex items-center gap-1">
                    <span className="text-sm font-semibold text-blue-600 dark:text-blue-400">
                      {formatTempShort(day.data.next_6_hours?.details?.air_temperature_min)}
                    </span>
                    <span className="text-gray-400">/</span>
                    <span className="text-sm font-semibold text-red-600 dark:text-red-400">
                      {formatTempShort(day.data.next_6_hours?.details?.air_temperature_max)}
                    </span>
                  </div>
                </div>
              </motion.div>
            ))}
          </div>
        </motion.div>
      </div>

      {/* Full-width API Attribution */}
      <motion.div
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        transition={{ delay: 0.4 }}
        className="lg:col-span-2 bg-gradient-to-r from-gray-50 to-gray-100 dark:from-gray-800/50 dark:to-gray-900/50 rounded-xl p-4 border border-gray-200 dark:border-gray-700"
      >
        <div className="flex items-center text-sm text-gray-600 dark:text-gray-400">
          <Info className="w-4 h-4 mr-2 flex-shrink-0" />
          <span>
            Weather data provided by MET Norway API. Location determined using browser's Geolocation API.
            Only your coordinates are shared with weather services to fetch local conditions. Data updates every 5 minutes.
          </span>
        </div>
      </motion.div>
    </div>
  );
};

export default WeatherDashboard;