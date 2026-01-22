'use client';

import { useState, useEffect, useRef } from 'react';
import { BatteryState } from '@/interfaces/ros/BatteryState';

export interface BatteryTimeEstimate {
  remainingMinutes: number;
  remainingHours: number;
  formattedTime: string;
  dischargeRate: number; // percentage per hour
  confidence: 'high' | 'medium' | 'low' | 'calculating';
  estimatedShutdownTime: Date | null;
  isCharging: boolean;
  timeToFullCharge: number | null; // minutes
  currentVoltage: number;
  currentAmperage: number;
  lastTechnicalUpdate: number;
}

interface BatterySnapshot {
  percentage: number;
  timestamp: number;
  voltage: number;
  current: number;
}

const HISTORY_WINDOW_SIZE = 20; // Keep last 20 snapshots
const MINIMUM_SNAPSHOTS_FOR_ESTIMATE = 2; // Start estimating after just 2 snapshots
const MINIMUM_SNAPSHOTS_FOR_GOOD = 4; // Need 4 for medium confidence
const MINIMUM_SNAPSHOTS_FOR_HIGH = 8; // Need 8 for high confidence
const SNAPSHOT_INTERVAL = 10000; // Take snapshot every 10 seconds - much lighter on resources
const INITIAL_SNAPSHOT_INTERVAL = 3000; // First few snapshots at 3 seconds for quicker initial estimate
const OUTLIER_THRESHOLD = 3; // Standard deviations for outlier detection
const TECHNICAL_UPDATE_INTERVAL = 5000; // Update voltage/current every 5 seconds (0.2Hz)
const DEFAULT_DISCHARGE_RATE = 5; // Default 5% per hour for initial estimate
const STORAGE_KEY = 'battery-estimator-history';
const STORAGE_EXPIRY = 10 * 60 * 1000; // Clear storage after 10 minutes of inactivity

export default function useBatteryTimeEstimator(batteryState: BatteryState) {
  const [estimate, setEstimate] = useState<BatteryTimeEstimate>({
    remainingMinutes: 0,
    remainingHours: 0,
    formattedTime: 'Initializing...',
    dischargeRate: DEFAULT_DISCHARGE_RATE,
    confidence: 'calculating',
    estimatedShutdownTime: null,
    isCharging: false,
    timeToFullCharge: null,
    currentVoltage: 0,
    currentAmperage: 0,
    lastTechnicalUpdate: 0,
  });

  const historyRef = useRef<BatterySnapshot[]>([]);
  const lastSnapshotTimeRef = useRef<number>(0);
  const lastTechnicalUpdateRef = useRef<number>(0);
  const initialEstimateRef = useRef<boolean>(true);
  const storageLoadedRef = useRef<boolean>(false);

  // Load history from localStorage on mount
  useEffect(() => {
    if (typeof window === 'undefined') return;

    try {
      const stored = localStorage.getItem(STORAGE_KEY);
      if (stored) {
        const parsed = JSON.parse(stored);
        const now = Date.now();

        // Check if data is not too old
        if (parsed.lastUpdate && (now - parsed.lastUpdate) < STORAGE_EXPIRY) {
          // Filter out snapshots that are too old
          const validSnapshots = (parsed.history || []).filter((snapshot: BatterySnapshot) =>
            (now - snapshot.timestamp) < STORAGE_EXPIRY
          );

          if (validSnapshots.length > 0) {
            historyRef.current = validSnapshots;
            initialEstimateRef.current = false; // Skip initial estimate if we have data
            storageLoadedRef.current = true;
          }
        } else {
          // Clear old data
          localStorage.removeItem(STORAGE_KEY);
        }
      }
    } catch (error) {
      console.error('Failed to load battery history from localStorage:', error);
      localStorage.removeItem(STORAGE_KEY);
    }
  }, []);

  // Save history to localStorage whenever it changes
  const saveToLocalStorage = (history: BatterySnapshot[]) => {
    if (typeof window === 'undefined') return;

    try {
      const dataToStore = {
        history: history.slice(-HISTORY_WINDOW_SIZE), // Only store recent history
        lastUpdate: Date.now(),
      };
      localStorage.setItem(STORAGE_KEY, JSON.stringify(dataToStore));
    } catch (error) {
      console.error('Failed to save battery history to localStorage:', error);
      // If storage is full or fails, try to clear it
      try {
        localStorage.removeItem(STORAGE_KEY);
      } catch (clearError) {
        console.error('Failed to clear localStorage:', clearError);
      }
    }
  };

  // Helper function to detect outliers using Z-score
  const removeOutliers = (values: number[]): number[] => {
    if (values.length < 3) return values;

    const mean = values.reduce((a, b) => a + b, 0) / values.length;
    const variance = values.reduce((a, b) => a + Math.pow(b - mean, 2), 0) / values.length;
    const stdDev = Math.sqrt(variance);

    if (stdDev === 0) return values;

    return values.filter(val => Math.abs((val - mean) / stdDev) < OUTLIER_THRESHOLD);
  };

  // Calculate weighted moving average with recent data having more weight
  const calculateWeightedAverage = (rates: number[]): number => {
    if (rates.length === 0) return DEFAULT_DISCHARGE_RATE;

    let weightedSum = 0;
    let totalWeight = 0;

    rates.forEach((rate, index) => {
      const weight = Math.pow(1.5, index); // Exponential weighting
      weightedSum += rate * weight;
      totalWeight += weight;
    });

    return weightedSum / totalWeight;
  };

  // Quick discharge rate calculation for initial estimates
  const calculateQuickDischargeRate = (history: BatterySnapshot[]): number => {
    if (history.length < 2) return DEFAULT_DISCHARGE_RATE;

    // For initial estimates, just use the last two snapshots
    const recent = history.slice(-2);
    const timeDiffHours = (recent[1].timestamp - recent[0].timestamp) / (1000 * 60 * 60);

    if (timeDiffHours <= 0) return DEFAULT_DISCHARGE_RATE;

    const percentageDiff = recent[0].percentage - recent[1].percentage;
    const rate = percentageDiff / timeDiffHours;

    // Return the rate if it's reasonable, otherwise use default
    if (rate > 0 && rate < 100) {
      return rate;
    }

    return DEFAULT_DISCHARGE_RATE;
  };

  // Advanced discharge rate calculation (for better accuracy over time)
  const calculateDischargeRate = (history: BatterySnapshot[], quick: boolean = false): number => {
    if (history.length < 2) return DEFAULT_DISCHARGE_RATE;

    // For quick estimates with few samples, use simple calculation
    if (quick || history.length < MINIMUM_SNAPSHOTS_FOR_GOOD) {
      return calculateQuickDischargeRate(history);
    }

    const rates: number[] = [];

    // Calculate rates between consecutive snapshots
    for (let i = 1; i < history.length; i++) {
      const timeDiffHours = (history[i].timestamp - history[i - 1].timestamp) / (1000 * 60 * 60);
      if (timeDiffHours > 0) {
        const percentageDiff = history[i - 1].percentage - history[i].percentage;
        const rate = percentageDiff / timeDiffHours;

        // Only consider positive discharge rates (battery depleting)
        if (rate > 0 && rate < 100) { // Cap at 100% per hour as sanity check
          rates.push(rate);
        }
      }
    }

    if (rates.length === 0) return DEFAULT_DISCHARGE_RATE;

    // Only remove outliers if we have enough data
    const cleanRates = history.length >= MINIMUM_SNAPSHOTS_FOR_HIGH
      ? removeOutliers(rates)
      : rates;

    if (cleanRates.length === 0) return DEFAULT_DISCHARGE_RATE;

    // Use weighted average favoring recent measurements
    return calculateWeightedAverage(cleanRates);
  };

  // Format time display
  const formatRemainingTime = (minutes: number): string => {
    if (minutes <= 0) return 'Depleted';
    if (minutes === Infinity || minutes > 10000) return 'Stable';

    const hours = Math.floor(minutes / 60);
    const mins = Math.round(minutes % 60);

    if (hours === 0) {
      return `${mins}m`;
    } else if (mins === 0) {
      return `${hours}h`;
    } else {
      return `${hours}h ${mins}m`;
    }
  };

  // Determine confidence level based on data quality
  const getConfidenceLevel = (history: BatterySnapshot[]): BatteryTimeEstimate['confidence'] => {
    const snapshotCount = history.length;

    if (snapshotCount < MINIMUM_SNAPSHOTS_FOR_ESTIMATE) return 'calculating';
    if (snapshotCount < MINIMUM_SNAPSHOTS_FOR_GOOD) return 'low';
    if (snapshotCount < MINIMUM_SNAPSHOTS_FOR_HIGH) return 'medium';

    // For high confidence, also check consistency of readings
    const rates: number[] = [];
    for (let i = 1; i < Math.min(history.length, 10); i++) {
      const timeDiffHours = (history[i].timestamp - history[i - 1].timestamp) / (1000 * 60 * 60);
      if (timeDiffHours > 0) {
        const percentageDiff = history[i - 1].percentage - history[i].percentage;
        const rate = percentageDiff / timeDiffHours;
        if (rate > 0 && rate < 100) rates.push(rate);
      }
    }

    if (rates.length < 3) return 'medium';

    const mean = rates.reduce((a, b) => a + b, 0) / rates.length;
    const variance = rates.reduce((a, b) => a + Math.pow(b - mean, 2), 0) / rates.length;
    const coefficientOfVariation = mean > 0 ? Math.sqrt(variance) / mean : 1;

    // Lower CV means more consistent readings
    if (coefficientOfVariation < 0.15) return 'high';
    if (coefficientOfVariation < 0.3) return 'medium';
    return 'low';
  };

  // Clean up localStorage on unmount if no recent activity
  useEffect(() => {
    return () => {
      // Save final state before unmount
      if (historyRef.current.length > 0) {
        saveToLocalStorage(historyRef.current);
      }
    };
  }, []);

  useEffect(() => {
    const currentTime = Date.now();
    const currentPercentage = batteryState.percentage * 100;

    // Check if battery percentage is valid
    if (currentPercentage < 0 || currentPercentage > 100) return;

    // Update technical readings at 0.2Hz
    let currentVoltage = estimate.currentVoltage;
    let currentAmperage = estimate.currentAmperage;
    let lastTechnicalUpdate = estimate.lastTechnicalUpdate;

    if (currentTime - lastTechnicalUpdateRef.current >= TECHNICAL_UPDATE_INTERVAL) {
      currentVoltage = batteryState.voltage;
      currentAmperage = batteryState.current;
      lastTechnicalUpdate = currentTime;
      lastTechnicalUpdateRef.current = currentTime;
    }

    // Use adaptive interval - faster at the beginning, slower once we have data
    const currentInterval = historyRef.current.length < MINIMUM_SNAPSHOTS_FOR_GOOD
      ? INITIAL_SNAPSHOT_INTERVAL
      : SNAPSHOT_INTERVAL;

    // Check if it's time for a new snapshot
    if (currentTime - lastSnapshotTimeRef.current >= currentInterval) {
      const newSnapshot: BatterySnapshot = {
        percentage: currentPercentage,
        timestamp: currentTime,
        voltage: batteryState.voltage,
        current: batteryState.current,
      };

      // Update history with sliding window
      const newHistory = [...historyRef.current, newSnapshot].slice(-HISTORY_WINDOW_SIZE);
      historyRef.current = newHistory;
      lastSnapshotTimeRef.current = currentTime;

      // Save to localStorage less frequently
      if (newHistory.length % 10 === 0 || newHistory.length === MINIMUM_SNAPSHOTS_FOR_GOOD) { // Save every 10 snapshots or when reaching good confidence
        saveToLocalStorage(newHistory);
      }

      // Check if charging (power_supply_status: 1 = charging in ROS2)
      const isCharging = batteryState.power_supply_status === 1 || batteryState.current > 0;

      if (isCharging) {
        // Calculate charging rate and time to full
        const chargeRate = Math.abs(calculateDischargeRate(newHistory, true)); // Use quick calculation
        const timeToFull = chargeRate > 0 ? ((100 - currentPercentage) / chargeRate) * 60 : null;

        setEstimate({
          remainingMinutes: 0,
          remainingHours: 0,
          formattedTime: 'Charging',
          dischargeRate: 0,
          confidence: 'high',
          estimatedShutdownTime: null,
          isCharging: true,
          timeToFullCharge: timeToFull,
          currentVoltage,
          currentAmperage,
          lastTechnicalUpdate,
        });
        initialEstimateRef.current = false;
      } else {
        // For the very first estimate, use a default rate unless we loaded from storage
        let dischargeRate: number;
        let confidence: BatteryTimeEstimate['confidence'];

        if (initialEstimateRef.current && newHistory.length < MINIMUM_SNAPSHOTS_FOR_ESTIMATE && !storageLoadedRef.current) {
          // Use default rate for immediate display
          dischargeRate = DEFAULT_DISCHARGE_RATE;
          confidence = 'calculating';
        } else {
          // Calculate discharge rate based on available data
          const quickCalc = newHistory.length < MINIMUM_SNAPSHOTS_FOR_GOOD;
          dischargeRate = calculateDischargeRate(newHistory, quickCalc);
          confidence = getConfidenceLevel(newHistory);
          initialEstimateRef.current = false;
        }

        // Calculate remaining time (stop at 5% as per requirement)
        const usablePercentage = Math.max(0, currentPercentage - 5);
        const remainingHours = dischargeRate > 0 ? usablePercentage / dischargeRate : Infinity;
        const remainingMinutes = remainingHours * 60;

        // Calculate estimated shutdown time
        const estimatedShutdownTime = dischargeRate > 0 && remainingMinutes < Infinity && remainingMinutes < 10000
          ? new Date(currentTime + remainingMinutes * 60 * 1000)
          : null;

        setEstimate({
          remainingMinutes: Math.round(remainingMinutes),
          remainingHours: remainingHours,
          formattedTime: formatRemainingTime(remainingMinutes),
          dischargeRate: Math.round(dischargeRate * 100) / 100,
          confidence,
          estimatedShutdownTime,
          isCharging: false,
          timeToFullCharge: null,
          currentVoltage,
          currentAmperage,
          lastTechnicalUpdate,
        });
      }
    } else if (currentTime - lastTechnicalUpdateRef.current >= TECHNICAL_UPDATE_INTERVAL) {
      // Just update technical values without recalculating everything
      setEstimate(prev => ({
        ...prev,
        currentVoltage,
        currentAmperage,
        lastTechnicalUpdate,
      }));
    }
  }, [batteryState, estimate.currentVoltage, estimate.currentAmperage, estimate.lastTechnicalUpdate]);

  return estimate;
}