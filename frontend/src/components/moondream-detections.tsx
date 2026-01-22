'use client';

import { useState, useEffect } from 'react';
import { Send, Eye, HelpCircle, Brain, Search, MapPin, Loader2, X } from 'lucide-react';
import useMoondreamServices, { MoondreamBoundingBox, MoondreamPoint } from '@/hooks/ros/useMoondreamServices';

interface MoondreamDetectionsProps {
  expandedView?: boolean;
  onBoundingBoxesChange?: (boxes: MoondreamBoundingBox[]) => void;
  onPointsChange?: (points: MoondreamPoint[]) => void;
}

type MoonDreamMode = 'caption' | 'query-no-reasoning' | 'query-with-reasoning' | 'detect' | 'point';

interface ModeConfigBase {
  title: string;
  description: string;
  icon: typeof Eye;
  color: string;
}

interface ModeConfigWithInput extends ModeConfigBase {
  hasInput: true;
  inputPlaceholder: string;
  inputLabel: string;
}

interface ModeConfigWithoutInput extends ModeConfigBase {
  hasInput: false;
}

type ModeConfigType = ModeConfigWithInput | ModeConfigWithoutInput;

const modeConfig: Record<MoonDreamMode, ModeConfigType> = {
  'caption': {
    title: 'Caption',
    description: 'Generate a description of the image',
    icon: Eye,
    color: 'from-blue-500 to-cyan-500',
    hasInput: false,
  },
  'query-no-reasoning': {
    title: 'Query - No Reasoning',
    description: 'Ask a question about the image',
    icon: HelpCircle,
    color: 'from-purple-500 to-pink-500',
    hasInput: true,
    inputPlaceholder: 'Ask a question about the image...',
    inputLabel: 'Question',
  },
  'query-with-reasoning': {
    title: 'Query - With Reasoning',
    description: 'Ask a question with detailed reasoning',
    icon: Brain,
    color: 'from-green-500 to-emerald-500',
    hasInput: true,
    inputPlaceholder: 'Ask a question for detailed analysis...',
    inputLabel: 'Question',
  },
  'detect': {
    title: 'Detect',
    description: 'Detect specific objects in the image',
    icon: Search,
    color: 'from-orange-500 to-red-500',
    hasInput: true,
    inputPlaceholder: 'Describe what to detect...',
    inputLabel: 'Description',
  },
  'point': {
    title: 'Point',
    description: 'Point to specific locations in the image',
    icon: MapPin,
    color: 'from-indigo-500 to-purple-500',
    hasInput: true,
    inputPlaceholder: 'Describe what to point to...',
    inputLabel: 'Description',
  },
};

export default function MoondreamDetections({
  expandedView = false,
  onBoundingBoxesChange,
  onPointsChange
}: MoondreamDetectionsProps) {
  const [selectedMode, setSelectedMode] = useState<MoonDreamMode>('caption');
  const [inputValue, setInputValue] = useState('');
  const [response, setResponse] = useState('Ready to interact with MoonDream. Select a mode and submit to begin.');
  const [error, setError] = useState<string | null>(null);

  const {
    callCaption,
    callQuery,
    callDetect,
    callPoint,
    clearOverlays,
    loading,
    boundingBoxes,
    points
  } = useMoondreamServices();

  const currentMode = modeConfig[selectedMode];
  const ModeIcon = currentMode.icon;

  // Update parent component when bounding boxes or points change
  useEffect(() => {
    if (onBoundingBoxesChange) {
      onBoundingBoxesChange(boundingBoxes);
    }
  }, [boundingBoxes, onBoundingBoxesChange]);

  useEffect(() => {
    if (onPointsChange) {
      onPointsChange(points);
    }
  }, [points, onPointsChange]);


  const handleSubmit = async () => {
    setError(null);

    try {
      let result;

      switch (selectedMode) {
        case 'caption':
          result = await callCaption();
          if (result.success && result.caption) {
            setResponse(result.caption);
          } else {
            setError(result.error || 'Failed to generate caption');
          }
          break;

        case 'query-no-reasoning':
          if (!inputValue.trim()) {
            setError('Please enter a question');
            return;
          }
          result = await callQuery(inputValue, false);
          if (result.success && result.answer) {
            setResponse(result.answer);
          } else {
            setError(result.error || 'Failed to answer question');
          }
          break;

        case 'query-with-reasoning':
          if (!inputValue.trim()) {
            setError('Please enter a question');
            return;
          }
          result = await callQuery(inputValue, true);
          if (result.success) {
            if (result.reasoning) {
              setResponse(`${result.reasoning.text}\n\nAnswer: ${result.answer}`);
            } else if (result.answer) {
              setResponse(result.answer);
            }
          } else {
            setError(result.error || 'Failed to answer question with reasoning');
          }
          break;

        case 'detect':
          if (!inputValue.trim()) {
            setError('Please enter a description');
            return;
          }
          result = await callDetect(inputValue);
          if (result.success && result.objects) {
            const count = result.objects.length;
            setResponse(`Found ${count} object${count !== 1 ? 's' : ''} matching "${inputValue}". ${count > 0 ? 'Purple bounding boxes shown on the live stream.' : ''}`);
          } else {
            setError(result.error || 'Failed to detect objects');
          }
          break;

        case 'point':
          if (!inputValue.trim()) {
            setError('Please enter a description');
            return;
          }
          result = await callPoint(inputValue);
          if (result.success && result.points) {
            const count = result.points.length;
            setResponse(`Identified ${count} point${count !== 1 ? 's' : ''} for "${inputValue}". ${count > 0 ? 'Red markers shown on the live stream.' : ''}`);
          } else {
            setError(result.error || 'Failed to identify points');
          }
          break;
      }
    } catch (err) {
      setError('An unexpected error occurred. Please try again.');
      console.error('MoonDream error:', err);
    }
  };

  return (
    <div className="h-full flex flex-col p-6 justify-center overflow-y-auto custom-scrollbar">
      {/* Mode Selection */}
      <div className="flex-shrink-0 mb-6">
        <div className="grid grid-cols-2 lg:grid-cols-3 xl:grid-cols-5 gap-3">
          {(Object.keys(modeConfig) as MoonDreamMode[]).map((mode) => {
            const config = modeConfig[mode];
            const Icon = config.icon;
            const isSelected = selectedMode === mode;

            return (
              <button
                key={mode}
                onClick={() => {
                  setSelectedMode(mode);
                  setInputValue(''); // Clear input when switching modes
                  setResponse('Ready to interact with MoonDream. Select a mode and submit to begin.');
                  setError(null);
                  clearOverlays(); // Clear any existing overlays
                }}
                className={`
                  relative overflow-hidden group transition-all duration-300
                  p-4 rounded-lg border-2
                  ${isSelected
                    ? 'border-purple-500 dark:border-purple-400 bg-gradient-to-br from-purple-500/10 to-blue-500/10'
                    : 'border-gray-200 dark:border-gray-700 hover:border-gray-300 dark:hover:border-gray-600'
                  }
                `}
              >
                {/* Background gradient on hover/selected */}
                <div className={`
                  absolute inset-0 opacity-0 group-hover:opacity-10 transition-opacity duration-300
                  bg-gradient-to-br ${config.color}
                  ${isSelected ? '!opacity-20' : ''}
                `} />

                <div className="relative flex flex-col items-center gap-2">
                  <Icon className={`
                    w-6 h-6 transition-colors duration-300
                    ${isSelected
                      ? 'text-purple-600 dark:text-purple-400'
                      : 'text-gray-600 dark:text-gray-400 group-hover:text-gray-800 dark:group-hover:text-gray-200'
                    }
                  `} />
                  <span className={`
                    text-sm font-medium transition-colors duration-300
                    ${isSelected
                      ? 'text-gray-900 dark:text-white'
                      : 'text-gray-700 dark:text-gray-300'
                    }
                  `}>
                    {config.title}
                  </span>
                </div>
              </button>
            );
          })}
        </div>
      </div>

      {/* Mode Details Card */}
      <div className="flex-shrink-0 mb-6">
        <div className={`
          relative overflow-hidden
          p-6 rounded-lg border border-gray-200 dark:border-gray-700
          bg-gradient-to-br from-gray-50 to-gray-100 dark:from-botbot-darkest/50 dark:to-botbot-darker/50
        `}>
          {/* Accent gradient */}
          <div className={`absolute top-0 left-0 w-full h-1.5 bg-gradient-to-r ${currentMode.color}`} />

          <div className="flex items-center gap-4">
            <div className={`
              p-3 rounded-lg bg-gradient-to-br ${currentMode.color}
              text-white shadow-lg
            `}>
              <ModeIcon className="w-6 h-6" />
            </div>

            <div className="flex-1">
              <h3 className="font-semibold text-lg text-gray-900 dark:text-white mb-1">
                {currentMode.title}
              </h3>
              <p className="text-gray-600 dark:text-gray-400">
                {currentMode.description}
              </p>
            </div>
          </div>
        </div>
      </div>

      {/* Input Field */}
      {currentMode.hasInput && (
        <div className="flex-shrink-0">
          <label className="block text-base font-medium text-gray-700 dark:text-gray-300 mb-3">
            {(currentMode as ModeConfigWithInput).inputLabel}
          </label>
          <div className="flex gap-3">
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={(e) => e.key === 'Enter' && inputValue.trim() && handleSubmit()}
              placeholder={(currentMode as ModeConfigWithInput).inputPlaceholder}
              className={`
                flex-1 px-5 py-3 rounded-lg text-base
                border border-gray-300 dark:border-gray-600
                bg-white dark:bg-botbot-darkest
                text-gray-900 dark:text-white
                placeholder-gray-400 dark:placeholder-gray-500
                focus:outline-none focus:ring-2 focus:ring-purple-500 dark:focus:ring-purple-400
                transition-all duration-200
              `}
            />

            <button
              onClick={handleSubmit}
              disabled={!inputValue.trim() || loading}
              className={`
                px-6 py-3 rounded-lg font-medium
                flex items-center gap-2
                transition-all duration-200
                ${inputValue.trim() && !loading
                  ? 'bg-gradient-to-r from-purple-500 to-blue-500 text-white hover:from-purple-600 hover:to-blue-600 shadow-lg hover:shadow-xl transform hover:-translate-y-0.5'
                  : 'bg-gray-200 dark:bg-gray-700 text-gray-400 dark:text-gray-500 cursor-not-allowed'
                }
              `}
            >
              {loading ? (
                <Loader2 className="w-4 h-4 animate-spin" />
              ) : (
                <Send className="w-4 h-4" />
              )}
              <span>{loading ? 'Processing...' : 'Send'}</span>
            </button>

            {/* Clear button for Detect and Point modes */}
            {(selectedMode === 'detect' || selectedMode === 'point') && (boundingBoxes.length > 0 || points.length > 0) && (
              <button
                onClick={() => {
                  clearOverlays();
                  setResponse('Overlays cleared from the live stream.');
                }}
                className={`
                  px-4 py-3 rounded-lg font-medium
                  flex items-center gap-2
                  transition-all duration-200
                  bg-gradient-to-r from-red-500 to-orange-500 text-white
                  hover:from-red-600 hover:to-orange-600
                  shadow-lg hover:shadow-xl transform hover:-translate-y-0.5
                `}
                title="Clear all bounding boxes and points from the live stream"
              >
                <X className="w-4 h-4" />
                <span>Clear</span>
              </button>
            )}
          </div>
        </div>
      )}

      {/* Submit Button for Caption mode */}
      {!currentMode.hasInput && (
        <div className="flex-shrink-0">
          <button
            onClick={handleSubmit}
            disabled={loading}
            className={`
              w-full px-6 py-4 rounded-lg font-medium text-base
              ${!loading
                ? 'bg-gradient-to-r from-purple-500 to-blue-500 text-white hover:from-purple-600 hover:to-blue-600 shadow-lg hover:shadow-xl transform hover:-translate-y-0.5'
                : 'bg-gray-200 dark:bg-gray-700 text-gray-400 dark:text-gray-500 cursor-not-allowed'
              }
              transition-all duration-200
              flex items-center justify-center gap-3
            `}
          >
            {loading ? (
              <Loader2 className="w-6 h-6 animate-spin" />
            ) : (
              <ModeIcon className="w-6 h-6" />
            )}
            <span>{loading ? 'Generating...' : 'Generate Caption'}</span>
          </button>
        </div>
      )}

      {/* Response Area */}
      <div className="flex-shrink-0 mt-6">
        <div className="relative">
          {/* Response Label */}
          <div className="flex items-center justify-between gap-2 mb-3">
            <div className="flex items-center gap-2">
              <div className={`w-2 h-2 rounded-full bg-gradient-to-r ${currentMode.color} animate-pulse`} />
              <span className="text-sm font-medium text-gray-700 dark:text-gray-300">
                MoonDream Response
              </span>
            </div>

            {/* Clear overlays indicator */}
            {(boundingBoxes.length > 0 || points.length > 0) && (
              <div className="flex items-center gap-2 text-xs">
                <span className="text-gray-500 dark:text-gray-400">
                  {boundingBoxes.length > 0 && `${boundingBoxes.length} box${boundingBoxes.length !== 1 ? 'es' : ''}`}
                  {boundingBoxes.length > 0 && points.length > 0 && ', '}
                  {points.length > 0 && `${points.length} point${points.length !== 1 ? 's' : ''}`}
                </span>
                <button
                  onClick={() => {
                    clearOverlays();
                    setResponse('All overlays have been cleared from the live stream.');
                  }}
                  className="px-2 py-1 rounded bg-red-500/10 hover:bg-red-500/20 text-red-600 dark:text-red-400 font-medium transition-colors duration-200"
                  title="Clear all overlays"
                >
                  Clear All
                </button>
              </div>
            )}
          </div>

          {/* Response Content */}
          <div className={`
            relative overflow-hidden
            p-6 rounded-lg
            bg-gradient-to-br from-white to-gray-50 dark:from-botbot-darkest/70 dark:to-botbot-darker/50
            border border-gray-200 dark:border-gray-700
            shadow-sm
            transition-all duration-300
            hover:shadow-md
          `}>
            {/* Gradient accent line */}
            <div className={`absolute top-0 left-0 w-full h-0.5 bg-gradient-to-r ${currentMode.color}`} />

            {/* Response Text or Error */}
            <div className="relative">
              {error ? (
                <div className="flex items-start gap-3">
                  <div className="text-red-500 mt-1">⚠️</div>
                  <p className="text-red-600 dark:text-red-400 leading-relaxed text-base">
                    {error}
                  </p>
                </div>
              ) : (
                <p className="text-gray-800 dark:text-gray-200 leading-relaxed text-base whitespace-pre-wrap">
                  {response}
                </p>
              )}
            </div>

            {/* Decorative gradient corner */}
            <div className={`
              absolute -bottom-1 -right-1 w-20 h-20 opacity-10
              bg-gradient-to-br ${currentMode.color}
              blur-2xl
            `} />
          </div>

          {/* Response metadata */}
          <div className="mt-2 flex items-center justify-between text-xs text-gray-500 dark:text-gray-400">
            <span className="italic">
              Mode: {currentMode.title}
            </span>
            {inputValue && currentMode.hasInput && (
              <span className="truncate max-w-[200px]">
                Input: "{inputValue}"
              </span>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}