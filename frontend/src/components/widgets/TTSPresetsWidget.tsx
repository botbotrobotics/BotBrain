'use client';

import { Widget } from './Widget';
import { useState, useEffect, useRef } from 'react';
import { useLanguage } from '@/contexts/LanguageContext';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import useRobotActionDispatcher from '@/hooks/ros/useRobotActionDispatcher';
import { Volume2, Plus, Edit2, Trash2, Check, X, Loader2 } from 'lucide-react';

interface TTSPresetsWidgetProps {
  id: string;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  title?: string;
}

interface TTSPreset {
  id: string;
  text: string;
  label: string;
  emoji?: string;
  display_order: number;
}

const EMOJI_OPTIONS = [
  // Sound & Music
  '🎤', '🔊', '📢', '🎵', '🎶', '🎸', '🎹', '🎺', '🎷', '🥁',
  // Actions & Gestures
  '👋', '✨', '🚀', '🔥', '⚡', '💫', '🌟', '💥', '🎯', '🎨',
  // Emotions & Faces
  '😊', '😎', '🤖', '👍', '👏', '💪', '🙌', '✌️', '🤝', '💯',
  // Nature & Weather
  '☀️', '🌙', '⭐', '☁️', '🌈', '❄️', '🌊', '🍃', '🌸', '🌺',
  // Food & Drink
  '☕', '🍵', '🥤', '🍽️', '🍕', '🍔', '🍟', '🍰', '🍪', '🍩',
  // Travel & Places
  '🏠', '🏢', '🏫', '🏥', '✈️', '🚗', '🚀', '🗺️', '🧭', '🚦',
  // Objects & Tools
  '💡', '🔧', '🔨', '⚙️', '🖥️', '📱', '⌚', '📷', '🎮', '🕹️',
  // Sports & Activities
  '⚽', '🏀', '🎾', '🏈', '⛳', '🏊', '🚴', '🏃', '🎿', '🏆',
  // Time & Calendar
  '⏰', '⏱️', '⏳', '📅', '🗓️', '🕐', '🕑', '🕒', '🕓', '🕔',
  // Symbols & Arrows
  '✅', '❌', '⚠️', '📍', '🔴', '🟢', '🔵', '⬆️', '⬇️', '↩️',
  // Numbers
  '1️⃣', '2️⃣', '3️⃣', '4️⃣', '5️⃣', '6️⃣', '7️⃣', '8️⃣', '9️⃣', '0️⃣',
];

export function TTSPresetsWidget({
  id,
  onRemove,
  onStartDrag,
  onEndDrag,
  initialPosition,
  initialSize = { width: 350, height: 300 },
  title = 'TTS Presets',
}: TTSPresetsWidgetProps) {
  const { t } = useLanguage();
  const { supabase, user } = useSupabase();
  const { connection } = useRobotConnection();
  const dispatchAction = useRobotActionDispatcher();
  
  const [presets, setPresets] = useState<TTSPreset[]>([]);
  const [loading, setLoading] = useState(true);
  const [editingId, setEditingId] = useState<string | null>(null);
  const [editText, setEditText] = useState('');
  const [editLabel, setEditLabel] = useState('');
  const [editEmoji, setEditEmoji] = useState('');
  const [showEmojiPicker, setShowEmojiPicker] = useState(false);
  const [isAddingNew, setIsAddingNew] = useState(false);
  const [newText, setNewText] = useState('');
  const [newLabel, setNewLabel] = useState('');
  const [newEmoji, setNewEmoji] = useState('🎤');
  const [showNewEmojiPicker, setShowNewEmojiPicker] = useState(false);
  const [sendingId, setSendingId] = useState<string | null>(null);

  useEffect(() => {
    if (user && supabase) {
      loadPresets();
    }
  }, [user, supabase]);

  const loadPresets = async () => {
    if (!supabase || !user) return;
    
    try {
      const { data, error } = await supabase
        .from('tts_presets')
        .select('*')
        .eq('user_id', user.id)
        .order('display_order', { ascending: true });

      if (error) throw error;
      setPresets(data || []);
    } catch (error) {
      console.error('Error loading TTS presets:', error);
    } finally {
      setLoading(false);
    }
  };

  const sendTTS = (text: string, presetId: string) => {
    if (!connection.online) return;
    
    setSendingId(presetId);
    
    dispatchAction({
      typeKey: 'talk',
      request: { text },
      callback: () => {
        setSendingId(null);
      },
      failedCallback: (error?: string) => {
        console.error('TTS service error:', error);
        setSendingId(null);
      },
    });
  };

  const startEdit = (preset: TTSPreset) => {
    setEditingId(preset.id);
    setEditText(preset.text);
    setEditLabel(preset.label);
    setEditEmoji(preset.emoji || '🎤');
    setShowEmojiPicker(false);
  };

  const cancelEdit = () => {
    setEditingId(null);
    setEditText('');
    setEditLabel('');
    setEditEmoji('');
    setShowEmojiPicker(false);
  };

  const saveEdit = async () => {
    if (!supabase || !editingId) return;
    
    try {
      const { error } = await supabase
        .from('tts_presets')
        .update({
          text: editText,
          label: editLabel,
          emoji: editEmoji,
          updated_at: new Date().toISOString(),
        })
        .eq('id', editingId);

      if (error) throw error;
      
      setPresets(presets.map(p => 
        p.id === editingId 
          ? { ...p, text: editText, label: editLabel, emoji: editEmoji }
          : p
      ));
      cancelEdit();
    } catch (error) {
      console.error('Error updating preset:', error);
    }
  };

  const deletePreset = async (presetId: string) => {
    if (!supabase) return;
    
    try {
      const { error } = await supabase
        .from('tts_presets')
        .delete()
        .eq('id', presetId);

      if (error) throw error;
      
      setPresets(presets.filter(p => p.id !== presetId));
    } catch (error) {
      console.error('Error deleting preset:', error);
    }
  };

  const addNewPreset = async () => {
    if (!supabase || !user || !newText.trim() || !newLabel.trim()) return;
    
    try {
      const { data, error } = await supabase
        .from('tts_presets')
        .insert({
          user_id: user.id,
          text: newText,
          label: newLabel,
          emoji: newEmoji,
          display_order: presets.length,
        })
        .select()
        .single();

      if (error) throw error;
      
      if (data) {
        setPresets([...presets, data]);
        setNewText('');
        setNewLabel('');
        setNewEmoji('🎤');
        setIsAddingNew(false);
        setShowNewEmojiPicker(false);
      }
    } catch (error) {
      console.error('Error adding preset:', error);
    }
  };

  return (
    <Widget
      id={id}
      title={
        <div className="flex items-center gap-2">
          <Volume2 className="w-4 h-4" />
          <span>{title}</span>
        </div>
      }
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      initialPosition={initialPosition}
      initialSize={initialSize}
      minWidth={300}
      minHeight={250}
    >
      <div className="h-full flex flex-col p-2">
        {/* Add New Button */}
        {!isAddingNew && presets.length < 10 && (
          <button
            onClick={() => setIsAddingNew(true)}
            className="mb-2 px-3 py-1.5 bg-primary/10 hover:bg-primary/20 dark:bg-botbot-accent/10 dark:hover:bg-botbot-accent/20 text-primary dark:text-botbot-accent rounded-lg transition-colors flex items-center justify-center gap-2 text-sm"
          >
            <Plus className="w-3.5 h-3.5" />
            Add Preset
          </button>
        )}

        {/* New Preset Form */}
        {isAddingNew && (
          <div className="mb-2 p-2 bg-gray-50 dark:bg-botbot-darker/50 rounded-lg">
            <div className="space-y-2">
              <div className="flex gap-2">
                <div className="relative">
                  <button
                    onClick={() => setShowNewEmojiPicker(!showNewEmojiPicker)}
                    className="w-10 h-10 flex items-center justify-center bg-white dark:bg-botbot-dark rounded-lg hover:bg-gray-100 dark:hover:bg-botbot-darker transition-colors text-xl"
                  >
                    {newEmoji}
                  </button>
                  {showNewEmojiPicker && (
                    <div className="absolute top-12 left-0 z-50 bg-white dark:bg-botbot-dark rounded-lg shadow-lg p-2 grid grid-cols-10 gap-1 max-w-[320px] max-h-[200px] overflow-y-auto">
                      {EMOJI_OPTIONS.map((emoji, index) => (
                        <button
                          key={index}
                          onClick={() => {
                            setNewEmoji(emoji);
                            setShowNewEmojiPicker(false);
                          }}
                          className="w-8 h-8 flex items-center justify-center hover:bg-gray-100 dark:hover:bg-botbot-darker rounded transition-colors"
                        >
                          {emoji}
                        </button>
                      ))}
                    </div>
                  )}
                </div>
                <input
                  type="text"
                  placeholder="Label"
                  value={newLabel}
                  onChange={(e) => setNewLabel(e.target.value)}
                  className="flex-1 px-2 py-1 text-sm bg-white dark:bg-botbot-dark rounded-lg focus:outline-none focus:ring-1 focus:ring-primary dark:focus:ring-botbot-accent"
                />
              </div>
              <input
                type="text"
                placeholder="Text to speak"
                value={newText}
                onChange={(e) => setNewText(e.target.value)}
                className="w-full px-2 py-1 text-sm bg-white dark:bg-botbot-dark rounded-lg focus:outline-none focus:ring-1 focus:ring-primary dark:focus:ring-botbot-accent"
              />
              <div className="flex gap-2">
                <button
                  onClick={addNewPreset}
                  disabled={!newText.trim() || !newLabel.trim()}
                  className="flex-1 px-2 py-1 bg-green-500 hover:bg-green-600 disabled:bg-gray-300 dark:disabled:bg-gray-700 text-white rounded-lg transition-colors flex items-center justify-center gap-1 text-sm"
                >
                  <Check className="w-3.5 h-3.5" />
                  Save
                </button>
                <button
                  onClick={() => {
                    setIsAddingNew(false);
                    setNewText('');
                    setNewLabel('');
                    setNewEmoji('🎤');
                    setShowNewEmojiPicker(false);
                  }}
                  className="flex-1 px-2 py-1 bg-gray-500 hover:bg-gray-600 text-white rounded-lg transition-colors flex items-center justify-center gap-1 text-sm"
                >
                  <X className="w-3.5 h-3.5" />
                  Cancel
                </button>
              </div>
            </div>
          </div>
        )}

        {/* Presets List */}
        <div className="flex-1 overflow-y-auto space-y-1.5 min-h-0">
          {loading ? (
            <div className="flex items-center justify-center h-full">
              <Loader2 className="w-5 h-5 animate-spin text-primary dark:text-botbot-accent" />
            </div>
          ) : presets.length === 0 ? (
            <div className="text-center text-gray-500 dark:text-gray-400 py-4 text-sm">
              No presets yet. Add one above!
            </div>
          ) : (
            presets.map((preset) => (
              <div key={preset.id}>
                {editingId === preset.id ? (
                  <div className="p-2 bg-gray-50 dark:bg-botbot-darker/50 rounded-lg">
                    <div className="space-y-2">
                      <div className="flex gap-2">
                        <div className="relative">
                          <button
                            onClick={() => setShowEmojiPicker(!showEmojiPicker)}
                            className="w-10 h-10 flex items-center justify-center bg-white dark:bg-botbot-dark rounded-lg hover:bg-gray-100 dark:hover:bg-botbot-darker transition-colors text-xl"
                          >
                            {editEmoji}
                          </button>
                          {showEmojiPicker && (
                            <div className="absolute top-12 left-0 z-50 bg-white dark:bg-botbot-dark rounded-lg shadow-lg p-2 grid grid-cols-10 gap-1 max-w-[320px] max-h-[200px] overflow-y-auto">
                              {EMOJI_OPTIONS.map((emoji, index) => (
                                <button
                                  key={index}
                                  onClick={() => {
                                    setEditEmoji(emoji);
                                    setShowEmojiPicker(false);
                                  }}
                                  className="w-8 h-8 flex items-center justify-center hover:bg-gray-100 dark:hover:bg-botbot-darker rounded transition-colors"
                                >
                                  {emoji}
                                </button>
                              ))}
                            </div>
                          )}
                        </div>
                        <input
                          type="text"
                          value={editLabel}
                          onChange={(e) => setEditLabel(e.target.value)}
                          className="flex-1 px-2 py-1 text-sm bg-white dark:bg-botbot-dark rounded-lg focus:outline-none focus:ring-1 focus:ring-primary dark:focus:ring-botbot-accent"
                        />
                      </div>
                      <input
                        type="text"
                        value={editText}
                        onChange={(e) => setEditText(e.target.value)}
                        className="w-full px-2 py-1 text-sm bg-white dark:bg-botbot-dark rounded-lg focus:outline-none focus:ring-1 focus:ring-primary dark:focus:ring-botbot-accent"
                      />
                      <div className="flex gap-2">
                        <button
                          onClick={saveEdit}
                          className="flex-1 px-2 py-1 bg-green-500 hover:bg-green-600 text-white rounded-lg transition-colors flex items-center justify-center gap-1 text-sm"
                        >
                          <Check className="w-3.5 h-3.5" />
                        </button>
                        <button
                          onClick={cancelEdit}
                          className="flex-1 px-2 py-1 bg-gray-500 hover:bg-gray-600 text-white rounded-lg transition-colors flex items-center justify-center gap-1 text-sm"
                        >
                          <X className="w-3.5 h-3.5" />
                        </button>
                      </div>
                    </div>
                  </div>
                ) : (
                  <div className="group relative bg-gradient-to-r from-primary/5 to-primary/10 dark:from-botbot-accent/5 dark:to-botbot-accent/10 hover:from-primary/10 hover:to-primary/20 dark:hover:from-botbot-accent/10 dark:hover:to-botbot-accent/20 rounded-lg transition-all duration-200">
                    <button
                      onClick={() => sendTTS(preset.text, preset.id)}
                      disabled={!connection.online || sendingId === preset.id}
                      className="w-full p-2 flex items-center gap-2 disabled:opacity-50 disabled:cursor-not-allowed"
                    >
                      <span className="text-xl">{preset.emoji || '🎤'}</span>
                      <div className="flex-1 text-left">
                        <div className="font-medium text-sm text-gray-800 dark:text-white">
                          {preset.label}
                        </div>
                        <div className="text-xs text-gray-600 dark:text-gray-400 truncate">
                          {preset.text}
                        </div>
                      </div>
                      {sendingId === preset.id && (
                        <Loader2 className="w-3.5 h-3.5 animate-spin text-primary dark:text-botbot-accent" />
                      )}
                    </button>
                    <div className="absolute right-1 top-1/2 -translate-y-1/2 opacity-0 group-hover:opacity-100 transition-opacity flex gap-1">
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          startEdit(preset);
                        }}
                        className="p-1 bg-white dark:bg-botbot-dark rounded hover:bg-gray-100 dark:hover:bg-botbot-darker transition-colors"
                      >
                        <Edit2 className="w-3 h-3" />
                      </button>
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          deletePreset(preset.id);
                        }}
                        className="p-1 bg-white dark:bg-botbot-dark rounded hover:bg-red-100 dark:hover:bg-red-900/30 transition-colors"
                      >
                        <Trash2 className="w-3 h-3 text-red-500" />
                      </button>
                    </div>
                  </div>
                )}
              </div>
            ))
          )}
        </div>
      </div>
    </Widget>
  );
}