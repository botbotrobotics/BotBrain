'use client';

import { useState, useEffect } from 'react';
import { Plus, X, Edit2, Check, Loader2, Sparkles, Volume2, Trash2, Mic, Music, Bell, Heart, Star, Zap, MessageSquare, Smile, Megaphone, Radio, Headphones, Speaker } from 'lucide-react';
import { createClient } from '@/utils/supabase/client';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';

interface TTSPreset {
  id: string;
  text: string;
  label: string;
  emoji?: string;
  display_order: number;
}

interface SoundboardTTSPresetsProps {
  onSendMessage: (text: string) => void;
}

const EMOJI_OPTIONS = [
  // Sound & Music
  '🎤', '🔊', '📢', '🎵', '🎶', '🎸', '🎹', '🎺', '🎷', '🥁',
  // Actions & Gestures
  '👋', '✨', '🚀', '🔥', '⚡', '💫', '🌟', '💥', '🎯', '🎨',
  // Expressions
  '😊', '😎', '🤖', '👀', '💬', '💭', '🗣️', '📣', '🔔', '❤️',
  // Entertainment
  '🎭', '🎪', '🎉', '🎊', '🏆', '🎁', '🌈', '☀️', '🌙', '⭐',
  // Numbers
  '1️⃣', '2️⃣', '3️⃣', '4️⃣', '5️⃣', '6️⃣', '7️⃣', '8️⃣', '9️⃣', '🔟',
  // Alerts & Status
  '⚠️', '🚨', '❗', '❓', '💡', '🔴', '🟢', '🟡', '🔵', '🟣',
  // Arrows & Directions
  '⬆️', '⬇️', '⬅️', '➡️', '↩️', '↪️', '🔄', '🔃', '⏮️', '⏭️',
  // Time & Control
  '⏰', '⏱️', '⏸️', '▶️', '⏹️', '⏺️', '🕐', '📅', '📆', '⌚',
  // Common Objects
  '📱', '💻', '🖥️', '🎮', '🕹️', '📷', '📹', '🔦', '💾', '📡',
  // Symbols
  '✅', '❌', '➕', '➖', '✖️', '➗', '💯', '🆗', '🆕', '🆒'
];

export default function SoundboardTTSPresets({ onSendMessage }: SoundboardTTSPresetsProps) {
  const [presets, setPresets] = useState<TTSPreset[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [isAdding, setIsAdding] = useState(false);
  const [newPreset, setNewPreset] = useState({ text: '', label: '', emoji: '🎤' });
  const [editingId, setEditingId] = useState<string | null>(null);
  const [editingPreset, setEditingPreset] = useState<TTSPreset | null>(null);
  const [showEmojiPicker, setShowEmojiPicker] = useState(false);
  const [editEmojiPicker, setEditEmojiPicker] = useState(false);
  const { user } = useSupabase();
  const { connection } = useRobotConnection();
  const supabase = createClient();

  useEffect(() => {
    if (user) {
      loadPresets();
    }
  }, [user]);

  const loadPresets = async () => {
    if (!user) return;
    
    setIsLoading(true);
    const { data, error } = await supabase
      .from('tts_presets')
      .select('*')
      .eq('user_id', user.id)
      .order('display_order', { ascending: true })
      .limit(10);

    if (!error && data) {
      setPresets(data);
    }
    setIsLoading(false);
  };

  const addPreset = async () => {
    if (!user || !newPreset.text || !newPreset.label) return;

    const { data, error } = await supabase
      .from('tts_presets')
      .insert({
        user_id: user.id,
        text: newPreset.text,
        label: newPreset.label,
        emoji: newPreset.emoji || '🎤',
        display_order: presets.length
      })
      .select()
      .single();

    if (!error && data) {
      setPresets([...presets, data]);
      setNewPreset({ text: '', label: '', emoji: '🎤' });
      setIsAdding(false);
    }
  };

  const updatePreset = async () => {
    if (!editingPreset || !editingId || !user) return;

    const { error } = await supabase
      .from('tts_presets')
      .update({
        text: editingPreset.text,
        label: editingPreset.label,
        emoji: editingPreset.emoji
      })
      .eq('id', editingId)
      .eq('user_id', user.id);

    if (!error) {
      setPresets(presets.map(p =>
        p.id === editingId ? editingPreset : p
      ));
      setEditingId(null);
      setEditingPreset(null);
    }
  };

  const deletePreset = async (id: string) => {
    if (!user) return;

    const { error } = await supabase
      .from('tts_presets')
      .delete()
      .eq('id', id)
      .eq('user_id', user.id);

    if (!error) {
      setPresets(presets.filter(p => p.id !== id));
    }
  };

  const handlePresetClick = (preset: TTSPreset) => {
    if (connection.online) {
      onSendMessage(preset.text);
    }
  };

  if (isLoading) {
    return (
      <div className="flex items-center justify-center py-8">
        <Loader2 className="w-6 h-6 animate-spin text-primary dark:text-botbot-accent" />
      </div>
    );
  }

  return (
    <div className="space-y-4">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <div className="p-1.5 bg-gradient-to-br from-purple-500 to-pink-500 rounded-lg">
            <Sparkles className="w-3.5 h-3.5 text-white" />
          </div>
          <h3 className="text-sm font-bold bg-gradient-to-r from-purple-600 to-pink-600 dark:from-purple-400 dark:to-pink-400 bg-clip-text text-transparent">
            Quick TTS Presets
          </h3>
        </div>
        {presets.length < 10 && !isAdding && (
          <button
            onClick={() => setIsAdding(true)}
            className="group p-2 bg-gradient-to-r from-purple-50 to-pink-50 dark:from-botbot-darker/80 dark:to-botbot-darker/60 hover:from-purple-100 hover:to-pink-100 dark:hover:from-botbot-dark dark:hover:to-botbot-dark/80 rounded-xl transition-all duration-200 shadow-sm hover:shadow-md"
            title="Add preset"
          >
            <Plus className="w-4 h-4 text-purple-600 dark:text-purple-400 group-hover:rotate-90 transition-transform duration-200" />
          </button>
        )}
      </div>

      {/* Presets Grid */}
      <div className="grid grid-cols-2 gap-3">
        {presets.map((preset) => (
          <div key={preset.id} className="group relative">
            {editingId === preset.id ? (
              <div className="relative bg-gradient-to-br from-gray-50 to-gray-100 dark:from-botbot-darker dark:to-botbot-darker/80 rounded-2xl p-3 border border-gray-200 dark:border-gray-700 shadow-sm">
                <div className="space-y-2">
                  <div className="flex items-center gap-2">
                    <button
                      onClick={() => setEditEmojiPicker(!editEmojiPicker)}
                      className="relative p-2 bg-white dark:bg-botbot-dark rounded-xl hover:scale-110 transition-transform"
                    >
                      <span className="text-xl">{editingPreset?.emoji || '🎤'}</span>
                    </button>
                    <input
                      type="text"
                      value={editingPreset?.label || ''}
                      onChange={(e) => setEditingPreset({ ...editingPreset!, label: e.target.value })}
                      className="flex-1 px-3 py-1.5 bg-white dark:bg-botbot-dark rounded-xl text-sm font-medium outline-none focus:ring-2 focus:ring-purple-500 dark:focus:ring-purple-400"
                      placeholder="Label"
                    />
                  </div>
                  
                  {editEmojiPicker && (
                    <div className="absolute top-14 left-0 z-50 p-3 bg-white dark:bg-botbot-dark rounded-xl shadow-xl border border-gray-200 dark:border-gray-700 max-h-64 overflow-y-auto min-w-[280px]">
                      <div className="grid grid-cols-6 gap-1">
                        {EMOJI_OPTIONS.map((emoji) => (
                          <button
                            key={emoji}
                            onClick={() => {
                              setEditingPreset({ ...editingPreset!, emoji });
                              setEditEmojiPicker(false);
                            }}
                            className="p-2 hover:bg-gray-100 dark:hover:bg-botbot-darker rounded-lg transition-colors"
                          >
                            <span className="text-lg">{emoji}</span>
                          </button>
                        ))}
                      </div>
                    </div>
                  )}
                  
                  <textarea
                    value={editingPreset?.text || ''}
                    onChange={(e) => setEditingPreset({ ...editingPreset!, text: e.target.value })}
                    className="w-full px-3 py-2 bg-white dark:bg-botbot-dark rounded-xl text-sm outline-none focus:ring-2 focus:ring-purple-500 dark:focus:ring-purple-400 resize-none"
                    placeholder="Text to speak"
                    rows={2}
                  />
                  
                  <div className="flex gap-2">
                    <button
                      onClick={updatePreset}
                      className="flex-1 py-1.5 bg-gradient-to-r from-green-500 to-emerald-500 hover:from-green-600 hover:to-emerald-600 text-white rounded-xl text-sm font-medium transition-all duration-200 shadow-sm hover:shadow-md"
                    >
                      Save
                    </button>
                    <button
                      onClick={() => {
                        setEditingId(null);
                        setEditingPreset(null);
                        setEditEmojiPicker(false);
                      }}
                      className="px-3 py-1.5 bg-gray-200 hover:bg-gray-300 dark:bg-botbot-dark dark:hover:bg-botbot-darker rounded-xl text-sm transition-colors"
                    >
                      Cancel
                    </button>
                  </div>
                </div>
              </div>
            ) : (
              <button
                onClick={() => handlePresetClick(preset)}
                disabled={!connection.online}
                className="relative w-full p-3 bg-gradient-to-br from-purple-50 via-pink-50 to-purple-50 dark:from-botbot-darker/90 dark:via-botbot-darker/70 dark:to-botbot-darker/90 hover:from-purple-100 hover:via-pink-100 hover:to-purple-100 dark:hover:from-botbot-dark/90 dark:hover:via-botbot-dark/70 dark:hover:to-botbot-dark/90 rounded-2xl transition-all duration-200 disabled:opacity-50 disabled:cursor-not-allowed group border border-purple-100 dark:border-botbot-dark/50 hover:border-purple-200 dark:hover:border-botbot-accent/30 shadow-sm hover:shadow-lg hover:scale-[1.02] active:scale-[0.98]"
              >
                <div className="flex items-center gap-3">
                  <div className="p-2 bg-white dark:bg-botbot-dark/80 rounded-xl shadow-sm group-hover:shadow-md transition-all">
                    <span className="text-2xl block group-hover:scale-110 transition-transform">
                      {preset.emoji || '🎤'}
                    </span>
                  </div>
                  <div className="flex-1 text-left">
                    <p className="text-sm font-bold text-gray-800 dark:text-white">
                      {preset.label}
                    </p>
                    <p className="text-xs text-gray-500 dark:text-gray-400 truncate">
                      {preset.text}
                    </p>
                  </div>
                  <Volume2 className="w-4 h-4 text-purple-400 dark:text-purple-300 opacity-0 group-hover:opacity-100 transition-all duration-200" />
                </div>
                
                {/* Hover action buttons */}
                <div className="absolute -top-2 -right-2 flex gap-1 opacity-0 group-hover:opacity-100 transition-all duration-200 pointer-events-none group-hover:pointer-events-auto">
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      setEditingId(preset.id);
                      setEditingPreset(preset);
                    }}
                    className="p-1.5 bg-white dark:bg-botbot-dark rounded-full shadow-lg hover:shadow-xl hover:scale-110 transition-all duration-200 border border-gray-200 dark:border-gray-700"
                  >
                    <Edit2 className="w-3 h-3 text-gray-600 dark:text-gray-400" />
                  </button>
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      deletePreset(preset.id);
                    }}
                    className="p-1.5 bg-white dark:bg-botbot-dark rounded-full shadow-lg hover:shadow-xl hover:scale-110 transition-all duration-200 border border-gray-200 dark:border-gray-700"
                  >
                    <Trash2 className="w-3 h-3 text-red-500" />
                  </button>
                </div>
              </button>
            )}
          </div>
        ))}

        {/* Add new preset form */}
        {isAdding && (
          <div className="col-span-2 bg-gradient-to-br from-purple-50 to-pink-50 dark:from-botbot-darker dark:to-botbot-darker/80 rounded-2xl p-4 border border-purple-100 dark:border-botbot-dark/50 shadow-sm">
            <div className="space-y-3">
              <div className="flex items-center gap-2">
                <button
                  onClick={() => setShowEmojiPicker(!showEmojiPicker)}
                  className="relative p-2 bg-white dark:bg-botbot-dark rounded-xl hover:scale-110 transition-transform shadow-sm"
                >
                  <span className="text-xl">{newPreset.emoji}</span>
                </button>
                <input
                  type="text"
                  value={newPreset.label}
                  onChange={(e) => setNewPreset({ ...newPreset, label: e.target.value })}
                  className="flex-1 px-3 py-2 bg-white dark:bg-botbot-dark rounded-xl text-sm font-medium outline-none focus:ring-2 focus:ring-purple-500 dark:focus:ring-purple-400 shadow-sm"
                  placeholder="Preset name"
                />
              </div>
              
              {showEmojiPicker && (
                <div className="absolute z-50 mt-1 p-3 bg-white dark:bg-botbot-dark rounded-xl shadow-xl border border-gray-200 dark:border-gray-700 max-h-64 overflow-y-auto min-w-[280px]">
                  <div className="grid grid-cols-6 gap-1">
                    {EMOJI_OPTIONS.map((emoji) => (
                      <button
                        key={emoji}
                        onClick={() => {
                          setNewPreset({ ...newPreset, emoji });
                          setShowEmojiPicker(false);
                        }}
                        className="p-2 hover:bg-gray-100 dark:hover:bg-botbot-darker rounded-lg transition-colors"
                      >
                        <span className="text-lg">{emoji}</span>
                      </button>
                    ))}
                  </div>
                </div>
              )}
              
              <textarea
                value={newPreset.text}
                onChange={(e) => setNewPreset({ ...newPreset, text: e.target.value })}
                className="w-full px-3 py-2 bg-white dark:bg-botbot-dark rounded-xl text-sm outline-none focus:ring-2 focus:ring-purple-500 dark:focus:ring-purple-400 shadow-sm resize-none"
                placeholder="Text to speak"
                rows={2}
              />
              
              <div className="flex gap-2">
                <button
                  onClick={addPreset}
                  disabled={!newPreset.text || !newPreset.label}
                  className="flex-1 py-2 bg-gradient-to-r from-purple-500 to-pink-500 hover:from-purple-600 hover:to-pink-600 text-white rounded-xl text-sm font-bold transition-all duration-200 shadow-sm hover:shadow-md disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  Add Preset
                </button>
                <button
                  onClick={() => {
                    setIsAdding(false);
                    setNewPreset({ text: '', label: '', emoji: '🎤' });
                    setShowEmojiPicker(false);
                  }}
                  className="px-4 py-2 bg-gray-200 hover:bg-gray-300 dark:bg-botbot-dark dark:hover:bg-botbot-darker rounded-xl text-sm font-medium transition-colors"
                >
                  Cancel
                </button>
              </div>
            </div>
          </div>
        )}
      </div>

      {/* Empty state */}
      {presets.length === 0 && !isAdding && (
        <div className="text-center py-8">
          <div className="inline-flex p-4 bg-gradient-to-br from-purple-50 to-pink-50 dark:from-botbot-darker/50 dark:to-botbot-darker/30 rounded-full mb-4">
            <Mic className="w-8 h-8 text-purple-400 dark:text-purple-300" />
          </div>
          <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">No presets created yet</p>
          <button
            onClick={() => setIsAdding(true)}
            className="inline-flex items-center gap-2 px-4 py-2 bg-gradient-to-r from-purple-500 to-pink-500 hover:from-purple-600 hover:to-pink-600 text-white rounded-xl text-sm font-bold transition-all duration-200 shadow-sm hover:shadow-md"
          >
            <Plus className="w-4 h-4" />
            Create Your First Preset
          </button>
        </div>
      )}
    </div>
  );
}