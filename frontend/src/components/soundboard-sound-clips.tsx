'use client';

import { useState, useEffect, useRef } from 'react';
import { Trash2, Upload, Play, Pause, Edit2, Check, X, Loader2, Music, FileAudio, Plus, Sparkles, Mic, Square, Radio, Volume2 } from 'lucide-react';
import { createClient } from '@/utils/supabase/client';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { useRosPlayAudio } from '@/hooks/ros/useRosPlayAudio';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';

interface SoundClip {
  id: string;
  user_id: string;
  name: string;
  emoji: string;
  file_path: string;
  file_size: number;
  duration: number | null;
  mime_type: string;
  created_at: string;
  updated_at: string;
}

const EMOJI_OPTIONS = [
  // Sound & Music
  '🎵', '🎶', '🎼', '🎤', '🎧', '🎸', '🥁', '🎹', '🎺', '🎻',
  '🪕', '🎷', '🪗', '🪘', '🔔', '📯', '🎙️', '📻', '💿', '📀',
  // Audio Controls
  '🔊', '🔉', '🔈', '🔇', '📢', '📣', '🔕', '⏯️', '⏸️', '▶️',
  // Effects & Alerts
  '🚨', '🔥', '💥', '✨', '⚡', '🌟', '💫', '🎆', '🎇', '🎯',
  // Fun & Games
  '🎮', '🎲', '🎰', '🏆', '🥇', '🎉', '🎊', '🎈', '🎁', '🎪',
  // Nature & Animals
  '🐦', '🦆', '🦅', '🦉', '🐺', '🦁', '🐯', '🐻', '🐸', '🦗',
  // Weather & Nature
  '⛈️', '🌩️', '💨', '🌊', '🌀', '☄️', '🌈', '🌅', '🌠', '🌌',
  // Actions
  '💣', '🧨', '🔫', '⚔️', '🛡️', '🏹', '🥊', '🏃', '🚀', '🛸',
  // Misc
  '❤️', '💙', '💚', '💛', '💜', '🖤', '🤍', '💔', '❗', '❓'
];

export function SoundboardSoundClips() {
  const { user } = useSupabase();
  const supabase = createClient();
  const { playAudioOnRobot } = useRosPlayAudio();
  const { connectionStatus } = useRobotConnection();
  const [clips, setClips] = useState<SoundClip[]>([]);
  const [loading, setLoading] = useState(true);
  const [uploading, setUploading] = useState(false);
  const [editingClip, setEditingClip] = useState<string | null>(null);
  const [editName, setEditName] = useState('');
  const [editEmoji, setEditEmoji] = useState('');
  const [showEditEmojiPicker, setShowEditEmojiPicker] = useState(false);
  const [playingClip, setPlayingClip] = useState<string | null>(null);
  
  // Upload form states
  const [showUploadForm, setShowUploadForm] = useState(false);
  const [selectedFile, setSelectedFile] = useState<File | null>(null);
  const [uploadName, setUploadName] = useState('');
  const [uploadEmoji, setUploadEmoji] = useState('🎵');
  const [showUploadEmojiPicker, setShowUploadEmojiPicker] = useState(false);
  
  // Recording states
  const [isRecording, setIsRecording] = useState(false);
  const [showRecordingForm, setShowRecordingForm] = useState(false);
  const [recordingTime, setRecordingTime] = useState(0);
  const [recordedBlob, setRecordedBlob] = useState<Blob | null>(null);
  const [recordingName, setRecordingName] = useState('');
  const [recordingEmoji, setRecordingEmoji] = useState('🎤');
  const [showRecordingEmojiPicker, setShowRecordingEmojiPicker] = useState(false);
  const [waveformData, setWaveformData] = useState<number[]>(new Array(40).fill(0));
  
  const fileInputRef = useRef<HTMLInputElement>(null);
  const audioRef = useRef<HTMLAudioElement>(null);
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const audioContextRef = useRef<AudioContext | null>(null);
  const analyserRef = useRef<AnalyserNode | null>(null);
  const animationFrameRef = useRef<number | null>(null);
  const recordingIntervalRef = useRef<NodeJS.Timeout | null>(null);
  const chunksRef = useRef<Blob[]>([]);
  
  const [audioUrl, setAudioUrl] = useState<string | null>(null);

  useEffect(() => {
    if (user) {
      fetchClips();
    }
  }, [user]);

  useEffect(() => {
    return () => {
      // Cleanup on unmount
      if (recordingIntervalRef.current) {
        clearInterval(recordingIntervalRef.current);
      }
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
      if (audioContextRef.current) {
        audioContextRef.current.close();
      }
    };
  }, []);

  const fetchClips = async () => {
    if (!user) return;
    
    setLoading(true);
    try {
      const { data, error } = await supabase
        .from('sound_clips')
        .select('*')
        .eq('user_id', user.id)
        .order('created_at', { ascending: false });

      if (error) throw error;
      setClips(data || []);
    } catch (error) {
      console.error('Error fetching sound clips:', error);
    } finally {
      setLoading(false);
    }
  };

  const startRecording = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      
      // Create MediaRecorder
      const mediaRecorder = new MediaRecorder(stream, {
        mimeType: 'audio/webm'
      });
      mediaRecorderRef.current = mediaRecorder;
      chunksRef.current = [];
      
      // Setup audio visualization
      audioContextRef.current = new AudioContext();
      const source = audioContextRef.current.createMediaStreamSource(stream);
      analyserRef.current = audioContextRef.current.createAnalyser();
      analyserRef.current.fftSize = 256;
      source.connect(analyserRef.current);
      
      // Start visualization
      visualizeAudio();
      
      mediaRecorder.ondataavailable = (event) => {
        if (event.data.size > 0) {
          chunksRef.current.push(event.data);
        }
      };
      
      mediaRecorder.onstop = () => {
        const blob = new Blob(chunksRef.current, { type: 'audio/webm' });
        setRecordedBlob(blob);
        
        // Stop all tracks
        stream.getTracks().forEach(track => track.stop());
        
        // Stop visualization
        if (animationFrameRef.current) {
          cancelAnimationFrame(animationFrameRef.current);
        }
        
        // Reset waveform
        setWaveformData(new Array(40).fill(0));
      };
      
      mediaRecorder.start();
      setIsRecording(true);
      setShowRecordingForm(true);
      setRecordingTime(0);
      
      // Generate default name with timestamp
      const now = new Date();
      const defaultName = `Recording ${now.toLocaleTimeString()}`;
      setRecordingName(defaultName);
      
      // Start timer
      recordingIntervalRef.current = setInterval(() => {
        setRecordingTime(prev => prev + 1);
      }, 1000);
      
    } catch (error) {
      console.error('Error starting recording:', error);
      alert('Failed to access microphone. Please check your permissions.');
    }
  };

  const visualizeAudio = () => {
    if (!analyserRef.current) return;
    
    const bufferLength = analyserRef.current.frequencyBinCount;
    const dataArray = new Uint8Array(bufferLength);
    
    const draw = () => {
      if (!isRecording && !analyserRef.current) return;
      
      animationFrameRef.current = requestAnimationFrame(draw);
      analyserRef.current!.getByteFrequencyData(dataArray);
      
      // Sample 40 points from the frequency data
      const samples = 40;
      const blockSize = Math.floor(bufferLength / samples);
      const sampledData: number[] = [];
      
      for (let i = 0; i < samples; i++) {
        let sum = 0;
        for (let j = 0; j < blockSize; j++) {
          sum += dataArray[i * blockSize + j];
        }
        sampledData.push((sum / blockSize) / 255); // Normalize to 0-1
      }
      
      setWaveformData(sampledData);
    };
    
    draw();
  };

  const stopRecording = () => {
    if (mediaRecorderRef.current && mediaRecorderRef.current.state !== 'inactive') {
      mediaRecorderRef.current.stop();
      setIsRecording(false);
      
      if (recordingIntervalRef.current) {
        clearInterval(recordingIntervalRef.current);
      }
    }
  };

  const handleRecordingUpload = async () => {
    if (!recordedBlob || !user || !recordingName.trim()) return;
    
    setUploading(true);
    
    try {
      // Convert webm to a more compatible format if needed
      const file = new File([recordedBlob], `recording_${Date.now()}.webm`, {
        type: 'audio/webm'
      });
      
      // Generate unique file path
      const fileName = `${Date.now()}_${Math.random().toString(36).substring(7)}.webm`;
      const filePath = `${user.id}/${fileName}`;
      
      // Upload file to storage
      const { data: uploadData, error: uploadError } = await supabase.storage
        .from('sound-clips')
        .upload(filePath, file, {
          cacheControl: '3600',
          upsert: false
        });
      
      if (uploadError) throw uploadError;
      
      // Get audio duration
      let duration: number | null = null;
      try {
        const audioElement = new Audio();
        audioElement.src = URL.createObjectURL(recordedBlob);
        await new Promise((resolve) => {
          audioElement.addEventListener('loadedmetadata', () => {
            duration = audioElement.duration;
            resolve(true);
          });
        });
      } catch (e) {
        console.log('Could not get audio duration:', e);
      }
      
      // Create database record
      const { data: clipData, error: dbError } = await supabase
        .from('sound_clips')
        .insert({
          user_id: user.id,
          name: recordingName.trim(),
          emoji: recordingEmoji,
          file_path: filePath,
          file_size: recordedBlob.size,
          duration: duration,
          mime_type: 'audio/webm'
        })
        .select()
        .single();
      
      if (dbError) throw dbError;
      
      // Add to local state
      setClips([clipData, ...clips]);
      
      // Reset recording form
      setShowRecordingForm(false);
      setRecordedBlob(null);
      setRecordingName('');
      setRecordingEmoji('🎤');
      setRecordingTime(0);
      
    } catch (error) {
      console.error('Error uploading recording:', error);
      alert('Failed to upload recording');
    } finally {
      setUploading(false);
    }
  };

  const cancelRecording = () => {
    if (isRecording) {
      stopRecording();
    }
    setShowRecordingForm(false);
    setRecordedBlob(null);
    setRecordingName('');
    setRecordingEmoji('🎤');
    setRecordingTime(0);
  };

  const formatRecordingTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins}:${secs.toString().padStart(2, '0')}`;
  };

  const handleFileSelect = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    // Check file size (1GB limit)
    if (file.size > 1024 * 1024 * 1024) {
      alert('File size must be less than 1GB');
      return;
    }

    // Check if it's an audio file
    if (!file.type.startsWith('audio/')) {
      alert('Please upload an audio file');
      return;
    }

    setSelectedFile(file);
    // Set default name from filename (without extension)
    const defaultName = file.name.replace(/\.[^/.]+$/, '');
    setUploadName(defaultName);
    setShowUploadForm(true);
  };

  const handleUpload = async () => {
    if (!selectedFile || !user || !uploadName.trim()) return;

    setUploading(true);

    try {
      // Generate unique file path
      const fileExt = selectedFile.name.split('.').pop();
      const fileName = `${Date.now()}_${Math.random().toString(36).substring(7)}.${fileExt}`;
      const filePath = `${user.id}/${fileName}`;

      // Upload file to storage
      const { data: uploadData, error: uploadError } = await supabase.storage
        .from('sound-clips')
        .upload(filePath, selectedFile, {
          cacheControl: '3600',
          upsert: false
        });

      if (uploadError) throw uploadError;

      // Get audio duration if possible
      let duration: number | null = null;
      try {
        const audioElement = new Audio();
        audioElement.src = URL.createObjectURL(selectedFile);
        await new Promise((resolve) => {
          audioElement.addEventListener('loadedmetadata', () => {
            duration = audioElement.duration;
            resolve(true);
          });
        });
      } catch (e) {
        console.log('Could not get audio duration:', e);
      }

      // Create database record
      const { data: clipData, error: dbError } = await supabase
        .from('sound_clips')
        .insert({
          user_id: user.id,
          name: uploadName.trim(),
          emoji: uploadEmoji,
          file_path: filePath,
          file_size: selectedFile.size,
          duration: duration,
          mime_type: selectedFile.type
        })
        .select()
        .single();

      if (dbError) throw dbError;

      // Add to local state
      setClips([clipData, ...clips]);
      
      // Reset upload form
      setShowUploadForm(false);
      setSelectedFile(null);
      setUploadName('');
      setUploadEmoji('🎵');
      if (fileInputRef.current) {
        fileInputRef.current.value = '';
      }
    } catch (error) {
      console.error('Error uploading file:', error);
      alert('Failed to upload sound clip');
    } finally {
      setUploading(false);
    }
  };

  const cancelUpload = () => {
    setShowUploadForm(false);
    setSelectedFile(null);
    setUploadName('');
    setUploadEmoji('🎵');
    if (fileInputRef.current) {
      fileInputRef.current.value = '';
    }
  };

  const handleDelete = async (clip: SoundClip) => {
    if (!confirm(`Delete "${clip.name}"?`)) return;

    try {
      // Delete from storage
      const { error: storageError } = await supabase.storage
        .from('sound-clips')
        .remove([clip.file_path]);

      if (storageError) console.error('Storage delete error:', storageError);

      // Delete from database
      const { error: dbError } = await supabase
        .from('sound_clips')
        .delete()
        .eq('id', clip.id);

      if (dbError) throw dbError;

      // Remove from local state
      setClips(clips.filter(c => c.id !== clip.id));
    } catch (error) {
      console.error('Error deleting sound clip:', error);
      alert('Failed to delete sound clip');
    }
  };

  const handleEdit = (clip: SoundClip) => {
    setEditingClip(clip.id);
    setEditName(clip.name);
    setEditEmoji(clip.emoji);
    setShowEditEmojiPicker(false);
  };

  const handleSaveEdit = async () => {
    if (!editingClip || !editName.trim()) return;

    try {
      const { error } = await supabase
        .from('sound_clips')
        .update({
          name: editName.trim(),
          emoji: editEmoji
        })
        .eq('id', editingClip);

      if (error) throw error;

      // Update local state
      setClips(clips.map(c => 
        c.id === editingClip 
          ? { ...c, name: editName.trim(), emoji: editEmoji }
          : c
      ));

      setEditingClip(null);
    } catch (error) {
      console.error('Error updating sound clip:', error);
      alert('Failed to update sound clip');
    }
  };

  const handleCancelEdit = () => {
    setEditingClip(null);
    setEditName('');
    setEditEmoji('');
    setShowEditEmojiPicker(false);
  };

  const handlePlayInBrowser = async (clip: SoundClip) => {
    if (playingClip === clip.id) {
      // Stop playing
      if (audioRef.current) {
        audioRef.current.pause();
        audioRef.current.currentTime = 0;
      }
      setPlayingClip(null);
      setAudioUrl(null);
    } else {
      // Start playing
      try {
        // Get signed URL for the audio file
        const { data, error } = await supabase.storage
          .from('sound-clips')
          .createSignedUrl(clip.file_path, 60); // 60 seconds expiry

        if (error) throw error;

        setAudioUrl(data.signedUrl);
        setPlayingClip(clip.id);

        // Play audio
        if (audioRef.current) {
          audioRef.current.src = data.signedUrl;
          audioRef.current.play();
        }
      } catch (error) {
        console.error('Error playing sound clip:', error);
        alert('Failed to play sound clip');
      }
    }
  };

  const handlePlayOnRobot = (clip: SoundClip) => {
    // Call the ROS service to play on robot
    playAudioOnRobot(clip.id, clip.name);
    
    // Visual feedback - briefly highlight the clip
    const element = document.getElementById(`clip-${clip.id}`);
    if (element) {
      element.classList.add('ring-2', 'ring-botbot-accent', 'ring-offset-2');
      setTimeout(() => {
        element.classList.remove('ring-2', 'ring-botbot-accent', 'ring-offset-2');
      }, 500);
    }
  };

  const formatFileSize = (bytes: number) => {
    if (bytes < 1024) return bytes + ' B';
    if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB';
    if (bytes < 1024 * 1024 * 1024) return (bytes / (1024 * 1024)).toFixed(1) + ' MB';
    return (bytes / (1024 * 1024 * 1024)).toFixed(2) + ' GB';
  };

  const formatDuration = (seconds: number | null) => {
    if (!seconds) return '';
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins}:${secs.toString().padStart(2, '0')}`;
  };

  if (!user) {
    return (
      <div className="flex items-center justify-center h-full text-gray-500">
        <p>Please log in to manage sound clips</p>
      </div>
    );
  }

  return (
    <div className="h-full flex flex-col bg-gray-50 dark:bg-botbot-darkest rounded-lg">
      {/* Header */}
      <div className="flex items-center justify-between p-3 border-b border-gray-200 dark:border-gray-700">
        <h3 className="text-sm font-semibold text-gray-800 dark:text-white flex items-center">
          <Sparkles className="w-4 h-4 mr-2 text-botbot-accent" />
          Sound Clips
        </h3>
        {!showUploadForm && !showRecordingForm && (
          <div className="flex items-center space-x-2">
            <button
              onClick={startRecording}
              className="flex items-center px-3 py-1 bg-red-500 text-white text-xs rounded-full hover:bg-red-600 transition-all hover:scale-105"
            >
              <Mic className="w-3 h-3 mr-1" />
              Record
            </button>
            <button
              onClick={() => fileInputRef.current?.click()}
              className="flex items-center px-3 py-1 bg-botbot-accent text-white text-xs rounded-full hover:bg-opacity-90 transition-all hover:scale-105"
            >
              <Plus className="w-3 h-3 mr-1" />
              Add Clip
            </button>
          </div>
        )}
      </div>

      {/* Recording Form */}
      {showRecordingForm && (
        <div className="p-3 border-b border-gray-200 dark:border-gray-700 bg-gradient-to-r from-red-50 to-pink-50 dark:from-red-900/20 dark:to-pink-900/20">
          <div className="space-y-3">
            {/* Recording status */}
            <div className="flex items-center justify-between">
              <div className="flex items-center space-x-3">
                {isRecording ? (
                  <>
                    <div className="relative">
                      <Radio className="w-5 h-5 text-red-500 animate-pulse" />
                      <div className="absolute inset-0 bg-red-500 rounded-full animate-ping opacity-75"></div>
                    </div>
                    <span className="text-sm font-medium text-red-600 dark:text-red-400">
                      Recording... {formatRecordingTime(recordingTime)}
                    </span>
                  </>
                ) : (
                  <>
                    <FileAudio className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                    <span className="text-sm font-medium text-gray-600 dark:text-gray-400">
                      Recording ready ({formatRecordingTime(recordingTime)})
                    </span>
                  </>
                )}
              </div>
              {isRecording && (
                <button
                  onClick={stopRecording}
                  className="p-2 bg-red-500 text-white rounded-lg hover:bg-red-600 transition-colors"
                >
                  <Square className="w-4 h-4" />
                </button>
              )}
            </div>

            {/* Waveform visualization */}
            {isRecording && (
              <div className="flex items-center justify-center space-x-1 h-16 bg-white/50 dark:bg-black/20 rounded-lg p-2">
                {waveformData.map((value, index) => (
                  <div
                    key={index}
                    className="flex-1 bg-gradient-to-t from-red-500 to-pink-500 rounded-full transition-all duration-100"
                    style={{
                      height: `${Math.max(4, value * 100)}%`,
                      opacity: 0.8 + value * 0.2
                    }}
                  />
                ))}
              </div>
            )}

            {/* Name and emoji input (only show after recording stops) */}
            {!isRecording && recordedBlob && (
              <div className="flex items-center space-x-2">
                {/* Emoji picker */}
                <div className="relative">
                  <button
                    onClick={() => setShowRecordingEmojiPicker(!showRecordingEmojiPicker)}
                    className="text-xl p-1.5 rounded-lg bg-gray-100 dark:bg-botbot-darker hover:bg-gray-200 dark:hover:bg-gray-700 transition-colors"
                  >
                    {recordingEmoji}
                  </button>
                  {showRecordingEmojiPicker && (
                    <div className="absolute top-10 left-0 z-50 bg-white dark:bg-botbot-darkest border border-gray-200 dark:border-gray-700 rounded-xl p-3 shadow-xl max-h-64 overflow-y-auto min-w-[320px]">
                      <div className="grid grid-cols-6 gap-1">
                        {EMOJI_OPTIONS.map((emoji) => (
                          <button
                            key={emoji}
                            onClick={() => {
                              setRecordingEmoji(emoji);
                              setShowRecordingEmojiPicker(false);
                            }}
                            className="p-2 text-lg hover:bg-gray-100 dark:hover:bg-gray-700 rounded-lg transition-colors"
                          >
                            {emoji}
                          </button>
                        ))}
                      </div>
                    </div>
                  )}
                </div>

                {/* Name input */}
                <input
                  type="text"
                  value={recordingName}
                  onChange={(e) => setRecordingName(e.target.value)}
                  className="flex-1 bg-white dark:bg-botbot-darker text-gray-800 dark:text-white px-3 py-1.5 rounded-lg border border-gray-200 dark:border-gray-700 focus:border-botbot-accent focus:outline-none text-sm"
                  placeholder="Recording name..."
                  autoFocus
                />

                {/* Action buttons */}
                <div className="flex items-center space-x-1">
                  <button
                    onClick={handleRecordingUpload}
                    disabled={uploading || !recordingName.trim()}
                    className="px-3 py-1.5 bg-green-500 text-white rounded-lg hover:bg-green-600 transition-colors disabled:opacity-50 disabled:cursor-not-allowed text-sm flex items-center"
                  >
                    {uploading ? (
                      <>
                        <Loader2 className="w-3 h-3 mr-1 animate-spin" />
                        Saving...
                      </>
                    ) : (
                      <>
                        <Upload className="w-3 h-3 mr-1" />
                        Save
                      </>
                    )}
                  </button>
                  <button
                    onClick={cancelRecording}
                    className="px-3 py-1.5 bg-gray-500 text-white rounded-lg hover:bg-gray-600 transition-colors text-sm"
                  >
                    Cancel
                  </button>
                </div>
              </div>
            )}
          </div>
        </div>
      )}

      {/* Upload Form */}
      {showUploadForm && (
        <div className="p-3 border-b border-gray-200 dark:border-gray-700 bg-gray-50 dark:bg-botbot-darker/50">
          <div className="space-y-3">
            {/* File info */}
            <div className="flex items-center space-x-2 text-xs text-gray-600 dark:text-gray-400">
              <FileAudio className="w-4 h-4" />
              <span className="font-medium">{selectedFile?.name}</span>
              <span>({formatFileSize(selectedFile?.size || 0)})</span>
            </div>

            {/* Name and emoji input */}
            <div className="flex items-center space-x-2">
              {/* Emoji picker */}
              <div className="relative">
                <button
                  onClick={() => setShowUploadEmojiPicker(!showUploadEmojiPicker)}
                  className="text-xl p-1.5 rounded-lg bg-gray-100 dark:bg-botbot-darker hover:bg-gray-200 dark:hover:bg-gray-700 transition-colors"
                >
                  {uploadEmoji}
                </button>
                {showUploadEmojiPicker && (
                  <div className="absolute top-10 left-0 z-50 bg-white dark:bg-botbot-darkest border border-gray-200 dark:border-gray-700 rounded-xl p-3 shadow-xl max-h-64 overflow-y-auto min-w-[320px]">
                    <div className="grid grid-cols-6 gap-1">
                      {EMOJI_OPTIONS.map((emoji) => (
                        <button
                          key={emoji}
                          onClick={() => {
                            setUploadEmoji(emoji);
                            setShowUploadEmojiPicker(false);
                          }}
                          className="p-2 text-lg hover:bg-gray-100 dark:hover:bg-gray-700 rounded-lg transition-colors"
                        >
                          {emoji}
                        </button>
                      ))}
                    </div>
                  </div>
                )}
              </div>

              {/* Name input */}
              <input
                type="text"
                value={uploadName}
                onChange={(e) => setUploadName(e.target.value)}
                className="flex-1 bg-white dark:bg-botbot-darker text-gray-800 dark:text-white px-3 py-1.5 rounded-lg border border-gray-200 dark:border-gray-700 focus:border-botbot-accent focus:outline-none text-sm"
                placeholder="Sound clip name..."
                autoFocus
              />

              {/* Action buttons */}
              <div className="flex items-center space-x-1">
                <button
                  onClick={handleUpload}
                  disabled={uploading || !uploadName.trim()}
                  className="px-3 py-1.5 bg-green-500 text-white rounded-lg hover:bg-green-600 transition-colors disabled:opacity-50 disabled:cursor-not-allowed text-sm flex items-center"
                >
                  {uploading ? (
                    <>
                      <Loader2 className="w-3 h-3 mr-1 animate-spin" />
                      Uploading...
                    </>
                  ) : (
                    <>
                      <Upload className="w-3 h-3 mr-1" />
                      Upload
                    </>
                  )}
                </button>
                <button
                  onClick={cancelUpload}
                  className="px-3 py-1.5 bg-gray-500 text-white rounded-lg hover:bg-gray-600 transition-colors text-sm"
                >
                  Cancel
                </button>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Clips List */}
      <div className="flex-1 overflow-y-auto p-3">
        {loading ? (
          <div className="flex items-center justify-center h-32">
            <Loader2 className="w-6 h-6 text-gray-500 animate-spin" />
          </div>
        ) : clips.length === 0 ? (
          <div className="flex flex-col items-center justify-center h-full text-gray-500 dark:text-gray-400">
            <Music className="w-12 h-12 mb-3 opacity-50" />
            <p className="text-sm font-medium">No sound clips yet</p>
            <p className="text-xs mt-1">Click "Record" or "Add Clip" to get started</p>
          </div>
        ) : (
          <div className="grid grid-cols-2 gap-3">
            {clips.map((clip) => (
              <div key={clip.id} className="group relative">
                {editingClip === clip.id ? (
                  // Edit mode
                  <div className="relative bg-gradient-to-br from-gray-50 to-gray-100 dark:from-botbot-darker dark:to-botbot-darker/80 rounded-2xl p-3 border border-gray-200 dark:border-gray-700 shadow-sm">
                    <div className="space-y-2">
                      <div className="flex items-center gap-2">
                        <button
                          onClick={() => setShowEditEmojiPicker(!showEditEmojiPicker)}
                          className="relative p-2 bg-white dark:bg-botbot-dark rounded-xl hover:scale-110 transition-transform"
                        >
                          <span className="text-xl">{editEmoji}</span>
                        </button>
                        <input
                          type="text"
                          value={editName}
                          onChange={(e) => setEditName(e.target.value)}
                          className="flex-1 px-3 py-1.5 bg-white dark:bg-botbot-dark rounded-xl text-sm font-medium outline-none focus:ring-2 focus:ring-purple-500 dark:focus:ring-purple-400"
                          placeholder="Clip name"
                          autoFocus
                        />
                      </div>
                      
                      {showEditEmojiPicker && (
                        <div className="absolute top-14 left-0 z-50 p-3 bg-white dark:bg-botbot-dark rounded-xl shadow-xl border border-gray-200 dark:border-gray-700 max-h-64 overflow-y-auto min-w-[280px]">
                          <div className="grid grid-cols-6 gap-1">
                            {EMOJI_OPTIONS.map((emoji) => (
                              <button
                                key={emoji}
                                onClick={() => {
                                  setEditEmoji(emoji);
                                  setShowEditEmojiPicker(false);
                                }}
                                className="p-2 hover:bg-gray-100 dark:hover:bg-botbot-darker rounded-lg transition-colors"
                              >
                                <span className="text-lg">{emoji}</span>
                              </button>
                            ))}
                          </div>
                        </div>
                      )}
                      
                      <div className="text-xs text-gray-500 dark:text-gray-400 px-1">
                        {formatFileSize(clip.file_size)}
                        {clip.duration && ` • ${formatDuration(clip.duration)}`}
                      </div>
                      
                      <div className="flex gap-2">
                        <button
                          onClick={handleSaveEdit}
                          className="flex-1 py-1.5 bg-gradient-to-r from-green-500 to-emerald-500 hover:from-green-600 hover:to-emerald-600 text-white rounded-xl text-sm font-medium transition-all duration-200 shadow-sm hover:shadow-md"
                        >
                          Save
                        </button>
                        <button
                          onClick={handleCancelEdit}
                          className="px-3 py-1.5 bg-gray-200 hover:bg-gray-300 dark:bg-botbot-dark dark:hover:bg-botbot-darker rounded-xl text-sm transition-colors"
                        >
                          Cancel
                        </button>
                      </div>
                    </div>
                  </div>
                ) : (
                  // Display mode
                  <div
                    id={`clip-${clip.id}`}
                    className="relative w-full"
                  >
                    <button
                      onClick={() => handlePlayOnRobot(clip)}
                      disabled={connectionStatus !== 'connected'}
                      className={`relative w-full p-3 bg-gradient-to-br from-purple-50 via-pink-50 to-purple-50 dark:from-botbot-darker/90 dark:via-botbot-darker/70 dark:to-botbot-darker/90 hover:from-purple-100 hover:via-pink-100 hover:to-purple-100 dark:hover:from-botbot-dark/90 dark:hover:via-botbot-dark/70 dark:hover:to-botbot-dark/90 rounded-2xl transition-all duration-200 group border border-purple-100 dark:border-botbot-dark/50 hover:border-purple-200 dark:hover:border-botbot-accent/30 shadow-sm hover:shadow-lg hover:scale-[1.02] active:scale-[0.98] focus:outline-none focus-visible:ring-2 focus-visible:ring-purple-500 focus-visible:ring-offset-2 ${
                        connectionStatus !== 'connected' ? 'opacity-60 cursor-not-allowed' : ''
                      }`}
                      title={connectionStatus !== 'connected' ? 'Connect to robot to play on robot' : `Play "${clip.name}" on robot`}
                    >
                      <div className="flex items-center gap-3">
                        <div className="p-2 bg-white dark:bg-botbot-dark/80 rounded-xl shadow-sm group-hover:shadow-md transition-all">
                          <span className="text-2xl block group-hover:scale-110 transition-transform">
                            {clip.emoji}
                          </span>
                        </div>
                        <div className="flex-1 text-left">
                          <p className="text-sm font-bold text-gray-800 dark:text-white">
                            {clip.name}
                          </p>
                          <p className="text-xs text-gray-500 dark:text-gray-400">
                            {formatFileSize(clip.file_size)}
                            {clip.duration && ` • ${formatDuration(clip.duration)}`}
                          </p>
                        </div>
                        <div className="flex items-center gap-1">
                          {/* Play in browser button */}
                          <button
                            onClick={(e) => {
                              e.stopPropagation();
                              handlePlayInBrowser(clip);
                            }}
                            className="p-1.5 bg-white/80 dark:bg-botbot-dark/80 rounded-lg hover:bg-white dark:hover:bg-botbot-dark transition-all focus:outline-none"
                            title="Play in browser"
                          >
                            {playingClip === clip.id ? (
                              <Pause className="w-4 h-4 text-purple-500 dark:text-purple-400" />
                            ) : (
                              <Volume2 className="w-4 h-4 text-purple-500 dark:text-purple-400" />
                            )}
                          </button>
                        </div>
                      </div>
                      
                      {/* Robot play indicator */}
                      {connectionStatus === 'connected' && (
                        <div className="absolute inset-0 flex items-center justify-center pointer-events-none opacity-0 group-hover:opacity-100 transition-opacity">
                          <div className="bg-black/70 text-white px-3 py-1 rounded-full text-xs font-medium">
                            Click to play on robot
                          </div>
                        </div>
                      )}
                    </button>
                    
                    {/* Hover action buttons */}
                    <div className="absolute -top-2 -right-2 flex gap-1 opacity-0 group-hover:opacity-100 transition-all duration-200 z-10">
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          handleEdit(clip);
                        }}
                        className="p-1.5 bg-white dark:bg-botbot-dark rounded-full shadow-lg hover:shadow-xl hover:scale-110 transition-all duration-200 border border-gray-200 dark:border-gray-700 focus:outline-none"
                      >
                        <Edit2 className="w-3 h-3 text-gray-600 dark:text-gray-400" />
                      </button>
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          handleDelete(clip);
                        }}
                        className="p-1.5 bg-white dark:bg-botbot-dark rounded-full shadow-lg hover:shadow-xl hover:scale-110 transition-all duration-200 border border-gray-200 dark:border-gray-700 focus:outline-none"
                      >
                        <Trash2 className="w-3 h-3 text-red-500" />
                      </button>
                    </div>
                  </div>
                )}
              </div>
            ))}
          </div>
        )}
      </div>

      {/* Hidden file input */}
      <input
        ref={fileInputRef}
        type="file"
        accept="audio/*"
        onChange={handleFileSelect}
        className="hidden"
      />

      {/* Hidden audio element */}
      <audio
        ref={audioRef}
        onEnded={() => {
          setPlayingClip(null);
          setAudioUrl(null);
        }}
        className="hidden"
      />
    </div>
  );
}