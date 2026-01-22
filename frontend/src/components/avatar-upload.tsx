'use client';

import { useState, useRef, useEffect } from 'react';
import { User, Camera, Trash2, Loader2 } from 'lucide-react';
import { useSupabase } from '@/contexts/SupabaseProvider';
import { useLanguage } from '@/contexts/LanguageContext';
import { AuditLogger } from '@/utils/audit-logger';

interface AvatarUploadProps {
  avatarUrl: string | null;
  userId: string;
  onAvatarUpdate: (url: string | null) => void;
}

export default function AvatarUpload({ avatarUrl, userId, onAvatarUpdate }: AvatarUploadProps) {
  const { supabase } = useSupabase();
  const { t } = useLanguage();
  const fileInputRef = useRef<HTMLInputElement>(null);
  const [uploading, setUploading] = useState(false);
  const [previewUrl, setPreviewUrl] = useState<string | null>(avatarUrl);

  // Update preview when avatarUrl prop changes
  useEffect(() => {
    setPreviewUrl(avatarUrl);
  }, [avatarUrl]);

  const handleFileSelect = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    if (!supabase) {
      alert('Database connection not available');
      return;
    }

    // Check authentication
    const { data: { session }, error: sessionError } = await supabase.auth.getSession();
    if (sessionError || !session) {
      alert('You must be logged in to upload an avatar');
      return;
    }

    // Validate file type
    const validTypes = ['image/jpeg', 'image/jpg', 'image/png', 'image/webp', 'image/gif'];
    if (!validTypes.includes(file.type)) {
      alert('Please select a valid image file (JPEG, PNG, WebP, or GIF)');
      return;
    }

    // Validate file size (5MB)
    if (file.size > 5 * 1024 * 1024) {
      alert('File size must be less than 5MB');
      return;
    }

    setUploading(true);

    try {
      // Create a unique filename
      const fileExt = file.name.split('.').pop();
      const fileName = `${userId}-${Date.now()}.${fileExt}`;
      const filePath = `${userId}/${fileName}`;

      // Delete old avatar if exists
      if (avatarUrl) {
        const oldPath = avatarUrl.split('/').slice(-2).join('/');
        const { error: deleteError } = await supabase.storage.from('profile-pictures').remove([oldPath]);
        if (deleteError) {
          console.warn('Failed to delete old avatar:', deleteError);
        }
      }

      // Upload new avatar
      const { error: uploadError, data } = await supabase.storage
        .from('profile-pictures')
        .upload(filePath, file, {
          cacheControl: '3600',
          upsert: false
        });

      if (uploadError) {
        console.error('Upload error details:', uploadError);
        throw uploadError;
      }

      // For private bucket, we need to create a signed URL
      const { data: signedUrlData, error: signedUrlError } = await supabase.storage
        .from('profile-pictures')
        .createSignedUrl(filePath, 31536000); // 1 year expiry

      if (signedUrlError) {
        throw signedUrlError;
      }

      setPreviewUrl(signedUrlData.signedUrl);
      onAvatarUpdate(signedUrlData.signedUrl);
      
      // Log avatar upload
      await AuditLogger.getInstance().log({
        event_type: 'system',
        event_action: 'profile_updated',
        event_details: {
          action: 'avatar_uploaded',
          oldAvatarUrl: avatarUrl,
          newAvatarUrl: signedUrlData.signedUrl,
          fileName: fileName,
          fileSize: file.size,
          fileType: file.type
        }
      });
    } catch (error: any) {
      console.error('Error uploading avatar:', error);
      const errorMessage = error?.message || error?.error_description || 'Error uploading avatar. Please try again.';
      alert(errorMessage);
    } finally {
      setUploading(false);
    }
  };

  const handleDelete = async () => {
    if (!avatarUrl || !supabase) return;

    const confirmDelete = window.confirm('Are you sure you want to delete your avatar?');
    if (!confirmDelete) return;

    setUploading(true);

    try {
      // Extract the file path from URL
      const filePath = avatarUrl.split('/').slice(-2).join('/');
      
      // Delete from storage
      const { error } = await supabase.storage
        .from('profile-pictures')
        .remove([filePath]);

      if (error) throw error;

      setPreviewUrl(null);
      onAvatarUpdate(null);
      
      // Log avatar deletion
      await AuditLogger.getInstance().log({
        event_type: 'system',
        event_action: 'profile_updated',
        event_details: {
          action: 'avatar_deleted',
          deletedAvatarUrl: avatarUrl,
          filePath: filePath
        }
      });
    } catch (error) {
      console.error('Error deleting avatar:', error);
      alert('Error deleting avatar. Please try again.');
    } finally {
      setUploading(false);
    }
  };

  return (
    <div className="flex flex-col items-center space-y-4">
      <div className="relative group">
        {/* Avatar Display - square container with proper aspect ratio support */}
        <div className="w-32 h-32 overflow-hidden bg-gray-200 dark:bg-botbot-dark border-4 border-white dark:border-botbot-darker shadow-lg flex items-center justify-center">
          {previewUrl ? (
            <img 
              src={previewUrl} 
              alt="Avatar" 
              className="max-w-full max-h-full object-contain"
            />
          ) : (
            <div className="w-full h-full flex items-center justify-center">
              <User className="w-16 h-16 text-gray-400 dark:text-gray-600" />
            </div>
          )}
        </div>

        {/* Upload Overlay */}
        <div 
          className="absolute inset-0 flex items-center justify-center bg-black/50 opacity-0 group-hover:opacity-100 transition-opacity cursor-pointer"
          onClick={() => fileInputRef.current?.click()}
        >
          {uploading ? (
            <Loader2 className="w-8 h-8 text-white animate-spin" />
          ) : (
            <Camera className="w-8 h-8 text-white" />
          )}
        </div>

        {/* Delete Button */}
        {previewUrl && !uploading && (
          <button
            onClick={handleDelete}
            className="absolute -top-2 -right-2 p-2 bg-red-600 hover:bg-red-700 text-white rounded-full shadow-lg transition-colors"
            title="Delete"
          >
            <Trash2 className="w-4 h-4" />
          </button>
        )}
      </div>

      {/* Hidden File Input */}
      <input
        ref={fileInputRef}
        type="file"
        accept="image/jpeg,image/jpg,image/png,image/webp,image/gif"
        onChange={handleFileSelect}
        className="hidden"
        disabled={uploading}
      />

      {/* Upload Button (Alternative) */}
      <button
        onClick={() => fileInputRef.current?.click()}
        disabled={uploading}
        className="px-4 py-2 text-sm font-medium text-white bg-primary dark:bg-botbot-accent hover:bg-primary/90 dark:hover:bg-botbot-accent/90 rounded-md transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
      >
        {uploading ? (
          <>
            <Loader2 className="w-4 h-4 inline-block mr-2 animate-spin" />
            Uploading...
          </>
        ) : (
          previewUrl ? 'Change Avatar' : 'Upload Avatar'
        )}
      </button>

      <p className="text-xs text-gray-500 dark:text-gray-400 text-center">
        Click on the avatar or button to upload. Max 5MB. JPEG, PNG, WebP, or GIF.
      </p>
    </div>
  );
} 