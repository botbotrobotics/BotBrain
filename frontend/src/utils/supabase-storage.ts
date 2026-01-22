import { SupabaseClient } from '@supabase/supabase-js';

/**
 * Extract the storage path from a full URL or return the path as-is if it's already relative
 */
export function extractStoragePath(urlOrPath: string | null): string | null {
  if (!urlOrPath || urlOrPath === 'n/a') {
    return null;
  }

  // If it's already a relative path, return it
  if (!urlOrPath.startsWith('http')) {
    return urlOrPath;
  }

  // Extract path from full URL
  try {
    const url = new URL(urlOrPath);
    const pathMatch = url.pathname.match(/\/storage\/v1\/object\/public\/(.+)/);
    if (pathMatch) {
      return pathMatch[1];
    }
    
    // Try authenticated path pattern
    const authPathMatch = url.pathname.match(/\/storage\/v1\/object\/sign\/(.+)/);
    if (authPathMatch) {
      return authPathMatch[1];
    }

    // If it's a storage URL but doesn't match patterns, try to extract the path after /storage/v1/object/
    if (url.pathname.includes('/storage/v1/object/')) {
      const parts = url.pathname.split('/storage/v1/object/');
      if (parts[1]) {
        // Remove 'public/' or 'sign/' prefix if present
        return parts[1].replace(/^(public|sign)\//, '');
      }
    }
  } catch (e) {
    console.error('Error parsing URL:', e);
  }

  return null;
}

/**
 * Get a signed URL for a protected storage object
 */
export async function getSignedImageUrl(
  supabase: SupabaseClient,
  urlOrPath: string | null,
  expiresIn: number = 3600 // 1 hour default
): Promise<string | null> {
  const path = extractStoragePath(urlOrPath);
  
  if (!path) {
    return null;
  }

  try {
    // For yolo_images bucket
    const { data, error } = await supabase.storage
      .from('yolo_images')
      .createSignedUrl(path.replace('yolo_images/', ''), expiresIn);

    if (error) {
      console.error('Error creating signed URL:', error);
      return null;
    }

    return data?.signedUrl || null;
  } catch (e) {
    console.error('Error getting signed URL:', e);
    return null;
  }
}

/**
 * Get multiple signed URLs in batch
 */
export async function getSignedImageUrls(
  supabase: SupabaseClient,
  urlsOrPaths: (string | null)[],
  expiresIn: number = 3600
): Promise<(string | null)[]> {
  const promises = urlsOrPaths.map(url => getSignedImageUrl(supabase, url, expiresIn));
  return Promise.all(promises);
}

/**
 * Delete an image from storage
 */
export async function deleteStorageImage(
  supabase: SupabaseClient,
  urlOrPath: string | null
): Promise<boolean> {
  const path = extractStoragePath(urlOrPath);
  
  if (!path) {
    return false;
  }

  try {
    // Remove the bucket name from the path if present
    const cleanPath = path.replace('yolo_images/', '');
    
    const { error } = await supabase.storage
      .from('yolo_images')
      .remove([cleanPath]);

    if (error) {
      console.error('Error deleting image from storage:', error);
      return false;
    }

    return true;
  } catch (e) {
    console.error('Error deleting storage image:', e);
    return false;
  }
}

/**
 * Delete multiple images from storage
 */
export async function deleteStorageImages(
  supabase: SupabaseClient,
  urlsOrPaths: (string | null)[]
): Promise<boolean[]> {
  const promises = urlsOrPaths.map(url => deleteStorageImage(supabase, url));
  return Promise.all(promises);
}