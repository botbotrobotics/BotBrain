-- Run this script in your Supabase SQL Editor to set up storage policies for profile pictures

-- Policy to allow authenticated users to upload their own profile pictures
CREATE POLICY "Users can upload own profile pictures"
ON storage.objects FOR INSERT
TO authenticated
WITH CHECK (
    bucket_id = 'profile-pictures' AND 
    (storage.foldername(name))[1] = auth.uid()::text
);

-- Policy to allow authenticated users to update their own profile pictures
CREATE POLICY "Users can update own profile pictures"
ON storage.objects FOR UPDATE
TO authenticated
USING (
    bucket_id = 'profile-pictures' AND 
    (storage.foldername(name))[1] = auth.uid()::text
)
WITH CHECK (
    bucket_id = 'profile-pictures' AND 
    (storage.foldername(name))[1] = auth.uid()::text
);

-- Policy to allow authenticated users to delete their own profile pictures
CREATE POLICY "Users can delete own profile pictures"
ON storage.objects FOR DELETE
TO authenticated
USING (
    bucket_id = 'profile-pictures' AND 
    (storage.foldername(name))[1] = auth.uid()::text
);

-- Policy to allow anyone to view profile pictures (since the bucket is public)
CREATE POLICY "Anyone can view profile pictures"
ON storage.objects FOR SELECT
TO public
USING (bucket_id = 'profile-pictures');

-- Verify policies were created
SELECT 
    policyname,
    cmd,
    roles::text
FROM pg_policies
WHERE schemaname = 'storage' 
AND tablename = 'objects'
AND policyname LIKE '%profile pictures%'
ORDER BY policyname; 