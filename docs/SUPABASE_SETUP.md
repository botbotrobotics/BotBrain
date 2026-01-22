# Supabase Setup Guide

This guide walks you through setting up Supabase for BotBrain. Supabase provides authentication, database storage, and file storage for the web dashboard.

## Table of Contents

- [1. Create a Supabase Project](#1-create-a-supabase-project)
- [2. Get Your API Keys](#2-get-your-api-keys)
- [3. Create Database Tables](#3-create-database-tables)
- [4. Set Up Row Level Security (RLS)](#4-set-up-row-level-security-rls)
- [5. Configure Authentication](#5-configure-authentication)
- [6. Set Up Storage (Optional)](#6-set-up-storage-optional)
- [7. Configure Environment Variables](#7-configure-environment-variables)

---

## 1. Create a Supabase Project

1. Go to [supabase.com](https://supabase.com) and sign up or log in
2. Click **New Project**
3. Fill in the project details:
   - **Name**: `botbrain` (or your preferred name)
   - **Database Password**: Generate a strong password and save it securely
   - **Region**: Choose the closest region to your location
4. Click **Create new project** and wait for setup to complete (~2 minutes)

---

## 2. Get Your API Keys

1. In your Supabase dashboard, go to **Project Settings** (gear icon in sidebar)
2. Click **API** in the left menu
3. Copy these values:
   - **Project URL** (`NEXT_PUBLIC_SUPABASE_URL`)
   - **anon/public key** (`NEXT_PUBLIC_SUPABASE_ANON_KEY`)

---

## 3. Create Database Tables

Go to the **SQL Editor** in your Supabase dashboard and run the following SQL to create all required tables:

### Complete Schema

```sql
-- =============================================
-- BotBrain Database Schema
-- =============================================

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- =============================================
-- Table: robots
-- Stores robot connection configurations
-- =============================================
CREATE TABLE IF NOT EXISTS public.robots (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL REFERENCES auth.users(id) ON DELETE CASCADE,
    name TEXT NOT NULL,
    address TEXT NOT NULL,
    type TEXT DEFAULT 'go2',
    key TEXT,
    is_favorite BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW()
);

-- Index for faster user queries
CREATE INDEX IF NOT EXISTS idx_robots_user_id ON public.robots(user_id);

-- =============================================
-- Table: user_profiles
-- Stores user preferences and settings
-- =============================================
CREATE TABLE IF NOT EXISTS public.user_profiles (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL UNIQUE REFERENCES auth.users(id) ON DELETE CASCADE,
    name TEXT,
    avatar_url TEXT,
    theme_color TEXT DEFAULT 'purple',
    speed_mode TEXT DEFAULT 'normal',
    connection_timeout INTEGER DEFAULT 30000,
    audit_logging_enabled BOOLEAN DEFAULT TRUE,
    hide_branding BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW()
);

-- Index for faster user queries
CREATE INDEX IF NOT EXISTS idx_user_profiles_user_id ON public.user_profiles(user_id);

-- =============================================
-- Table: audit_logs
-- Stores activity logs for tracking and debugging
-- =============================================
CREATE TABLE IF NOT EXISTS public.audit_logs (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL REFERENCES auth.users(id) ON DELETE CASCADE,
    robot_id UUID REFERENCES public.robots(id) ON DELETE SET NULL,
    robot_name TEXT,
    event_type TEXT NOT NULL,
    event_action TEXT NOT NULL,
    event_details JSONB,
    ip_address INET,
    user_agent TEXT,
    created_at TIMESTAMPTZ DEFAULT NOW()
);

-- Indexes for faster queries
CREATE INDEX IF NOT EXISTS idx_audit_logs_user_id ON public.audit_logs(user_id);
CREATE INDEX IF NOT EXISTS idx_audit_logs_created_at ON public.audit_logs(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_audit_logs_event_type ON public.audit_logs(event_type);

-- =============================================
-- Table: missions
-- Stores navigation mission definitions
-- =============================================
CREATE TABLE IF NOT EXISTS public.missions (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL REFERENCES auth.users(id) ON DELETE CASCADE,
    robot_id UUID REFERENCES public.robots(id) ON DELETE SET NULL,
    name TEXT NOT NULL,
    description TEXT,
    map_name TEXT,
    is_active BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW()
);

-- Index for faster user queries
CREATE INDEX IF NOT EXISTS idx_missions_user_id ON public.missions(user_id);

-- =============================================
-- Table: waypoints
-- Stores navigation waypoints for missions
-- =============================================
CREATE TABLE IF NOT EXISTS public.waypoints (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    mission_id UUID NOT NULL REFERENCES public.missions(id) ON DELETE CASCADE,
    order_index INTEGER NOT NULL,
    x DOUBLE PRECISION NOT NULL,
    y DOUBLE PRECISION NOT NULL,
    theta DOUBLE PRECISION NOT NULL DEFAULT 0,
    is_reached BOOLEAN DEFAULT FALSE,
    reached_at TIMESTAMPTZ,
    created_at TIMESTAMPTZ DEFAULT NOW(),
    updated_at TIMESTAMPTZ DEFAULT NOW()
);

-- Index for faster mission queries
CREATE INDEX IF NOT EXISTS idx_waypoints_mission_id ON public.waypoints(mission_id);

-- =============================================
-- Trigger: Auto-update updated_at timestamp
-- =============================================
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Apply trigger to all tables with updated_at column
CREATE TRIGGER update_robots_updated_at
    BEFORE UPDATE ON public.robots
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_profiles_updated_at
    BEFORE UPDATE ON public.user_profiles
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_missions_updated_at
    BEFORE UPDATE ON public.missions
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_waypoints_updated_at
    BEFORE UPDATE ON public.waypoints
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- =============================================
-- Function: Auto-create user profile on signup
-- =============================================
CREATE OR REPLACE FUNCTION public.handle_new_user()
RETURNS TRIGGER AS $$
BEGIN
    INSERT INTO public.user_profiles (user_id, name)
    VALUES (NEW.id, NEW.raw_user_meta_data->>'name');
    RETURN NEW;
END;
$$ LANGUAGE plpgsql SECURITY DEFINER;

-- Trigger to create profile when user signs up
DROP TRIGGER IF EXISTS on_auth_user_created ON auth.users;
CREATE TRIGGER on_auth_user_created
    AFTER INSERT ON auth.users
    FOR EACH ROW EXECUTE FUNCTION public.handle_new_user();
```

---

## 4. Set Up Row Level Security (RLS)

Row Level Security ensures users can only access their own data. Run this SQL:

```sql
-- =============================================
-- Row Level Security (RLS) Policies
-- =============================================

-- Enable RLS on all tables
ALTER TABLE public.robots ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.user_profiles ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.audit_logs ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.missions ENABLE ROW LEVEL SECURITY;
ALTER TABLE public.waypoints ENABLE ROW LEVEL SECURITY;

-- =============================================
-- Robots Policies
-- =============================================
CREATE POLICY "Users can view their own robots"
    ON public.robots FOR SELECT
    USING (auth.uid() = user_id);

CREATE POLICY "Users can insert their own robots"
    ON public.robots FOR INSERT
    WITH CHECK (auth.uid() = user_id);

CREATE POLICY "Users can update their own robots"
    ON public.robots FOR UPDATE
    USING (auth.uid() = user_id);

CREATE POLICY "Users can delete their own robots"
    ON public.robots FOR DELETE
    USING (auth.uid() = user_id);

-- =============================================
-- User Profiles Policies
-- =============================================
CREATE POLICY "Users can view their own profile"
    ON public.user_profiles FOR SELECT
    USING (auth.uid() = user_id);

CREATE POLICY "Users can insert their own profile"
    ON public.user_profiles FOR INSERT
    WITH CHECK (auth.uid() = user_id);

CREATE POLICY "Users can update their own profile"
    ON public.user_profiles FOR UPDATE
    USING (auth.uid() = user_id);

-- =============================================
-- Audit Logs Policies
-- =============================================
CREATE POLICY "Users can view their own audit logs"
    ON public.audit_logs FOR SELECT
    USING (auth.uid() = user_id);

CREATE POLICY "Users can insert their own audit logs"
    ON public.audit_logs FOR INSERT
    WITH CHECK (auth.uid() = user_id);

-- =============================================
-- Missions Policies
-- =============================================
CREATE POLICY "Users can view their own missions"
    ON public.missions FOR SELECT
    USING (auth.uid() = user_id);

CREATE POLICY "Users can insert their own missions"
    ON public.missions FOR INSERT
    WITH CHECK (auth.uid() = user_id);

CREATE POLICY "Users can update their own missions"
    ON public.missions FOR UPDATE
    USING (auth.uid() = user_id);

CREATE POLICY "Users can delete their own missions"
    ON public.missions FOR DELETE
    USING (auth.uid() = user_id);

-- =============================================
-- Waypoints Policies
-- =============================================
CREATE POLICY "Users can view waypoints of their missions"
    ON public.waypoints FOR SELECT
    USING (EXISTS (
        SELECT 1 FROM public.missions
        WHERE missions.id = waypoints.mission_id
        AND missions.user_id = auth.uid()
    ));

CREATE POLICY "Users can insert waypoints to their missions"
    ON public.waypoints FOR INSERT
    WITH CHECK (EXISTS (
        SELECT 1 FROM public.missions
        WHERE missions.id = waypoints.mission_id
        AND missions.user_id = auth.uid()
    ));

CREATE POLICY "Users can update waypoints of their missions"
    ON public.waypoints FOR UPDATE
    USING (EXISTS (
        SELECT 1 FROM public.missions
        WHERE missions.id = waypoints.mission_id
        AND missions.user_id = auth.uid()
    ));

CREATE POLICY "Users can delete waypoints of their missions"
    ON public.waypoints FOR DELETE
    USING (EXISTS (
        SELECT 1 FROM public.missions
        WHERE missions.id = waypoints.mission_id
        AND missions.user_id = auth.uid()
    ));
```

---

## 5. Configure Authentication

1. In your Supabase dashboard, go to **Authentication** > **Providers**
2. Ensure **Email** is enabled (it should be by default)
3. Optionally configure:
   - **Confirm email**: Disable for easier testing, enable for production
   - **Secure email change**: Recommended to enable

### Email Templates (Optional)

Go to **Authentication** > **Email Templates** to customize:
- Confirmation email
- Password reset email
- Magic link email

---

## 6. Set Up Storage (Optional)

If you want to support user avatar uploads:

1. Go to **Storage** in your Supabase dashboard
2. Click **New bucket**
3. Create a bucket named `avatars`
4. Set the bucket to **Public** (for avatar images)
5. Run this SQL to set up storage policies:

```sql
-- =============================================
-- Storage Policies for Avatars
-- =============================================

-- Allow users to upload their own avatars
CREATE POLICY "Users can upload avatars"
    ON storage.objects FOR INSERT
    WITH CHECK (
        bucket_id = 'avatars' AND
        auth.uid()::text = (storage.foldername(name))[1]
    );

-- Allow users to update their own avatars
CREATE POLICY "Users can update their avatars"
    ON storage.objects FOR UPDATE
    USING (
        bucket_id = 'avatars' AND
        auth.uid()::text = (storage.foldername(name))[1]
    );

-- Allow users to delete their own avatars
CREATE POLICY "Users can delete their avatars"
    ON storage.objects FOR DELETE
    USING (
        bucket_id = 'avatars' AND
        auth.uid()::text = (storage.foldername(name))[1]
    );

-- Allow public read access to avatars
CREATE POLICY "Public can view avatars"
    ON storage.objects FOR SELECT
    USING (bucket_id = 'avatars');
```

---

## 7. Configure Environment Variables

1. Navigate to your BotBrain frontend directory:
   ```bash
   cd frontend
   ```

2. Copy the example environment file:
   ```bash
   cp .env.example .env.local
   ```

3. Edit `.env.local` and add your Supabase credentials:
   ```bash
   NEXT_PUBLIC_SUPABASE_URL=https://your-project-id.supabase.co
   NEXT_PUBLIC_SUPABASE_ANON_KEY=your-anon-key-here
   ```

4. Start the development server:
   ```bash
   npm install
   npm run dev
   ```

5. Open [http://localhost:3000](http://localhost:3000) and create your first account!

---

## Database Schema Overview

| Table | Description |
|-------|-------------|
| `robots` | Robot connection configurations (address, name, type) |
| `user_profiles` | User preferences (theme, avatar, settings) |
| `audit_logs` | Activity logging for debugging and compliance |
| `missions` | Navigation mission definitions |
| `waypoints` | Navigation waypoints within missions |

---

## Troubleshooting

### "Permission denied" errors
- Ensure RLS policies are created correctly
- Check that the user is authenticated before accessing data

### User profile not created on signup
- Verify the `handle_new_user` trigger is created
- Check the Supabase logs for any trigger errors

### Can't upload avatars
- Ensure the `avatars` bucket exists and is public
- Verify storage policies are in place

---

## Next Steps

- Return to the [main README](../README.md) for installation instructions
- See the [BotBrain Workspace Guide](../botbrain_ws/README.md) for ROS2 setup
