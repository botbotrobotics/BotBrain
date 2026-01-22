import { createClient } from '@/utils/supabase/client';
import { Widget } from '@/contexts/DashboardContext';

export interface DashboardLayout {
  id: string;
  user_id: string;
  name: string;
  description?: string;
  layout_data: Widget[];
  is_public: boolean;
  is_favorite?: boolean;
  created_at: string;
  updated_at: string;
}

export interface CreateLayoutInput {
  name: string;
  description?: string;
  layout_data: Widget[];
  is_public?: boolean;
}

export interface UpdateLayoutInput {
  name?: string;
  description?: string;
  layout_data?: Widget[];
  is_public?: boolean;
}

class SupabaseDashboardService {
  private supabase = createClient();

  async getUserLayouts(): Promise<DashboardLayout[]> {
    const { data: { user } } = await this.supabase.auth.getUser();
    
    if (!user) {
      throw new Error('User not authenticated');
    }

    const { data, error } = await this.supabase
      .from('dashboard_layouts')
      .select('*')
      .eq('user_id', user.id)
      .order('created_at', { ascending: false });

    if (error) {
      console.error('Error fetching layouts:', error);
      throw error;
    }

    return data || [];
  }

  async getLayout(layoutId: string): Promise<DashboardLayout | null> {
    const { data, error } = await this.supabase
      .from('dashboard_layouts')
      .select('*')
      .eq('id', layoutId)
      .single();

    if (error) {
      console.error('Error fetching layout:', error);
      throw error;
    }

    return data;
  }

  async createLayout(input: CreateLayoutInput): Promise<DashboardLayout> {
    const { data: { user } } = await this.supabase.auth.getUser();
    
    if (!user) {
      throw new Error('User not authenticated');
    }

    const { data, error } = await this.supabase
      .from('dashboard_layouts')
      .insert({
        user_id: user.id,
        name: input.name,
        description: input.description,
        layout_data: input.layout_data,
        is_public: input.is_public || false,
      })
      .select()
      .single();

    if (error) {
      console.error('Error creating layout:', error);
      throw error;
    }

    return data;
  }

  async updateLayout(layoutId: string, input: UpdateLayoutInput): Promise<DashboardLayout> {
    const { data, error } = await this.supabase
      .from('dashboard_layouts')
      .update({
        ...input,
        updated_at: new Date().toISOString(),
      })
      .eq('id', layoutId)
      .select()
      .single();

    if (error) {
      console.error('Error updating layout:', error);
      throw error;
    }

    return data;
  }

  async deleteLayout(layoutId: string): Promise<void> {
    const { error } = await this.supabase
      .from('dashboard_layouts')
      .delete()
      .eq('id', layoutId);

    if (error) {
      console.error('Error deleting layout:', error);
      throw error;
    }
  }

  async getPublicLayouts(): Promise<DashboardLayout[]> {
    const { data, error } = await this.supabase
      .from('dashboard_layouts')
      .select('*')
      .eq('is_public', true)
      .order('created_at', { ascending: false });

    if (error) {
      console.error('Error fetching public layouts:', error);
      throw error;
    }

    return data || [];
  }

  async toggleFavorite(layoutId: string): Promise<void> {
    const { data: { user } } = await this.supabase.auth.getUser();

    if (!user) {
      throw new Error('User not authenticated');
    }

    // Get current state
    const { data: currentLayout } = await this.supabase
      .from('dashboard_layouts')
      .select('is_favorite')
      .eq('id', layoutId)
      .single();

    const newFavoriteState = !currentLayout?.is_favorite;

    const { error } = await this.supabase
      .from('dashboard_layouts')
      .update({ is_favorite: newFavoriteState })
      .eq('id', layoutId);

    if (error) {
      console.error('Error toggling favorite:', error);
      throw error;
    }
  }

  async getFavoriteLayout(): Promise<DashboardLayout | null> {
    const { data: { user } } = await this.supabase.auth.getUser();

    if (!user) {
      return null;
    }

    const { data, error } = await this.supabase
      .from('dashboard_layouts')
      .select('*')
      .eq('user_id', user.id)
      .eq('is_favorite', true)
      .single();

    if (error && error.code !== 'PGRST116') { // PGRST116 = no rows returned
      console.error('Error fetching favorite layout:', error);
      return null;
    }

    return data;
  }
}

export const dashboardService = new SupabaseDashboardService();