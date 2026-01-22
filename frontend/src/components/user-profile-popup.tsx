'use client';

import type * as React from 'react';
import { LogOut, User2 } from 'lucide-react';
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from '@/components/ui/dropdown-menu';
import { useLanguage } from '@/contexts/LanguageContext';
import { cn } from '@/utils/cn';
import { useSupabase } from '@/contexts/SupabaseProvider';

interface UserProfilePopupProps {
  children: React.ReactNode;
}

export function UserProfilePopup({ children }: UserProfilePopupProps) {
  const { t } = useLanguage();
  const { user, signOut } = useSupabase();

  const hoverClasses = 'hover:hover:bg-purple-200 dark:hover:bg-purple-800';

  const handleLogout = async () => {
    await signOut();
  };

  return (
    <DropdownMenu>
      <DropdownMenuTrigger asChild>{children}</DropdownMenuTrigger>
      <DropdownMenuContent
        className="w-56 bg-pink-lighter dark:bg-botbot-darker border-gray-300 dark:border-black"
        side="right"
        align="end"
        alignOffset={8}
        sideOffset={8}
        forceMount
      >
        <div className="flex items-center gap-2 p-2">
          <div className="w-10 h-10 rounded-full overflow-hidden bg-muted">
            <User2 className="w-full h-full object-cover" />
          </div>
          <div className="flex flex-col text-black dark:text-botbot-accent">
            <span className="text-sm font-medium">{user?.email || 'Guest'}</span>
          </div>
        </div>
        <DropdownMenuSeparator />
        <DropdownMenuItem
          className={cn(hoverClasses, 'text-red-600 dark:text-red-400')}
          onClick={handleLogout}
        >
          <LogOut className="mr-2 h-4 w-4" />
          <span>{t('userProfile', 'logout')}</span>
        </DropdownMenuItem>
      </DropdownMenuContent>
    </DropdownMenu>
  );
}
