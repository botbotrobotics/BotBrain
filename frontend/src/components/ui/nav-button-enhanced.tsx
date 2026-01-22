'use client';

import React from 'react';
import Image from 'next/image';

interface NavButtonEnhancedProps {
  name: string;
  icon: string | React.ComponentType<{ className?: string; size?: number }>;
  alt?: string;
  size?: number;
  layout?: 'desktop' | 'mobile';
  onClick?: () => void;
  isActive?: boolean;
}

export default function NavButtonEnhanced({
  name,
  icon,
  alt = '',
  size = 25,
  layout = 'desktop',
  onClick,
  isActive = false,
}: NavButtonEnhancedProps): React.JSX.Element {
  const divClasses = `${
    layout === 'desktop' ? 'min-w-fit' : 'w-full my-0.5'
  } rounded-lg p-2 hover:bg-purple-100 dark:hover:bg-purple-700
     active:bg-purple-200 dark:active:bg-purple-800 transition-all duration-200`;

  const btnClasses =
    layout === 'desktop'
      ? `flex flex-col items-center justify-center text-xs whitespace-nowrap ${
          isActive
            ? 'text-purple-700 dark:text-purple-400 font-medium'
            : 'text-gray-700 dark:text-gray-200 hover:text-purple-700 dark:hover:text-purple-300'
        }`
      : `w-full flex flex-row items-center justify-start text-sm gap-2 px-2 py-1 ${
          isActive
            ? 'text-purple-700 dark:text-purple-400 font-medium bg-purple-50 dark:bg-purple-900/20 rounded'
            : 'text-gray-700 dark:text-gray-200 hover:text-purple-700 dark:hover:text-purple-300'
        }`;

  const iconClasses = `${layout === 'desktop' ? 'mb-1' : ''} ${
    isActive
      ? 'text-purple-700 dark:text-purple-400'
      : 'text-gray-700 dark:text-gray-200'
  }`;

  const renderIcon = () => {
    if (typeof icon === 'string') {
      // Legacy SVG path support
      const path = `/icons/${icon}.svg`;
      return (
        <Image
          className={layout === 'desktop' ? 'mb-1' : ''}
          src={path}
          alt={alt}
          width={size}
          height={size}
          style={{
            width: `${size}px`,
            height: `${size}px`,
          }}
        />
      );
    } else {
      // React component icon
      const IconComponent = icon;
      return <IconComponent className={iconClasses} size={size} />;
    }
  };

  return (
    <div className={divClasses}>
      <button className={btnClasses} onClick={onClick}>
        {renderIcon()}
        {name}
      </button>
    </div>
  );
}