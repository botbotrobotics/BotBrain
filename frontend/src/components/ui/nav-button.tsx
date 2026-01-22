'use client';

import React from 'react';
import Image from 'next/image';

interface NavButtonProps {
  name: string;
  icon: string;
  alt?: string;
  size?: number;
  layout?: 'desktop' | 'mobile'; // prop opcional
  onClick?: () => void;
  isActive?: boolean;
}

export default function NavButton({
  name,
  icon,
  alt = '',
  size = 25,
  layout = 'desktop', // valor padrão 'desktop'
  onClick,
  isActive = false,
}: NavButtonProps): React.JSX.Element {
  const path = `/icons/${icon}.svg`;

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

  return (
    <div className={divClasses}>
      <button className={btnClasses} onClick={onClick}>
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
        {name}
      </button>
    </div>
  );
}
