import React from 'react';

export default function SeparatorLine({ color, className }: { color?: string; className?: string }) {
  const classColor = color || 'bg-gray-300 dark:bg-black';
  const lineClasses = `w-full h-px ${classColor} ${className || ''}`;
  return <div className={lineClasses} />;
}
