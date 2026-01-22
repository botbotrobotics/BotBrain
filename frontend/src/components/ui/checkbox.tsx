'use client';

import * as React from 'react';

export default function Checkbox(
  props: Readonly<{
    label?: string;
    id?: string;
    checked: boolean;
    placeholder?: string;
    className?: string;
    txtColor?: string;
    labelClassName?: string;
    onChange?: (e: React.ChangeEvent<HTMLInputElement>) => void;
  }>
) {
  const {
    label,
    id,
    checked,
    placeholder,
    txtColor,
    className,
    labelClassName,
    onChange,
  } = props;
  const checkBoxClasses = `mr-2 bg-white rounded-full font-semibold text-lg hover:bg-gray-200 ${
    className || ''
  }`;
  const labelClasses =
    'font-light ' +
    (txtColor ? txtColor : 'text-primary') +
    ' ' +
    (labelClassName || '');

  return (
    <div className="w-full flex flex-row items-center justify-end">
      <input
        className={checkBoxClasses}
        id={id}
        type="checkbox"
        placeholder={placeholder}
        onChange={onChange}
        checked={checked}
      />
      {label && (
        <label htmlFor={id} className={labelClasses}>
          {label}
        </label>
      )}
    </div>
  );
}
