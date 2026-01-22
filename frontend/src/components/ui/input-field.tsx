import { cn } from '@/utils/cn';
import React from 'react';

/**
 * Campo de Input usado dentro do sistema nos containers e outros componentes.
 */
export default function InputField(
  props: Readonly<{
    label?: string;
    name?: string;
    type?: string;
    value?: string;
    placeholder?: string;
    className?: string;
    disabled?: boolean;
    children?: React.ReactNode;
    onChange?: (e: React.ChangeEvent<HTMLInputElement>) => void;
  }>
) {
  const {
    label,
    name,
    type = 'text',
    value,
    placeholder,
    className,
    disabled,
    children,
    onChange,
  } = props;

  const _inputName = name;

  return (
    <div className="w-full flex flex-col items-center justify-center">
      {label && <label className="font-light mb-2">{label}</label>}
      <div
        className={cn(
          'w-full flex flex-row p-1 items-center justify-between ring-1 ring-gray-300 rounded-full',
          className
        )}
      >
        <input
          // name={_inputName}
          type={type}
          value={value}
          onChange={onChange}
          placeholder={placeholder}
          className="w-10/12 px-2 py-2 rounded-full text-sm outline-none"
          disabled={disabled}
        />

        <>{children}</>
      </div>
    </div>
  );
}
