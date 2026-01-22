import * as React from 'react';

/**
 * Campo de Input usado na tela de login.
 */
export default function LoginInput(
  props: Readonly<{
    label?: string;
    name?: string;
    type?: string;
    value?: string;
    placeholder?: string;
    customClasses?: string;
    autoComplete?: string;
    onChange?: (e: React.ChangeEvent<HTMLInputElement>) => void;
  }>
) {
  const {
    label,
    name,
    type = 'text',
    value,
    placeholder,
    customClasses,
    autoComplete,
    onChange,
  } = props;
  
  const classes = `
    w-full px-4 py-3 
    bg-white dark:bg-botbot-darker/50 
    border border-gray-200 dark:border-botbot-purple/30 
    rounded-2xl 
    text-gray-800 dark:text-white 
    placeholder-gray-400 dark:placeholder-gray-500
    focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-purple focus:border-transparent
    transition-all duration-200
    ${customClasses || ''}
  `;

  return (
    <div className="w-full">
      {label && (
        <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-2">
          {label}
        </label>
      )}
      <input
        className={classes}
        name={name}
        type={type}
        placeholder={placeholder}
        value={value}
        autoComplete={autoComplete}
        onChange={onChange}
      />
    </div>
  );
}
