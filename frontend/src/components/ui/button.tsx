import * as React from 'react';
import Image from 'next/image';

const classMap: { [key: string]: string } = {
  default:
    'outline outline-1 outline-border shadow-[0_4px_0px_#230633] mt-3 w-full py-3 text-white bg-primary rounded-full font-semibold text-lg hover:bg-focus',
  white:
    'outline outline-1 outline-gray-500 shadow-[0_4px_0px_gray] mt-3 w-full py-3 text-gray-600 bg-white rounded-full font-semibold text-lg hover:bg-gray-200',
  'google-auth':
    'outline outline-1 flex items-center justify-center mt-2 w-full py-3 bg-white border rounded-full shadow-md hover:bg-gray-200',
  'facebook-auth':
    'outline outline-1 outline-[#3b5998] flex items-center justify-center mt-2 w-full py-3 bg-white text-[#3b5998] font-bold rounded-full shadow-md hover:bg-gray-200',
  'apple-auth':
    'outline outline-1 flex items-center justify-center mt-2 w-full py-3 bg-black text-white rounded-full shadow-md hover:bg-gray-900',
  'text-only': 'flex items-center justify-center w-full',
  warning:
    'outline outline-1 flex items-center justify-center mt-2 w-full py-3 bg-red-500 text-white rounded-xl shadow-md hover:bg-red-600 hover:scale-105 transition-transform focus:bg-red-700 focus:scale-100',
};

const labelMap: { [key: string]: string } = {
  'google-auth': 'Sign in with Google',
  'facebook-auth': 'Login with Facebook',
  'apple-auth': 'Sign in with Apple',
};

export default function Button(
  props: Readonly<{
    label?: string;
    type?: 'submit' | 'button' | 'reset';
    colorPalette?: string;
    customClasses?: string;
    onClick?: () => void;
  }>
) {
  const {
    colorPalette = 'default',
    label,
    type = 'button',
    customClasses,
    onClick,
  } = props;
  const classes =
    (classMap[colorPalette] || classMap.default) +
    (customClasses ? ` ${customClasses}` : '');
  const buttonLabel = label || labelMap[colorPalette] || '';

  return (
    <button
      type={type}
      className={classes}
      onClick={() => onClick && onClick()}
    >
      {colorPalette.includes('auth') && (
        <Image
          src={`/${colorPalette.split('-')[0]}-icon.png`}
          alt={colorPalette.split('-')[0]}
          width={24}
          height={24}
          className="mr-2"
          style={{ width: 'auto', height: 'auto' }}
        />
      )}
      {buttonLabel}
    </button>
  );
}
