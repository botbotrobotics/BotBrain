import React from 'react';
import Image from 'next/image';

export default function Avatar(
  props: Readonly<{
    imgSrc: string;
    size?: number;
  }>
) {
  const src = `/${props.imgSrc}`;
  const imgSize = props.size || 70;

  return (
    <Image
      src={src}
      alt="Avatar"
      width={imgSize}
      height={imgSize}
      className="rounded-full border-2 border-gray-300 shadow-lg"
      style={{ width: 'auto', height: 'auto' }}
    />
  );
}
