'use client';

import { useState, useEffect } from 'react';

/**
 * Retorna a largura (width) da janela do navegador.
 * @returns O valor numérico da largura da janela.
 */
export default function useWindowWidth(): number {
  const [width, setWidth] = useState<number>(
    typeof window !== 'undefined' ? window.innerWidth : 0
  );

  useEffect(() => {
    const handleResize = () => {
      setWidth(window.innerWidth);
    };

    handleResize();

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  return width;
}
