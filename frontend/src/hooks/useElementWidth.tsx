'use client';
import { useState, useEffect } from 'react';

/**
 * Retorna a largura (width) de um elemento desejado.
 * @param containerRef A referência ao elemento.
 * @returns O valor numérico da sua largura.
 */
export default function useElementWidth(
  containerRef: React.RefObject<HTMLElement | null>
): number {
  const [width, setWidth] = useState<number>(0);

  useEffect(() => {
    const checkWidth = () => {
      if (containerRef.current) setWidth(containerRef.current.clientWidth);
    };
    const observer = new ResizeObserver(checkWidth);
    if (containerRef.current) observer.observe(containerRef.current);
    checkWidth();
    return () => observer.disconnect();
  }, [containerRef]);
  return width;
}
