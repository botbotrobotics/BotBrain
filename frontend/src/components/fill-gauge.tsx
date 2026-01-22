'use client';

import clamp from '@/utils/clamp';
import React from 'react';

interface RadialGaugeProps {
  fillColor?: string; // Cor sólida (caso não tenha gradiente)
  gradientColors?: string[]; // Array de cores para o gradiente dinâmico
  backgroundColor?: string; // Cor de fundo
  label?: string; // Rótulo customizável
  valueProp?: number; // Valor externo vindo de um server component
  minValue?: number; // Valor mínimo personalizado
  maxValue?: number; // Valor máximo personalizado
  showPercentage?: boolean; // Se true, exibe a porcentagem no label
  showDecimal?: boolean; // Se true, exibe até duas casas decimais
  unit?: string; // Unidade de medida (ex: "°C", "m", "km/h")
}

// Função para formatar números
const formatNumber = (num: number, showDecimal: boolean = false) => {
  if (showDecimal) {
    // Format with 2 decimal places
    return num.toFixed(2);
  } else {
    // Original behavior: show 1 decimal place if needed, remove if .0
    return parseFloat(num.toFixed(1)).toString().replace(/\.0$/, '');
  }
};

const RadialGauge: React.FC<RadialGaugeProps> = ({
  fillColor = '#821db7',
  gradientColors = [],
  backgroundColor = '#C88DEA',
  label = 'Radial Gauge',
  valueProp,
  minValue = 0,
  maxValue = 100,
  showPercentage = false,
  showDecimal = false,
  unit = '',
}) => {
  const clampedValue = clamp(valueProp ?? minValue, minValue, maxValue);
  const value = clampedValue;

  const strokeWidth = 12;
  const centerX = 100;
  const centerY = 90;
  const angle = ((value - minValue) / (maxValue - minValue)) * 180 - 90;

  // Cálculo da porcentagem sempre que showPercentage for true
  const _percentage = showPercentage
    ? formatNumber(((value - minValue) / (maxValue - minValue)) * 100, showDecimal)
    : null;

  // Geração dinâmica do gradiente se houver cores
  const hasGradient = gradientColors.length > 1;
  const gradientStops = hasGradient
    ? gradientColors.map((color, index) => (
        <stop
          key={index}
          offset={`${(index / (gradientColors.length - 1)) * 100}%`}
          stopColor={color}
        />
      ))
    : null;

  // Format min and max values
  const formattedMinValue = showDecimal ? minValue.toFixed(2) : Math.round(minValue).toString();
  const formattedMaxValue = showDecimal ? maxValue.toFixed(2) : Math.round(maxValue).toString();

  return (
    <div className="flex flex-col items-center justify-center w-full h-full">
      <h2 className="text-sm sm:text-base text-center font-bold mb-1">
        {label} {unit ? `(${unit})` : ''}
      </h2>
      <div className="w-full flex items-center justify-center">
        <svg
          viewBox="0 0 200 140"
          preserveAspectRatio="xMidYMid meet"
          className="w-full max-w-[180px] h-auto"
        >
          {/* Definição do Gradiente Dinâmico */}
          {hasGradient && (
            <defs>
              <linearGradient
                id="gaugeGradient"
                x1="0%"
                y1="0%"
                x2="100%"
                y2="0%"
              >
                {gradientStops}
              </linearGradient>
            </defs>
          )}

          {/* Barra de Fundo */}
          <path
            d="M 20 90 A 80 80 0 1 1 180 90"
            stroke={backgroundColor}
            strokeWidth={strokeWidth}
            fill="none"
            strokeLinecap="butt"
          />
          {/* Barra de Preenchimento (Gradiente Dinâmico ou Cor Sólida) */}
          <path
            d="M 20 90 A 80 80 0 1 1 180 90"
            stroke={hasGradient ? 'url(#gaugeGradient)' : fillColor}
            strokeWidth={strokeWidth}
            fill="none"
            strokeLinecap="butt"
            strokeDasharray="251.2"
            strokeDashoffset={
              251.2 - (251.2 * (value - minValue)) / (maxValue - minValue)
            }
            className="transition-all duration-500 ease-out"
          />
          {/* Ponteiro */}
          <g transform={`rotate(${angle}, ${centerX}, ${centerY})`}>
            <polygon points="100,25 95,90 105,90" fill="black" />
            <circle cx="100" cy="90" r="4" fill="black" />
          </g>
          {/* Números das Extremidades */}
          <text
            x="20"
            y="115"
            fill="black"
            fontSize="10"
            fontWeight="bold"
            textAnchor="middle"
          >
            {`${formattedMinValue}${unit ? `${unit}` : ''}`}
          </text>
          {/* Valor central */}
          <text
            x="100"
            y="115"
            fill="black"
            fontSize="12"
            fontWeight="bold"
            textAnchor="middle"
          >
            {`${formatNumber(value, showDecimal)}${unit}`}
          </text>
          <text
            x="180"
            y="115"
            fill="black"
            fontSize="10"
            fontWeight="bold"
            textAnchor="middle"
          >
            {`${formattedMaxValue}${unit ? `${unit}` : ''}`}
          </text>
        </svg>
      </div>
    </div>
  );
};

export default RadialGauge;
