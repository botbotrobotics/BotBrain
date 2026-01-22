import { Vector3 } from 'roslib';

const DECIMAL_DIGITS = 3;

interface Vector3ToStringOptions {
  useX?: boolean;
  useY?: boolean;
  useZ?: boolean;
}

export function formatDecimal(
  value: number,
  decimalDigits = DECIMAL_DIGITS
): string {
  return parseFloat(value.toFixed(decimalDigits)).toString();
}

export const vector3ToString = (
  vector3: Vector3,
  options: Vector3ToStringOptions = {
    useX: true,
    useY: true,
    useZ: true,
  }
) => {
  let result = '(';

  if (options.useX) {
    result += `x: ${formatDecimal(vector3.x)}`;
    result += options.useY || options.useZ ? ', ' : '';
  }

  if (options.useY) {
    result += `y: ${formatDecimal(vector3.y)}`;
    result += options.useZ ? ', ' : '';
  }

  if (options.useZ) {
    result += `z: ${formatDecimal(vector3.z)}`;
  }

  result += ')';

  return result;
};
