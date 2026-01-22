import { Header } from './Header';

export interface Image {
  header: Header;
  height: number;
  width: number;
  format: string;
  is_bigendian: number;
  step: number;
  data: string;
}
