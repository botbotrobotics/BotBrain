import { Header } from './Header';

export interface CompressedImage {
  data: string;
  format: string;
  header: Header;
}
