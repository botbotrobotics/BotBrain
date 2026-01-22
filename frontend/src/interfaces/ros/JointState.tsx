import { Header } from './Header';

export interface JointState {
  header: Header;
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}
