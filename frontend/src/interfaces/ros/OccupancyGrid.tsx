import { Header } from './Header';
import { MapMetaData } from './MapMetaData';

export interface OccupancyGrid {
  header: Header;
  info: MapMetaData;
  data: number[];
}
