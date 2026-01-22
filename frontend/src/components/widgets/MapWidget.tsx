'use client';

import { Widget } from './Widget';
import MapViewNav2 from '../map-view-nav2';

interface MapWidgetProps {
  id: string;
  initialPosition?: { x: number; y: number };
  initialSize?: { width: number; height: number };
  props?: Record<string, any>;
  onRemove: (id: string) => void;
  onStartDrag?: (id: string) => void;
  onEndDrag?: (id: string) => void;
}

export function MapWidget({
  id,
  initialPosition,
  initialSize = { width: 400, height: 350 },
  onRemove,
  onStartDrag,
  onEndDrag,
}: MapWidgetProps) {
  return (
    <Widget
      id={id}
      title="Map View"
      initialPosition={initialPosition}
      initialSize={initialSize}
      onRemove={onRemove}
      onStartDrag={onStartDrag}
      onEndDrag={onEndDrag}
      minWidth={300}
      minHeight={250}
    >
      <div className="h-full w-full relative -m-4">
        <MapViewNav2 className="absolute inset-0" />
      </div>
    </Widget>
  );
}