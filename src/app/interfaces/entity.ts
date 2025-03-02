export interface BaseEntity {
  id: string;
  type: 'point' | 'polyline' | 'polygon';
}

export interface PointEntity extends BaseEntity {
  type: 'point';
  longitude: number;
  latitude: number;
  pixelSize?: number;
  color?: string;
  label?: string;
}

export interface PolylineEntity extends BaseEntity {
  type: 'polyline';
  positions: { longitude: number; latitude: number }[];
  width?: number;
  color?: string;
}

export interface PolygonEntity extends BaseEntity {
  type: 'polygon';
  positions: { longitude: number; latitude: number }[];
  fillColor?: string;
  outlineColor?: string;
}

export type CesiumEntity = PointEntity | PolylineEntity | PolygonEntity;
