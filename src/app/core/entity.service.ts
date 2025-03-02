import { Injectable } from '@angular/core';
import {CesiumEntity} from '../interfaces/entity';
import * as Cesium from 'cesium';

@Injectable({
  providedIn: 'root'
})
export class EntityService {

  spawnEntity(viewer: any, entity: CesiumEntity): Cesium.Entity | undefined {
    let cesiumEntity: Cesium.Entity | undefined;

    switch (entity.type) {
      case 'point':
        cesiumEntity = viewer.entities.add({
          id: entity.id,
          position: Cesium.Cartesian3.fromDegrees(entity.longitude, entity.latitude),
          point: {
            pixelSize: entity.pixelSize || 10,
            color: Cesium.Color.fromCssColorString(entity.color || '#ff0000'),
          },
          label: entity.label ? {
            text: entity.label,
            font: '8pt sans-serif',
            style: Cesium.LabelStyle.FILL_AND_OUTLINE,
            outlineWidth: 2,
            verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
            pixelOffset: new Cesium.Cartesian2(0, -9)
          } : undefined
        });
        break;

      case 'polyline':
        cesiumEntity = viewer.entities.add({
          id: entity.id,
          polyline: {
            positions: entity.positions.map(pos => Cesium.Cartesian3.fromDegrees(pos.longitude, pos.latitude)),
            width: entity.width || 5,
            material: Cesium.Color.fromCssColorString(entity.color || '#0000ff')
          }
        });
        break;

      case 'polygon':
        cesiumEntity = viewer.entities.add({
          id: entity.id,
          polygon: {
            hierarchy: new Cesium.PolygonHierarchy(
              entity.positions.map(pos => Cesium.Cartesian3.fromDegrees(pos.longitude, pos.latitude))
            ),
            material: Cesium.Color.fromCssColorString(entity.fillColor || '#00ff00').withAlpha(0.5),
            outline: true,
            outlineColor: Cesium.Color.fromCssColorString(entity.outlineColor || '#000000')
          }
        });
        break;

      default:
        console.warn('Unknown entity type:', entity);
        break;
    }

    return cesiumEntity;
  }

  hideEntity(viewer: any, entityId: string): void {
    const entity = viewer.entities.getById(entityId);
    if (entity) {
      entity.show = false;
      console.log(`Entity with id ${entityId} is now hidden.`);
    } else {
      console.warn(`Entity with id ${entityId} not found.`);
    }
  }

  showEntity(viewer: any, entityId: string): void {
    const entity = viewer.entities.getById(entityId);
    if (entity) {
      entity.show = true;
    }
  }

  constructor() {
  }
}
