import { AfterViewInit, Component, OnDestroy, OnInit } from '@angular/core';
import {
  Viewer,
  Cartesian3,
  Cartographic,
  Ellipsoid,
  CatmullRomSpline,
  Math as CesiumMath,
  Color,
  VerticalOrigin,
  ScreenSpaceEventHandler,
  ScreenSpaceEventType, Cartesian2, ColorMaterialProperty
} from 'cesium';
import { CesiumService } from '../core/cesium.service';
import { DataService } from '../core/data-service.service';
import { EntityService } from '../core/entity.service';
import {CdkDrag, CdkDragHandle} from '@angular/cdk/drag-drop';
import {DecimalPipe, NgForOf, NgIf} from '@angular/common';
import * as cesium from 'cesium';

// Simple UUID generator (for demonstration)
function generateUUID(): string {
  return Math.random().toString(36).substring(2, 10);
}

interface SavedPolyline {
  uuid: string;
  points: Cartesian3[];
  polylineEntity: any;
  wallEntity: any;
  originalColor: ColorMaterialProperty;
}

@Component({
  selector: 'app-map',
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.css'],
  imports: [
    CdkDrag,
    CdkDragHandle,
    DecimalPipe,
    NgForOf,
    NgIf
  ],
  standalone: true
})
export class MapComponent implements OnInit, AfterViewInit, OnDestroy {
  protected viewer!: Viewer;

  points: Cartesian3[] = [];
  polylineEntity: any = null;
  wallEntity: any = null;
  markers: any[] = [];

  savedPolylines: Map<string, SavedPolyline> = new Map();

  drawingActive = true;
  editingMode = false;
  selectedUuid: string | null = null;
  popupVisible = true;

  defaultLineColor = new ColorMaterialProperty(new Color(0, 1, 0, 1));
  selectedLineColor = new ColorMaterialProperty(new Color(1, 0, 0, 1));
  wallColor = new ColorMaterialProperty(new Color(0, 1, 1, 0.5));

  dragging = false;
  dragIndex = -1;

  // Cesium event handler
  handler!: ScreenSpaceEventHandler;

  constructor(
    public cesium: CesiumService,
    private dataService: DataService
  ) {}

  ngOnInit(): void {
    this.cesium.initScene('cesiumContainer');
    this.viewer = this.cesium.viewer;
  }

  ngAfterViewInit(): void {
    this.handler = new ScreenSpaceEventHandler(this.viewer.canvas);

    // LEFT_DOWN: If a marker is clicked, start dragging.
    this.handler.setInputAction((movement: any) => {
      const picked = this.viewer.scene.pick(movement.position);
      if (picked && picked.id && (picked.id as any).isControlPoint === true) {
        this.dragging = true;
        this.dragIndex = (picked.id as any).markerIndex;
      }
    }, ScreenSpaceEventType.LEFT_DOWN);

    // MOUSE_MOVE: If dragging, update the corresponding point.
    this.handler.setInputAction((movement: any) => {
      if (this.dragging && this.dragIndex >= 0) {
        const newPos = this.viewer.camera.pickEllipsoid(movement.endPosition, Ellipsoid.WGS84);
        if (newPos) {
          const currentCarto = Cartographic.fromCartesian(this.points[this.dragIndex]);
          let newCarto = Cartographic.fromCartesian(newPos);
          newCarto.height = currentCarto.height; // preserve height
          const updatedPoint = Cartesian3.fromRadians(
            newCarto.longitude,
            newCarto.latitude,
            newCarto.height
          );
          this.points[this.dragIndex] = updatedPoint;
          this.markers[this.dragIndex].position = updatedPoint;
          this.updateSpline();
        }
      }
    }, ScreenSpaceEventType.MOUSE_MOVE);

    // LEFT_UP: End dragging.
    this.handler.setInputAction(() => {
      if (this.dragging) {
        this.dragging = false;
        this.dragIndex = -1;
      }
    }, ScreenSpaceEventType.LEFT_UP);

    this.handler.setInputAction((movement: any) => {
      if (this.dragging) { return; }

      if (this.drawingActive || this.editingMode) {
        this.addPoint(movement.position);
        return;
      }

      const picked = this.viewer.scene.pick(movement.position);
      console.log(picked);
      if (picked && picked.id) {
        let uuid: string | undefined;
        if (picked.id.polyline && (picked.id as any).uuid) {
          uuid = (picked.id as any).uuid;
        } else if (picked.id.wall && picked.id.name) {
          uuid = picked.id.name;
        }
        if (uuid) {
          const savedState = this.savedPolylines.get(uuid);
          if (savedState) {
            this.deselectCurrentEntity();
            this.selectedUuid = uuid;
            this.editingMode = true;
            this.drawingActive = false;
            this.points = savedState.points.slice();
            this.polylineEntity = savedState.polylineEntity;
            this.wallEntity = savedState.wallEntity;
            if (this.polylineEntity) {
              this.polylineEntity.polyline.material = this.selectedLineColor;
            }
            this.updateControlPointMarkers();
            return;
          }
        }
      }
      // If nothing picked, start a new line.
      this.deselectCurrentEntity();
      this.startNewLine();
      this.addPoint(movement.position);
    }, ScreenSpaceEventType.LEFT_CLICK);

    // RIGHT_CLICK: Finalize active line (if drawing) or save/deselect in editing.
    this.handler.setInputAction((movement: any) => {
      if (this.editingMode && this.selectedUuid) {
        this.saveChanges();
        this.editingMode = false;
      } else if (this.drawingActive && this.points.length >= 2) {
        this.finalizeLine();
      } else if (this.selectedUuid) {
        this.deselectCurrentEntity();
        this.startNewLine();
      }
    }, ScreenSpaceEventType.RIGHT_CLICK);
  }

  ngOnDestroy(): void {
    if (this.viewer) {
      this.viewer.destroy();
    }
  }

  // Add a new point at the given screen position.
  addPoint(screenPos: { x: number; y: number }): void {
    console.log('adding point', screenPos);
    let pos;
    if (screenPos instanceof Cartesian2) {
      pos = this.viewer.camera.pickEllipsoid(screenPos, Ellipsoid.WGS84);
    }
    if (pos) {
      let defaultHeight = 0; // 1000 ft in meters
      if (this.points.length >= 2) {
        var cartographic = cesium.Cartographic.fromCartesian(this.points[this.points.length-1]);
        defaultHeight = cartographic.height;
      }

      console.log(defaultHeight)
      let carto = Cartographic.fromCartesian(pos);
      carto.height = defaultHeight;
      const newPoint = Cartesian3.fromRadians(carto.longitude, carto.latitude, carto.height);
      this.points.push(newPoint);
      this.updateSpline();
      this.updateControlPointMarkers();
      // Optionally send data to server.
      const lat = CesiumMath.toDegrees(carto.latitude);
      const lon = CesiumMath.toDegrees(carto.longitude);
      this.dataService.postData({ message: `${lat},${lon}` }).subscribe();
    }
  }

  // Recompute the spline and update the polyline and wall.
// updateSpline() creates or updates the active polyline/wall entities.
// It does not remove previously finalized lines.
  updateSpline(): void {
    if (this.points.length < 2) { return; }

    const times = this.points.map((_, i) => i);
    const spline = new CatmullRomSpline({ points: this.points, times });
    const sampleCount = 100;
    const sampledPositions: Cartesian3[] = [];
    for (let i = 0; i <= sampleCount; i++) {
      const t = (this.points.length - 1) * (i / sampleCount);
      sampledPositions.push(spline.evaluate(t));
    }

    // If an active polyline exists, update it; otherwise, create a new one.
    if (this.polylineEntity) {
      this.polylineEntity.polyline.positions = sampledPositions;
      (this.polylineEntity as any).myPoints = this.points.slice();
    } else {
      this.polylineEntity = this.viewer.entities.add({
        polyline: {
          positions: sampledPositions,
          width: 5,
          material: this.defaultLineColor // this.editingMode ? this.selectedLineColor : this.defaultLineColor
        }
      });
      (this.polylineEntity as any).myPoints = this.points.slice();
    }

    const minHeights = sampledPositions.map(() => 0);
    const maxHeights = sampledPositions.map(pos => {
      const carto = Cartographic.fromCartesian(pos);
      return carto.height;
    });
    const wallMaterial = new ColorMaterialProperty(new Color(0, 1, 1, 0.5));
    if (this.wallEntity) {
      this.wallEntity.wall.positions = sampledPositions;
      this.wallEntity.wall.minimumHeights = minHeights;
      this.wallEntity.wall.maximumHeights = maxHeights;
    } else {
      this.wallEntity = this.viewer.entities.add({
        wall: {
          positions: sampledPositions,
          minimumHeights: minHeights,
          maximumHeights: maxHeights,
          material: this.wallColor
        }
      });
      (this.wallEntity as any).name = this.polylineEntity ? (this.polylineEntity as any).uuid : '';
    }
  }

  updateControlPointMarkers(): void {
    for (let i = 0; i < this.points.length; i++) {
      if (this.markers[i]) {
        this.markers[i].position = this.points[i];
      } else {
        const marker = this.viewer.entities.add({
          position: this.points[i],
          billboard: {
            image: 'marker.png', // update path as needed
            scale: 0.5,
            verticalOrigin: VerticalOrigin.BOTTOM
          }
        });
        (marker as any).isControlPoint = true;
        (marker as any).markerIndex = i;
        this.markers.push(marker);
      }
    }
    while (this.markers.length > this.points.length) {
      const m = this.markers.pop();
      if (m) { this.viewer.entities.remove(m); }
    }
  }

  finalizeLine(): void {
    if (this.points.length < 2) { return; }
    const uuid = generateUUID();
    if (this.polylineEntity) {
      (this.polylineEntity as any).uuid = uuid;
      (this.polylineEntity as any).myPoints = this.points.slice();
    }
    if (this.wallEntity) {
      (this.wallEntity as any).name = uuid;
    }
    const saved: SavedPolyline = {
      uuid: uuid,
      points: this.points.slice(),
      polylineEntity: this.polylineEntity,
      wallEntity: this.wallEntity,
      originalColor: this.defaultLineColor
    };

    this.savedPolylines.set(uuid, saved);
    this.drawingActive = false;
    this.editingMode = false;
    //this.startNewLine();
  }

  startNewLine(): void {
    this.points = [];
    this.markers.forEach(m => this.viewer.entities.remove(m));
    this.markers = [];
    this.drawingActive = true;
    this.editingMode = false;
    this.selectedUuid = null;
    this.polylineEntity = null;
    this.wallEntity = null;
  }


  deselectCurrentEntity(): void {
    if (this.selectedUuid) {
      const savedState = this.savedPolylines.get(this.selectedUuid);
      if (savedState && savedState.polylineEntity) {
        savedState.polylineEntity.polyline.material = this.defaultLineColor;
      }
      this.selectedUuid = null;
      this.editingMode = false;
    }
  }

  saveChanges(): void {
    if (!this.selectedUuid) { return; }
    const savedState = this.savedPolylines.get(this.selectedUuid);
    if (savedState) {
      savedState.points = this.points.slice();
      if (this.polylineEntity) {
        this.polylineEntity.polyline.material = this.defaultLineColor;
      }
    }
  }

  promptUpdateHeight(index: number): void {
    const currentHeightMeters = this.getHeight(this.points[index]);
    const currentHeightFeet = (currentHeightMeters / 0.3048).toFixed(0);
    const newHeightStr = prompt('Enter new height in feet (FL):', currentHeightFeet);
    if (newHeightStr !== null) {
      const newHeightFeet = parseFloat(newHeightStr);
      if (!isNaN(newHeightFeet)) {
        const newHeightMeters = newHeightFeet * 0.3048;
        this.updatePointHeight(index, newHeightMeters);
      }
    }
  }

  updatePointHeight(index: number, newHeightMeters: number): void {
    let carto = Cartographic.fromCartesian(this.points[index]);
    carto.height = newHeightMeters;
    const updatedPoint = Cartesian3.fromRadians(carto.longitude, carto.latitude, carto.height);
    this.points[index] = updatedPoint;
    this.updateSpline();
    this.updateControlPointMarkers();
    if (this.editingMode && this.selectedUuid) {
      const savedState = this.savedPolylines.get(this.selectedUuid);
      if (savedState) { savedState.points = this.points.slice(); }
    }
  }

  deletePoint(index: number): void {
    this.points.splice(index, 1);
    if (this.points.length < 2) {
      if (this.polylineEntity) { this.viewer.entities.remove(this.polylineEntity); this.polylineEntity = null; }
      if (this.wallEntity) { this.viewer.entities.remove(this.wallEntity); this.wallEntity = null; }
      if (this.editingMode && this.selectedUuid) {
        this.savedPolylines.delete(this.selectedUuid);
        this.startNewLine();
      }
    } else {
      this.updateSpline();
      this.updateControlPointMarkers();
      if (this.editingMode && this.selectedUuid) {
        const savedState = this.savedPolylines.get(this.selectedUuid);
        if (savedState) { savedState.points = this.points.slice(); }
      }
    }
  }

  getLatitude(point: Cartesian3): number {
    const carto = Cartographic.fromCartesian(point);
    return CesiumMath.toDegrees(carto.latitude);
  }
  getLongitude(point: Cartesian3): number {
    const carto = Cartographic.fromCartesian(point);
    return CesiumMath.toDegrees(carto.longitude);
  }
  getHeight(point: Cartesian3): number {
    const carto = Cartographic.fromCartesian(point);
    return carto.height;
  }

  // Toggle edit mode from UI.
  toggleEditMode(): void {
    if (this.selectedUuid) {
      if (this.editingMode) {
        this.saveChanges();
        this.editingMode = false;
      } else {
        this.editingMode = true;
      }
    }
  }

  togglePopup(): void {
    this.popupVisible = !this.popupVisible;
  }
}
