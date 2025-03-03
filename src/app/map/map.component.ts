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
  ScreenSpaceEventType, Cartesian2
} from 'cesium';
import { CesiumService } from '../core/cesium.service';

@Component({
  selector: 'app-map',
  template: `
    <div id="cesiumContainer" style="width: 100%; height: 100vh;"></div>
    <div style="position: absolute; top: 10px; left: 10px; background: #fff; padding: 8px;">
      <p>Left‑click to add a control point.</p>
      <p>Click and drag a marker to reposition a control point.</p>
    </div>
  `,
  styles: [],
  standalone: true
})
export class MapComponent implements OnInit, AfterViewInit, OnDestroy {
  protected viewer!: Viewer;
  // Array of control points (each is a Cartesian3)
  points: Cartesian3[] = [];
  // The polyline entity that displays the computed spline.
  polylineEntity: any;
  // Marker entities for each control point.
  markers: any[] = [];
  // Variables to track dragging of a marker.
  dragging = false;
  dragIndex = -1;
  // Screen space event handler.
  handler!: ScreenSpaceEventHandler;

  constructor(private cesium: CesiumService) {}

  ngOnInit(): void {
    // Initialize the Cesium scene via your service.
    this.cesium.initScene('cesiumContainer');
    this.viewer = this.cesium.viewer;
  }

  ngAfterViewInit(): void {
    this.handler = new ScreenSpaceEventHandler(this.viewer.canvas);

    // LEFT_CLICK: Add a new control point (if not dragging)
    this.handler.setInputAction((movement: any) => {
      if (this.dragging) { return; }
      this.addControlPoint(movement.position);
    }, ScreenSpaceEventType.LEFT_CLICK);

    // LEFT_DOWN: Check if a control point marker is clicked to start dragging.
    this.handler.setInputAction((movement: any) => {
      const picked = this.viewer.scene.pick(movement.position);
      if (picked && picked.id && (picked.id as any).isControlPoint === true) {
        this.dragging = true;
        this.dragIndex = (picked.id as any).markerIndex;
      }
    }, ScreenSpaceEventType.LEFT_DOWN);

    // MOUSE_MOVE: If dragging, update the corresponding control point and marker in real time.
    this.handler.setInputAction((movement: any) => {
      if (this.dragging && this.dragIndex >= 0) {
        const newPos = this.viewer.camera.pickEllipsoid(movement.endPosition, Ellipsoid.WGS84);
        if (newPos) {
          // Preserve the current height.
          const currentCarto = Cartographic.fromCartesian(this.points[this.dragIndex]);
          const newCarto = Cartographic.fromCartesian(newPos);
          newCarto.height = currentCarto.height;
          const updatedPoint = Cartesian3.fromRadians(
            newCarto.longitude,
            newCarto.latitude,
            newCarto.height
          );
          // Update control point and marker.
          this.points[this.dragIndex] = updatedPoint;
          this.markers[this.dragIndex].position = updatedPoint;
          // Update the polyline.
          this.updatePolyline();
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
  }

  ngOnDestroy(): void {
    if (this.viewer) {
      this.viewer.destroy();
    }
  }

  // Adds a control point at the given screen position.
  addControlPoint(screenPos: { x: number; y: number }): void {
    if (screenPos instanceof Cartesian2) {
      const pickedPos = this.viewer.camera.pickEllipsoid(screenPos, Ellipsoid.WGS84);
    if (pickedPos) {
      // Use a default height (e.g., 1000 feet converted to meters)
      const defaultHeightMeters = 1000 * 0.3048;
      let carto = Cartographic.fromCartesian(pickedPos);
      carto.height = defaultHeightMeters;
      const point = Cartesian3.fromRadians(carto.longitude, carto.latitude, carto.height);
      this.points.push(point);
      // Recalculate polyline and update markers.
      this.updatePolyline();
      this.updateControlPointMarkers();
    }
    }
  }

  // Compute a smooth polyline (using Catmull-Rom spline) and update the entity.
  updatePolyline(): void {
    if (this.points.length < 2) return;
    const times = this.points.map((_, i) => i);
    const spline = new CatmullRomSpline({ points: this.points, times });
    const sampleCount = 100;
    const sampledPositions: Cartesian3[] = [];
    for (let i = 0; i <= sampleCount; i++) {
      const t = (this.points.length - 1) * (i / sampleCount);
      sampledPositions.push(spline.evaluate(t));
    }
    if (this.polylineEntity) {
      this.polylineEntity.polyline.positions = sampledPositions;
    } else {
      this.polylineEntity = this.viewer.entities.add({
        polyline: {
          positions: sampledPositions,
          width: 5,
          material: Color.YELLOW
        }
      });
    }
  }

  // Create or update billboard markers for each control point.
  updateControlPointMarkers(): void {
    // Loop through control points.
    for (let i = 0; i < this.points.length; i++) {
      if (this.markers[i]) {
        // Update marker position.
        this.markers[i].position = this.points[i];
      } else {
        // Create a new marker.
        const marker = this.viewer.entities.add({
          position: this.points[i],
          billboard: {
            image: 'assets/marker.png', // Update with your marker image path.
            scale: 0.5,
            verticalOrigin: VerticalOrigin.BOTTOM
          }
        });
        // Set custom properties after creation.
        (marker as any).isControlPoint = true;
        (marker as any).markerIndex = i;
        this.markers.push(marker);
      }
    }
    // Remove extra markers if points were removed.
    while (this.markers.length > this.points.length) {
      const marker = this.markers.pop();
      if (marker) {
        this.viewer.entities.remove(marker);
      }
    }
  }
}
