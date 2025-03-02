import {AfterViewInit, Component, OnDestroy, OnInit} from '@angular/core';

import {CesiumService} from '../core/cesium.service';
import {
  Cartographic,
  Math as CesiumMath,
  Ellipsoid,
  ScreenSpaceEventHandler,
  ScreenSpaceEventType,
  Viewer, Entity, ITwinData
} from 'cesium';
import {DataService} from '../core/data-service.service';
import {CesiumEntity, PointEntity} from '../interfaces/entity';
import {EntityService} from '../core/entity.service';

@Component({
  selector: 'app-map',
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.css']
})
export class MapComponent implements OnInit, AfterViewInit, OnDestroy {
  protected viewer: Viewer | any = null;
  private entityService: EntityService = new EntityService();
  constructor(public cesium: CesiumService, private dataService: DataService) {
  }

  fixes_id : string[] = []
  sectors_id : string[] = []

  ngOnInit() {
    this.cesium.initScene("cesiumContainer");
    this.viewer = this.cesium.viewer;
  }

  ngAfterViewInit() {
    const handler = new ScreenSpaceEventHandler(this.viewer.canvas);
    handler.setInputAction((movement: { position: { x: number; y: number } }) => {
      this.closeContextMenu(movement);
      this.sendPosition(movement);
    }, ScreenSpaceEventType.LEFT_CLICK);
    handler.setInputAction((movement: { position: { x: number; y: number } }) => {
      this.openContextMenu(movement);
    }, ScreenSpaceEventType.RIGHT_CLICK);

    this.dataService.getFixes().subscribe(
      data => this.drawFixes(data),
      error => console.error('HTTP error:', error)
    );

    this.dataService.getSectors().subscribe(
      data => this.drawSectors(data),
      error => console.error('HTTP error:', error)
    )
  }

  drawFixes(data: any) {
    console.log('drawing fixes function')

    for (let i = 0; i < data.length; i++) {
      console.log('drawing fix {}', i)
      const e : PointEntity = {
        type: 'point',
        longitude: data[i]['lng'],
        latitude: data[i]['lat'],
        pixelSize: 2,
        label: data[i]['fixlongcode'],
        id: data[i]['fixlongcode'],
        };

      this.fixes_id.push(data[i].fixlongcode);
      this.entityService.spawnEntity(this.viewer, e);
    }
  }

  drawSectors(data: any) {
    console.log(data);
    /*

    for (let i = 0; i < data.length; i++) {
      console.log('drawing fix {}', i)
      const e : PointEntity = {
        type: 'point',
        longitude: data[i]['lng'],
        latitude: data[i]['lat'],
        pixelSize: 2,
        label: data[i]['fixlongcode'],
        id: data[i]['fixlongcode'],
      };

      this.fixes_id.push(data[i].fixlongcode);
      this.entityService.spawnEntity(this.viewer, e);
    }
     */
  }

  ngOnDestroy() {
    if (this.viewer) {
      this.viewer.destroy();
    }
  }

  sendPosition(movement: { position: { x: number; y: number } }) {
    const pickedPosition = this.viewer.camera.pickEllipsoid(movement.position, Ellipsoid.WGS84);
    if (pickedPosition) {
      const cartographic = Cartographic.fromCartesian(pickedPosition);
      const latitude = CesiumMath.toDegrees(cartographic.latitude);
      const longitude = CesiumMath.toDegrees(cartographic.longitude);

      const data = `${latitude},${longitude}`;

      this.dataService.postData({ message: data }).subscribe(
        response => console.log('Server response:', response),
        error => console.error('Error sending data:', error)
      );
    } else {
      console.log('No position picked.');
    }
  }

  openContextMenu(click: {"position": any}) {
    const position = click.position;
    const menu = document.getElementById('contextMenu');

    if (!menu) {
      return;
    }

    menu.style.display = 'block';
    menu.style.left = '0px';
    menu.style.top = '0px';

    const menuWidth = menu.offsetWidth;
    const menuHeight = menu.offsetHeight;
    const windowWidth = window.innerWidth;
    const windowHeight = window.innerHeight;

    let left = position.x;
    let top = position.y;

    if (position.x + menuWidth > windowWidth) {
      left = position.x - menuWidth;
    }

    if (position.y + menuHeight > windowHeight) {
      top = position.y - menuHeight;
    }

    menu.style.left = `${left}px`;
    menu.style.top = `${top}px`;
  }

  closeContextMenu(event: any) {
    const menu = document.getElementById('contextMenu');
    if (menu && !menu.contains(event.target)) {
      menu.style.display = 'none';
    }
  }

  spawnPoint() {
    this.dataService.getData();
  }

  hidePoint() {
    for (let i = 0; i < this.fixes_id.length; i++) {
      this.entityService.hideEntity(this.viewer, this.fixes_id[i]);
    }
  }

  showEntity() {
    for (let i = 0; i < this.fixes_id.length; i++) {
      this.entityService.showEntity(this.viewer, this.fixes_id[i]);
    }
  }
}
