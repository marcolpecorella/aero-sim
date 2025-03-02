import {Injectable} from '@angular/core';

// @ts-ignore
declare let Cesium: any;
Cesium.Ion.defaultAccessToken = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiIwMGI3MDBhYi01MThkLTQ3NGUtYWFlMy0yOWViYTczZGFmZTEiLCJpZCI6MjU3MzQwLCJpYXQiOjE3MzIzMDAzMjZ9.H7iCxM7NNsclikVYNDuBs_Xe_jf8cFPtJk9gSQJhRLg"
@Injectable({
  providedIn: 'root'
})
export class CesiumService {
  constructor() { }
  viewer: any;
  private scene: any;

  async initScene(div: string) {
    this.viewer = new Cesium.Viewer(div, {
      skyAtmosphere: new Cesium.SkyAtmosphere(),
      sceneModePicker: false,
      baseLayerPicker: false,
      terrain: Cesium.Terrain.fromWorldTerrain(),
      requestVertexNormals: true,
      animation: false,
      timeline: false,
      homeButton: false,
      navigationHelpButton: false,
      selectionIndicator: false,
      infoBox: false,
    });

    this.viewer.screenSpaceEventHandler.removeInputAction(Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);


    this.scene = this.viewer.scene;
    this.scene.skyBox.destroy();
    this.scene.skyBox = undefined;
    this.scene.sun.destroy();
    this.scene.sun = undefined;
    this.scene.backgroundColor = Cesium.Color.BLACK.clone();
    this.scene.moon.destroy();
    this.scene.moon = undefined;
  }
}
